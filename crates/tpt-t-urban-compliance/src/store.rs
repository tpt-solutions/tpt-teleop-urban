//! Durable, power-loss-safe storage for the audit ledger (spec §5.4).
//!
//! Each entry is written as one length-prefixed record, and the file is synced
//! before [`DurableLedger::append`] returns. An entry the caller saw committed
//! therefore survives power loss.
//!
//! A power cut during a write leaves a torn record at the end of the file. On
//! open, that tail is cut off. A last record that does not verify is also cut
//! off, since the write that produced it never completed. Both cases are
//! reported in [`Recovery`]. A bad record with valid records after it is not a
//! crash. It is corruption or tampering, so the file is refused, not repaired.
//!
//! Record layout, all integers big-endian:
//!
//! | field       | bytes                    |
//! |-------------|--------------------------|
//! | body length | 4                        |
//! | seq         | 8                        |
//! | prev hash   | 32                       |
//! | signature   | 64                       |
//! | hash        | 32                       |
//! | body        | body length              |

use std::fs::{File, OpenOptions};
use std::io::{self, Read, Seek, SeekFrom, Write};
use std::path::Path;

use ring::signature::{Ed25519KeyPair, KeyPair};

use crate::ledger::{Entry, Ledger, VerifyError, verify_chain};

/// Bytes in a record before its body: the length prefix and the fixed fields.
const RECORD_HEADER_LEN: usize = 4 + 8 + 32 + 64 + 32;

/// Why the store could not be opened or written.
#[derive(Debug)]
pub enum StoreError {
    /// The file could not be read, written or synced.
    Io(io::Error),
    /// A record in the middle of the file fails verification. The file was
    /// not modified.
    Corrupt(VerifyError),
    /// The body is longer than a record can describe, which is 4 GiB.
    BodyTooLarge,
}

impl From<io::Error> for StoreError {
    fn from(error: io::Error) -> Self {
        Self::Io(error)
    }
}

/// What [`DurableLedger::open`] did to the file to make it consistent.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Recovery {
    /// Entries kept from the file.
    pub recovered: usize,
    /// Bytes cut from the end of the file: a torn write, or a final record that
    /// did not verify.
    pub discarded_bytes: u64,
}

/// A [`Ledger`] whose entries are also stored in a file, synced on every append.
pub struct DurableLedger {
    file: File,
    ledger: Ledger,
}

impl DurableLedger {
    /// Opens the ledger at `path`, creating the file if it does not exist.
    ///
    /// The stored entries are verified against `signer`'s public key. A torn or
    /// unverifiable final record is discarded, and the discard is reported in the
    /// returned [`Recovery`].
    pub fn open(path: &Path, signer: Ed25519KeyPair) -> Result<(Self, Recovery), StoreError> {
        // Not `append(true)`: on Windows an append-only handle cannot truncate.
        let mut file = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            // Never truncate on open: the stored entries are the ledger.
            .truncate(false)
            .open(path)?;
        let mut bytes = Vec::new();
        file.read_to_end(&mut bytes)?;

        let mut records = parse_records(&bytes);
        let mut public_key = [0u8; 32];
        public_key.copy_from_slice(signer.public_key().as_ref());

        let entries = |records: &[(Entry, usize)]| -> Vec<Entry> {
            records.iter().map(|(entry, _)| entry.clone()).collect()
        };
        if let Err(error) = verify_chain(&public_key, &entries(&records)) {
            // Only a final record that fails after a verified prefix is treated as
            // a torn write. Anything else (an earlier failure, or a single record
            // that fails, as with the wrong key) means the file is not ours to
            // repair, so it is refused and left as it is.
            if records.len() < 2 || error.seq() + 1 != records.len() as u64 {
                return Err(StoreError::Corrupt(error));
            }
            records.pop();
            verify_chain(&public_key, &entries(&records)).map_err(StoreError::Corrupt)?;
        }

        let good_len = records.last().map_or(0, |(_, end)| *end);
        let discarded_bytes = (bytes.len() - good_len) as u64;
        if discarded_bytes > 0 {
            file.set_len(good_len as u64)?;
            file.sync_all()?;
        }
        // Writes go to the end of the file, which the cut may have moved.
        file.seek(SeekFrom::End(0))?;

        let recovered = records.len();
        let entries = records.into_iter().map(|(entry, _)| entry).collect();
        let ledger = Ledger::restore(signer, entries);
        Ok((
            Self { file, ledger },
            Recovery {
                recovered,
                discarded_bytes,
            },
        ))
    }

    /// Signs and stores `body` as the next entry. Returns once it is on disk.
    ///
    /// If the write or sync fails, the entry is not added to the ledger.
    pub fn append(&mut self, body: &[u8]) -> Result<&Entry, StoreError> {
        let entry = self.ledger.next_entry(body);
        let record = encode_record(&entry)?;
        self.file.write_all(&record)?;
        self.file.sync_data()?;
        self.ledger.push(entry);
        Ok(self.ledger.entries().last().expect("entry was just pushed"))
    }

    /// Every stored entry, in order.
    #[must_use]
    pub fn entries(&self) -> &[Entry] {
        self.ledger.entries()
    }

    /// Public half of the signing key, for verifiers.
    #[must_use]
    pub fn public_key(&self) -> [u8; 32] {
        self.ledger.public_key()
    }
}

/// Reads every complete record in `bytes`. Returns each entry with the offset
/// just past its record. Parsing stops at the first record that is incomplete.
fn parse_records(bytes: &[u8]) -> Vec<(Entry, usize)> {
    let mut records = Vec::new();
    let mut at = 0;
    while bytes.len() - at >= 4 {
        let body_len = u32::from_be_bytes(bytes[at..at + 4].try_into().expect("4 bytes")) as usize;
        let end = at + RECORD_HEADER_LEN + body_len;
        if end > bytes.len() {
            break;
        }
        let record = &bytes[at + 4..end];
        records.push((
            Entry {
                seq: u64::from_be_bytes(record[0..8].try_into().expect("8 bytes")),
                prev_hash: record[8..40].try_into().expect("32 bytes"),
                signature: record[40..104].try_into().expect("64 bytes"),
                hash: record[104..136].try_into().expect("32 bytes"),
                body: record[136..].to_vec(),
            },
            end,
        ));
        at = end;
    }
    records
}

/// The bytes of one record, ready to write in a single call.
fn encode_record(entry: &Entry) -> Result<Vec<u8>, StoreError> {
    let body_len = u32::try_from(entry.body.len()).map_err(|_| StoreError::BodyTooLarge)?;
    let mut out = Vec::with_capacity(RECORD_HEADER_LEN + entry.body.len());
    out.extend_from_slice(&body_len.to_be_bytes());
    out.extend_from_slice(&entry.seq.to_be_bytes());
    out.extend_from_slice(&entry.prev_hash);
    out.extend_from_slice(&entry.signature);
    out.extend_from_slice(&entry.hash);
    out.extend_from_slice(&entry.body);
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use ring::rand::SystemRandom;
    use std::path::PathBuf;

    /// A PKCS#8 key shared by every open in one test, so the public key matches
    /// across reopens.
    fn pkcs8() -> Vec<u8> {
        Ed25519KeyPair::generate_pkcs8(&SystemRandom::new())
            .expect("keygen")
            .as_ref()
            .to_vec()
    }

    fn signer(pkcs8: &[u8]) -> Ed25519KeyPair {
        Ed25519KeyPair::from_pkcs8(pkcs8).expect("pkcs8")
    }

    /// A file path unique to this test and process, removed on drop.
    struct TempFile(PathBuf);

    impl TempFile {
        fn new(name: &str) -> Self {
            let path = std::env::temp_dir().join(format!(
                "tpt-ledger-{}-{}.log",
                std::process::id(),
                name
            ));
            let _ = std::fs::remove_file(&path);
            Self(path)
        }
    }

    impl Drop for TempFile {
        fn drop(&mut self) {
            let _ = std::fs::remove_file(&self.0);
        }
    }

    #[test]
    fn entries_survive_a_reopen_and_the_chain_continues() {
        let file = TempFile::new("reopen");
        let key = pkcs8();
        {
            let (mut ledger, recovery) = DurableLedger::open(&file.0, signer(&key)).unwrap();
            assert_eq!(
                recovery,
                Recovery {
                    recovered: 0,
                    discarded_bytes: 0
                }
            );
            ledger.append(b"yield: vehicle on right").unwrap();
            ledger.append(b"proceed: right-of-way clear").unwrap();
        }
        let (mut ledger, recovery) = DurableLedger::open(&file.0, signer(&key)).unwrap();
        assert_eq!(
            recovery,
            Recovery {
                recovered: 2,
                discarded_bytes: 0
            }
        );
        assert_eq!(ledger.entries().len(), 2);
        ledger.append(b"stop: at stop line").unwrap();

        let public_key = ledger.public_key();
        assert_eq!(verify_chain(&public_key, ledger.entries()), Ok(()));
        assert_eq!(ledger.entries()[2].seq, 2);
    }

    #[test]
    fn a_torn_write_at_the_tail_is_cut_off() {
        let file = TempFile::new("torn");
        let key = pkcs8();
        let full_len = {
            let (mut ledger, _) = DurableLedger::open(&file.0, signer(&key)).unwrap();
            ledger.append(b"first").unwrap();
            ledger.append(b"second").unwrap();
            ledger.append(b"third, interrupted").unwrap();
            std::fs::metadata(&file.0).unwrap().len()
        };
        // Simulate a power cut partway through the last record.
        let torn = OpenOptions::new().write(true).open(&file.0).unwrap();
        torn.set_len(full_len - 7).unwrap();
        drop(torn);

        let (ledger, recovery) = DurableLedger::open(&file.0, signer(&key)).unwrap();
        assert_eq!(recovery.recovered, 2);
        assert!(recovery.discarded_bytes > 0);
        assert_eq!(ledger.entries().len(), 2);
        assert_eq!(
            std::fs::metadata(&file.0).unwrap().len(),
            full_len - 7 - recovery.discarded_bytes
        );

        // The cut is durable: the next open finds a clean file.
        drop(ledger);
        let (_, again) = DurableLedger::open(&file.0, signer(&key)).unwrap();
        assert_eq!(
            again,
            Recovery {
                recovered: 2,
                discarded_bytes: 0
            }
        );
    }

    #[test]
    fn a_single_bad_record_is_refused_not_erased() {
        // With one record there is no verified prefix, so the failure may be a
        // wrong key rather than a torn write. The file must not be truncated.
        let file = TempFile::new("single-bad");
        let key = pkcs8();
        {
            let (mut ledger, _) = DurableLedger::open(&file.0, signer(&key)).unwrap();
            ledger.append(b"proceed").unwrap();
        }
        let before = std::fs::read(&file.0).unwrap();
        let result = DurableLedger::open(&file.0, signer(&pkcs8()));
        assert!(matches!(result, Err(StoreError::Corrupt(_))));
        assert_eq!(std::fs::read(&file.0).unwrap(), before);
    }

    #[test]
    fn a_final_record_that_does_not_verify_is_discarded() {
        let file = TempFile::new("last-bad");
        let key = pkcs8();
        {
            let (mut ledger, _) = DurableLedger::open(&file.0, signer(&key)).unwrap();
            ledger.append(b"proceed").unwrap();
            ledger.append(b"stop").unwrap();
        }
        // Flip the last byte of the file, which is inside the final record's body.
        let mut bytes = std::fs::read(&file.0).unwrap();
        let last = bytes.len() - 1;
        bytes[last] ^= 0x01;
        std::fs::write(&file.0, &bytes).unwrap();

        let (ledger, recovery) = DurableLedger::open(&file.0, signer(&key)).unwrap();
        assert_eq!(recovery.recovered, 1);
        assert_eq!(recovery.discarded_bytes as usize, RECORD_HEADER_LEN + 4);
        assert_eq!(ledger.entries().len(), 1);
    }

    #[test]
    fn corruption_in_the_middle_refuses_to_open() {
        let file = TempFile::new("middle");
        let key = pkcs8();
        {
            let (mut ledger, _) = DurableLedger::open(&file.0, signer(&key)).unwrap();
            ledger.append(b"proceed").unwrap();
            ledger.append(b"stop").unwrap();
            ledger.append(b"proceed").unwrap();
        }
        // Flip a byte in the first record's body, with valid records after it.
        let mut bytes = std::fs::read(&file.0).unwrap();
        bytes[RECORD_HEADER_LEN] ^= 0x01;
        std::fs::write(&file.0, &bytes).unwrap();
        let before = std::fs::read(&file.0).unwrap();

        let result = DurableLedger::open(&file.0, signer(&key));
        assert!(matches!(
            result,
            Err(StoreError::Corrupt(VerifyError::BadSignature { seq: 0 }))
        ));
        assert_eq!(
            std::fs::read(&file.0).unwrap(),
            before,
            "file must not be modified"
        );
    }
}
