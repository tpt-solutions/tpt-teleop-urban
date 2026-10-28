//! Hash-chained, append-only, Ed25519-signed audit ledger.
//!
//! Each entry commits to its predecessor's hash, so editing, dropping or
//! reordering any entry breaks the chain at that point. Each entry is also signed,
//! so a forged entry needs the signing key, not just the hash function.
//!
//! This type keeps the chain in memory. [`crate::store::DurableLedger`] wraps it
//! with power-loss-safe file storage (spec §5.4).

use ring::digest::{SHA256, digest};
use ring::signature::{ED25519, Ed25519KeyPair, KeyPair, UnparsedPublicKey};

/// Hash of the (nonexistent) entry before the first one.
pub const GENESIS_HASH: [u8; 32] = [0; 32];

/// One signed, chained record.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Entry {
    /// Position in the chain, from zero.
    pub seq: u64,
    /// Hash of the entry before this one, or `GENESIS_HASH` for the first.
    pub prev_hash: [u8; 32],
    /// The recorded payload.
    pub body: Vec<u8>,
    /// Ed25519 signature over the sequence number, previous hash and body.
    pub signature: [u8; 64],
    /// SHA-256 of the signed message and its signature.
    pub hash: [u8; 32],
}

/// Why `verify_chain` rejected the ledger.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VerifyError {
    /// Entry `seq` does not carry the sequence number of its position.
    SequenceGap {
        /// The entry whose sequence number is wrong.
        seq: u64,
    },
    /// Entry `seq` does not link to the hash of the entry before it.
    BrokenLink {
        /// The entry that does not link.
        seq: u64,
    },
    /// Entry `seq` has a signature that does not verify.
    BadSignature {
        /// The entry whose signature fails.
        seq: u64,
    },
    /// Entry `seq` has a stored hash that does not match its contents.
    BadHash {
        /// The entry whose hash does not match.
        seq: u64,
    },
}

/// Append-only ledger that signs every entry with one Ed25519 key.
pub struct Ledger {
    signer: Ed25519KeyPair,
    entries: Vec<Entry>,
}

impl Ledger {
    #[must_use]
    /// A ledger that signs with `signer`, starting empty.
    pub fn new(signer: Ed25519KeyPair) -> Self {
        Self {
            signer,
            entries: Vec::new(),
        }
    }

    /// Public half of the signing key, for verifiers.
    #[must_use]
    pub fn public_key(&self) -> [u8; 32] {
        let mut out = [0u8; 32];
        out.copy_from_slice(self.signer.public_key().as_ref());
        out
    }

    /// Appends `body` as the next entry and returns it.
    pub fn append(&mut self, body: &[u8]) -> &Entry {
        let entry = self.next_entry(body);
        self.push(entry);
        self.entries.last().expect("entry was just pushed")
    }

    /// Signs `body` as the entry after the current tail, without storing it.
    pub(crate) fn next_entry(&self, body: &[u8]) -> Entry {
        let seq = self.entries.len() as u64;
        let prev_hash = self.entries.last().map_or(GENESIS_HASH, |e| e.hash);
        let message = signing_input(seq, &prev_hash, body);
        let mut signature = [0u8; 64];
        signature.copy_from_slice(self.signer.sign(&message).as_ref());
        let hash = entry_hash(&message, &signature);
        Entry {
            seq,
            prev_hash,
            body: body.to_vec(),
            signature,
            hash,
        }
    }

    /// Stores an entry produced by [`Ledger::next_entry`]. Callers must push in
    /// order, which durable storage does after the entry is on disk.
    pub(crate) fn push(&mut self, entry: Entry) {
        self.entries.push(entry);
    }

    /// Rebuilds a ledger from entries that were already verified.
    pub(crate) fn restore(signer: Ed25519KeyPair, entries: Vec<Entry>) -> Self {
        Self { signer, entries }
    }

    #[must_use]
    /// Every stored entry, in order.
    pub fn entries(&self) -> &[Entry] {
        &self.entries
    }
}

impl VerifyError {
    /// The sequence number of the entry that failed.
    #[must_use]
    pub const fn seq(self) -> u64 {
        match self {
            Self::SequenceGap { seq }
            | Self::BrokenLink { seq }
            | Self::BadSignature { seq }
            | Self::BadHash { seq } => seq,
        }
    }
}

/// Checks every entry's sequence number, link, hash and signature.
pub fn verify_chain(public_key: &[u8; 32], entries: &[Entry]) -> Result<(), VerifyError> {
    let key = UnparsedPublicKey::new(&ED25519, public_key);
    let mut expected_prev = GENESIS_HASH;
    for (index, entry) in entries.iter().enumerate() {
        let seq = index as u64;
        if entry.seq != seq {
            return Err(VerifyError::SequenceGap { seq: entry.seq });
        }
        if entry.prev_hash != expected_prev {
            return Err(VerifyError::BrokenLink { seq });
        }
        let message = signing_input(seq, &entry.prev_hash, &entry.body);
        if key.verify(&message, &entry.signature).is_err() {
            return Err(VerifyError::BadSignature { seq });
        }
        if entry_hash(&message, &entry.signature) != entry.hash {
            return Err(VerifyError::BadHash { seq });
        }
        expected_prev = entry.hash;
    }
    Ok(())
}

/// Bytes covered by the signature: sequence number, previous hash, body.
fn signing_input(seq: u64, prev_hash: &[u8; 32], body: &[u8]) -> Vec<u8> {
    let mut message = Vec::with_capacity(8 + 32 + body.len());
    message.extend_from_slice(&seq.to_be_bytes());
    message.extend_from_slice(prev_hash);
    message.extend_from_slice(body);
    message
}

/// SHA-256 over the signed message and its signature, so the hash binds both.
fn entry_hash(message: &[u8], signature: &[u8; 64]) -> [u8; 32] {
    let mut input = Vec::with_capacity(message.len() + 64);
    input.extend_from_slice(message);
    input.extend_from_slice(signature);
    let mut out = [0u8; 32];
    out.copy_from_slice(digest(&SHA256, &input).as_ref());
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use ring::rand::SystemRandom;

    fn new_ledger() -> Ledger {
        let pkcs8 = Ed25519KeyPair::generate_pkcs8(&SystemRandom::new()).expect("keygen");
        Ledger::new(Ed25519KeyPair::from_pkcs8(pkcs8.as_ref()).expect("pkcs8"))
    }

    #[test]
    fn sha256_matches_the_fips_test_vector() {
        // FIPS 180-4 example: SHA-256("abc").
        let mut out = [0u8; 32];
        out.copy_from_slice(digest(&SHA256, b"abc").as_ref());
        let expected = [
            0xba, 0x78, 0x16, 0xbf, 0x8f, 0x01, 0xcf, 0xea, 0x41, 0x41, 0x40, 0xde, 0x5d, 0xae,
            0x22, 0x23, 0xb0, 0x03, 0x61, 0xa3, 0x96, 0x17, 0x7a, 0x9c, 0xb4, 0x10, 0xff, 0x61,
            0xf2, 0x00, 0x15, 0xad,
        ];
        assert_eq!(out, expected);
    }

    #[test]
    fn appended_chain_verifies() {
        let mut ledger = new_ledger();
        ledger.append(b"yield: vehicle on right");
        ledger.append(b"proceed: right-of-way clear");
        ledger.append(b"stop: at stop line");
        let key = ledger.public_key();
        assert_eq!(verify_chain(&key, ledger.entries()), Ok(()));
        assert_eq!(ledger.entries()[0].prev_hash, GENESIS_HASH);
        assert_eq!(ledger.entries()[1].prev_hash, ledger.entries()[0].hash);
    }

    #[test]
    fn tampered_body_is_detected() {
        let mut ledger = new_ledger();
        ledger.append(b"proceed");
        ledger.append(b"stop");
        let key = ledger.public_key();
        let mut entries = ledger.entries().to_vec();
        entries[0].body = b"yield".to_vec();
        assert_eq!(
            verify_chain(&key, &entries),
            Err(VerifyError::BadSignature { seq: 0 })
        );
    }

    #[test]
    fn dropped_entry_breaks_the_link() {
        let mut ledger = new_ledger();
        ledger.append(b"a");
        ledger.append(b"b");
        ledger.append(b"c");
        let key = ledger.public_key();
        let mut entries = ledger.entries().to_vec();
        entries.remove(1);
        assert_eq!(
            verify_chain(&key, &entries),
            Err(VerifyError::SequenceGap { seq: 2 })
        );
    }

    #[test]
    fn forged_signature_with_other_key_is_rejected() {
        let mut ledger = new_ledger();
        ledger.append(b"proceed");
        let other = new_ledger().public_key();
        assert_eq!(
            verify_chain(&other, ledger.entries()),
            Err(VerifyError::BadSignature { seq: 0 })
        );
    }

    #[test]
    fn stored_hash_mismatch_is_detected() {
        let mut ledger = new_ledger();
        ledger.append(b"proceed");
        let key = ledger.public_key();
        let mut entries = ledger.entries().to_vec();
        entries[0].hash[0] ^= 0x01;
        assert_eq!(
            verify_chain(&key, &entries),
            Err(VerifyError::BadHash { seq: 0 })
        );
    }
}
