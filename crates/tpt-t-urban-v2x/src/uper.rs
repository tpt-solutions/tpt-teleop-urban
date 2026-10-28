//! Unaligned PER (ITU-T X.691 UPER) decoding primitives.
//!
//! Each function cites the X.691 clause it implements (edition 02/2021). The
//! UNALIGNED variant never pads to octet boundaries, so every field is read at
//! the current bit position. Fragmented lengths (16K and above) are rejected as
//! `Unsupported`; no V2X core message needs them.
//!
//! The reader borrows its input and never allocates.

/// Why a decode failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UperError {
    /// The input ended before the field was complete.
    EndOfInput,
    /// A decoded value falls outside the constrained range.
    OutOfRange,
    /// The encoding uses a feature this decoder does not implement.
    Unsupported,
}

/// Result of a UPER decode step.
pub type Result<T> = core::result::Result<T, UperError>;

/// Reads bit fields from a byte slice, most significant bit first.
pub struct BitReader<'a> {
    bytes: &'a [u8],
    /// Bit position from the start of `bytes`.
    pos: usize,
}

impl<'a> BitReader<'a> {
    #[must_use]
    /// A reader positioned at the first bit of `bytes`.
    pub const fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, pos: 0 }
    }

    /// Bits not yet consumed.
    #[must_use]
    pub const fn bits_remaining(&self) -> usize {
        self.bytes.len() * 8 - self.pos
    }

    /// X.691 11.3: reads `n` bits as a non-negative binary integer, most
    /// significant bit first. `n` must be at most 64.
    pub fn read_bits(&mut self, n: u32) -> Result<u64> {
        if n > 64 {
            return Err(UperError::Unsupported);
        }
        if n as usize > self.bits_remaining() {
            return Err(UperError::EndOfInput);
        }
        let mut value = 0u64;
        for _ in 0..n {
            let byte = self.bytes[self.pos >> 3];
            let bit = (byte >> (7 - (self.pos & 7))) & 1;
            value = (value << 1) | u64::from(bit);
            self.pos += 1;
        }
        Ok(value)
    }

    /// X.691 12: a BOOLEAN is one bit, 1 for TRUE.
    pub fn read_bool(&mut self) -> Result<bool> {
        Ok(self.read_bits(1)? == 1)
    }

    /// X.691 11.5 (UNALIGNED): a constrained whole number in `lb..=ub`, sent as
    /// the offset from `lb` in the minimum number of bits for the range.
    pub fn constrained_whole(&mut self, lb: i64, ub: i64) -> Result<i64> {
        let range = constrained_range(lb, ub)?;
        let offset = self.read_bits(bits_for_range(range))?;
        if offset >= range {
            return Err(UperError::OutOfRange);
        }
        i64::try_from(i128::from(lb) + i128::from(offset)).map_err(|_| UperError::OutOfRange)
    }

    /// X.691 14: an enumerated index. Without an extension marker the index is a
    /// constrained whole number over the root. With one, a leading bit selects
    /// the root or an extension addition.
    pub fn enumerated(&mut self, root_count: u64, extensible: bool) -> Result<Enumerated> {
        if root_count == 0 {
            return Err(UperError::Unsupported);
        }
        if extensible && self.read_bool()? {
            return self.normally_small().map(Enumerated::Extension);
        }
        let index = self.constrained_whole(0, (root_count - 1) as i64)?;
        Ok(Enumerated::Root(index as u64))
    }

    /// X.691 11.6: a normally small non-negative whole number. Values of 63 or
    /// less are a 0 bit then six bits. Larger values are not needed here.
    pub fn normally_small(&mut self) -> Result<u64> {
        if self.read_bool()? {
            return Err(UperError::Unsupported);
        }
        self.read_bits(6)
    }

    /// X.691 11.9.4 (UNALIGNED): a length determinant. `ub` is the upper bound
    /// when the type has one.
    ///
    /// A bounded length below 64K uses a constrained whole number. Otherwise the
    /// length takes one bit-field of 7 bits (value below 128) or 14 bits (value
    /// below 16K). Longer lengths need fragmentation, which is rejected.
    pub fn length(&mut self, ub: Option<u64>) -> Result<u64> {
        match ub {
            Some(ub) if ub < 65_536 => {
                // Unbounded-by-lb lengths use lb = 0 for these types.
                let len = self.constrained_whole(0, ub as i64)?;
                Ok(len as u64)
            }
            _ => {
                if !self.read_bool()? {
                    return self.read_bits(7);
                }
                if !self.read_bool()? {
                    return self.read_bits(14);
                }
                Err(UperError::Unsupported)
            }
        }
    }

    /// X.691 17.6 to 17.7: an octet string of fixed length `N` has no length
    /// determinant. Its octets are unaligned in UPER.
    pub fn fixed_octets<const N: usize>(&mut self) -> Result<[u8; N]> {
        let mut out = [0u8; N];
        for octet in &mut out {
            *octet = self.read_bits(8)? as u8;
        }
        Ok(out)
    }
}

/// Result of an enumerated decode: a root value or an extension addition.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Enumerated {
    /// A root enumerator, by its index.
    Root(
        /// The index of the root enumerator.
        u64,
    ),
    /// An extension addition, by its index.
    Extension(
        /// The index of the extension addition.
        u64,
    ),
}

/// Number of values in `lb..=ub`, or an error if the bounds are inverted or the
/// range does not fit in 64 bits.
fn constrained_range(lb: i64, ub: i64) -> Result<u64> {
    if ub < lb {
        return Err(UperError::Unsupported);
    }
    u64::try_from(i128::from(ub) - i128::from(lb) + 1).map_err(|_| UperError::Unsupported)
}

/// X.691 11.5.6 and its table (11.5.7.1): the bit count for a range.
///
/// A range of 1 has zero bits. Otherwise the count is ceil(log2(range)), which
/// is `64 - leading_zeros(range - 1)`.
#[must_use]
pub const fn bits_for_range(range: u64) -> u32 {
    if range <= 1 {
        0
    } else {
        64 - (range - 1).leading_zeros()
    }
}

/// Writes UPER fields, most significant bit first. The inverse of [`BitReader`].
///
/// A field that does not fit its constraint is refused, so an encoder cannot
/// produce a message the decoder would reject.
#[derive(Clone, Debug, Default)]
pub struct BitWriter {
    bytes: Vec<u8>,
    pos: usize,
}

impl BitWriter {
    /// An empty writer.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            bytes: Vec::new(),
            pos: 0,
        }
    }

    /// Bits written so far.
    #[must_use]
    pub const fn bit_len(&self) -> usize {
        self.pos
    }

    /// Writes the low `n` bits of `value`, most significant first. `n` must be at
    /// most 64, and `value` must fit in `n` bits.
    ///
    /// # Errors
    ///
    /// Returns [`UperError::Unsupported`] for `n > 64`, and
    /// [`UperError::OutOfRange`] if `value` does not fit.
    pub fn write_bits(&mut self, value: u64, n: u32) -> Result<()> {
        if n > 64 {
            return Err(UperError::Unsupported);
        }
        if n < 64 && value >> n != 0 {
            return Err(UperError::OutOfRange);
        }
        for i in (0..n).rev() {
            if self.pos.is_multiple_of(8) {
                self.bytes.push(0);
            }
            if (value >> i) & 1 == 1 {
                let last = self.bytes.len() - 1;
                self.bytes[last] |= 0x80 >> (self.pos % 8);
            }
            self.pos += 1;
        }
        Ok(())
    }

    /// X.691 12: one bit, 1 for TRUE.
    pub fn write_bool(&mut self, value: bool) {
        // One bit always fits, so this cannot fail.
        let _ = self.write_bits(u64::from(value), 1);
    }

    /// X.691 11.5 (UNALIGNED): a constrained whole number in `lb..=ub`, sent as
    /// the offset from `lb` in the minimum number of bits for the range.
    ///
    /// # Errors
    ///
    /// Returns [`UperError::OutOfRange`] if `value` is outside `lb..=ub`, and
    /// [`UperError::Unsupported`] for inverted bounds.
    pub fn write_constrained_whole(&mut self, value: i64, lb: i64, ub: i64) -> Result<()> {
        let range = constrained_range(lb, ub)?;
        if value < lb || value > ub {
            return Err(UperError::OutOfRange);
        }
        let offset = (i128::from(value) - i128::from(lb)) as u64;
        self.write_bits(offset, bits_for_range(range))
    }

    /// X.691 17.6 to 17.7: fixed-length octets, with no length determinant.
    ///
    /// # Errors
    ///
    /// Never fails for octets, but returns the writer's error type for
    /// consistency with the other fields.
    pub fn write_octets(&mut self, octets: &[u8]) -> Result<()> {
        for &octet in octets {
            self.write_bits(u64::from(octet), 8)?;
        }
        Ok(())
    }

    /// The bytes written, with the last byte padded with zero bits.
    #[must_use]
    pub fn into_bytes(self) -> Vec<u8> {
        self.bytes
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bit_count_follows_the_x691_table() {
        // X.691 11.5.7.1 table: range -> bit-field size.
        let cases = [
            (1, 0),
            (2, 1),
            (3, 2),
            (4, 2),
            (5, 3),
            (8, 3),
            (9, 4),
            (16, 4),
            (17, 5),
            (32, 5),
            (33, 6),
            (64, 6),
            (65, 7),
            (128, 7),
            (129, 8),
            (255, 8),
            (256, 8),
            (4002, 12),
            (65536, 16),
        ];
        for (range, bits) in cases {
            assert_eq!(bits_for_range(range), bits, "range {range}");
        }
    }

    #[test]
    fn constrained_whole_reads_offset_from_lower_bound() {
        // range 256 (10..=265): one octet holding 15, so the value is 25.
        let mut r = BitReader::new(&[0x0F]);
        assert_eq!(r.constrained_whole(10, 265), Ok(25));
        // range 2 (0..=1): a single bit.
        let mut r = BitReader::new(&[0x80]);
        assert_eq!(r.constrained_whole(0, 1), Ok(1));
    }

    #[test]
    fn constrained_whole_rejects_offsets_past_the_range() {
        // range 3 (0..=2) uses 2 bits, so offset 3 is representable but invalid.
        let mut r = BitReader::new(&[0b1100_0000]);
        assert_eq!(r.constrained_whole(0, 2), Err(UperError::OutOfRange));
    }

    #[test]
    fn length_determinant_one_octet_and_two_octet_forms() {
        // 5 -> 0 0000101
        assert_eq!(BitReader::new(&[0x05]).length(None), Ok(5));
        // 200 -> 10 then 14 bits of 200
        assert_eq!(BitReader::new(&[0x80, 0xC8]).length(None), Ok(200));
    }

    #[test]
    fn bounded_length_uses_constrained_whole_number() {
        // ub 3 -> range 4 -> 2 bits; value 2 is 10.
        let mut r = BitReader::new(&[0b1000_0000]);
        assert_eq!(r.length(Some(3)), Ok(2));
    }

    #[test]
    fn fixed_octets_are_unaligned() {
        // Three unaligned bits, then a 16-bit field that straddles the byte edge.
        let mut r = BitReader::new(&[0b1010_1011, 0b1100_0000, 0x00]);
        assert_eq!(r.read_bits(3), Ok(0b101));
        assert_eq!(r.fixed_octets::<2>(), Ok([0b0101_1110, 0b0000_0000]));
    }

    #[test]
    fn enumerated_with_extension_marker() {
        // extension bit 0, then root index in ceil(log2(4)) = 2 bits: 3.
        let mut r = BitReader::new(&[0b0110_0000]);
        assert_eq!(r.enumerated(4, true), Ok(Enumerated::Root(3)));
        // extension bit 1, normally small bit 0, 6-bit index 2 (000010).
        let mut r = BitReader::new(&[0b1000_0010]);
        assert_eq!(r.enumerated(4, true), Ok(Enumerated::Extension(2)));
    }

    #[test]
    fn end_of_input_is_reported() {
        let mut r = BitReader::new(&[0xFF]);
        assert_eq!(r.read_bits(9), Err(UperError::EndOfInput));
    }

    #[test]
    fn writer_and_reader_agree_on_mixed_widths() {
        let mut w = BitWriter::new();
        w.write_constrained_whole(-2000, -2000, 2001).unwrap();
        w.write_bits(0b101, 3).unwrap();
        w.write_constrained_whole(7, 0, 9).unwrap();
        let bytes = w.into_bytes();

        let mut r = BitReader::new(&bytes);
        assert_eq!(r.constrained_whole(-2000, 2001), Ok(-2000));
        assert_eq!(r.read_bits(3), Ok(0b101));
        assert_eq!(r.constrained_whole(0, 9), Ok(7));
    }

    /// Drives every primitive with random bytes and random bounds, including
    /// extreme ones. Each call must return a value or an error, never panic,
    /// and every decoded value must respect its bounds. The bit cursor must
    /// never move past the end of the input.
    #[test]
    fn random_frames_never_panic_and_decoded_values_are_in_range() {
        let mut state = 0x9E37_79B9_u32;
        let mut next = || {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            state
        };
        let bounds = [
            (0, 0),
            (0, 1),
            (-2000, 2001),
            (0, 63),
            (i64::MIN, i64::MAX),
            (i64::MIN, i64::MIN + 1),
            (i64::MAX - 3, i64::MAX),
            (5, 4),
        ];
        let mut buf = [0u8; 24];
        for _ in 0..5_000 {
            for b in &mut buf {
                *b = next() as u8;
            }
            for &(lb, ub) in &bounds {
                let mut r = BitReader::new(&buf);
                if let Ok(v) = r.constrained_whole(lb, ub) {
                    assert!((lb..=ub).contains(&v));
                }
                assert!(r.bits_remaining() <= buf.len() * 8);
            }

            let mut r = BitReader::new(&buf);
            let _ = r.enumerated(3, true);
            let _ = r.enumerated(1, false);
            let _ = r.normally_small();
            let _ = r.length(Some(1000));
            let _ = r.length(None);
            let _ = r.fixed_octets::<5>();
            assert!(r.bits_remaining() <= buf.len() * 8);

            let mut r = BitReader::new(&buf);
            let n = next() % 70;
            if let Ok(v) = r.read_bits(n) {
                assert!(n == 64 || v < (1_u64 << n));
            }
        }
    }
}
