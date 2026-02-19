//! MIP packet checksum utilities
//!
//! This is fletcher-like, but without the mod255 performed by some many fletcher16 implementations.

/// Packet checksum implementation
pub struct Checksum {
    a: u8,
    b: u8,
}

impl Default for Checksum {
    fn default() -> Self {
        Self::new()
    }
}

impl Checksum {
    /// Create a new checksum calculator
    pub fn new() -> Self {
        Self { a: 0, b: 0 }
    }

    /// Update the checksum value with more bytes
    pub fn update(&mut self, bytes: &[u8]) {
        for b in bytes {
            self.a = self.a.overflowing_add(*b).0;
            self.b = self.b.overflowing_add(self.a).0;
        }
    }

    /// Get the current checksum value
    pub fn value(&self) -> u16 {
        ((self.a as u16) << 8) + self.b as u16
    }
}

/// Compute a checksum over a slice of bytes
pub fn calc_checksum(bytes: &[u8]) -> u16 {
    let mut chk = Checksum::new();
    chk.update(bytes);
    chk.value()
}
