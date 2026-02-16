/// Packet checksum implementation
pub struct Checksum {
    a: u8,
    b: u8,
}

impl Checksum {
    pub fn new() -> Self {
        Self { a: 0, b: 0 }
    }

    pub fn update(&mut self, bytes: &[u8]) {
        for b in bytes {
            self.a = self.a.overflowing_add(*b).0;
            self.b = self.b.overflowing_add(self.a).0;
        }
    }

    pub fn value(&self) -> u16 {
        ((self.a as u16) << 8) + self.b as u16
    }

    pub fn calc_checksum(bytes: &[u8]) -> u16 {
        let mut chk = Self::new();
        chk.update(bytes);
        chk.value()
    }
}
