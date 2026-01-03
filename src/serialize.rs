const CRC_SIZE: usize = 2;
const MAX_PAYLOAD_SIZE: usize = 255;
const HEADER_SIZE: usize = 4;

const BUF_SIZE: usize = HEADER_SIZE + MAX_PAYLOAD_SIZE + CRC_SIZE;

pub struct OwnedMessage {
    buf: [u8; BUF_SIZE],
}

impl OwnedMessage {
    const SYNC1: u8 = 0x75;
    const SYNC2: u8 = 0x65;

    pub fn new(descriptor_set: u8, payload: &[u8]) -> Self {
        assert!(payload.len() <= MAX_PAYLOAD_SIZE);
        let mut buf = [0; BUF_SIZE];
        buf[0] = Self::SYNC1;
        buf[1] = Self::SYNC2;
        buf[2] = descriptor_set;
        buf[3] = payload.len() as u8;
        buf[HEADER_SIZE..HEADER_SIZE + payload.len()].copy_from_slice(payload);
        Self { buf }
    }

    fn checksum(&mut self) {
        let payload_len = self.payload_len() as usize;
        let chk = fletcher::calc_fletcher16(&self.buf[0..payload_len + HEADER_SIZE]);
        self.buf[payload_len + HEADER_SIZE..payload_len + HEADER_SIZE + CRC_SIZE]
            .copy_from_slice(&chk.to_be_bytes());
    }

    pub fn as_slice(&mut self) -> &[u8] {
        self.checksum();
        &self.buf[..self.payload_len() as usize + HEADER_SIZE + CRC_SIZE]
    }

    pub fn payload(&self) -> &[u8] {
        &self.buf[HEADER_SIZE..self.payload_len() as usize + HEADER_SIZE]
    }

    pub fn payload_len(&self) -> u8 {
        self.buf[3]
    }

    pub fn descriptor_set(&self) -> u8 {
        self.buf[2]
    }
}
