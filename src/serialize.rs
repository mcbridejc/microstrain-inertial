
use crate::{CRC_SIZE, HEADER_SIZE, MAX_PACKET_SIZE, MAX_PAYLOAD_SIZE, SYNC1, SYNC2, checksum::{self, Checksum}};


#[derive(Clone, Copy, Debug)]
pub struct OwnedMessage {
    buf: [u8; MAX_PACKET_SIZE],
}

impl OwnedMessage {
    pub fn new(descriptor_set: u8) -> Self {
        let mut buf = [0; MAX_PACKET_SIZE];
        buf[0] = SYNC1;
        buf[1] = SYNC2;
        buf[2] = descriptor_set;
        buf[3] = 0;

        Self { buf }
    }
    pub fn new_with_payload(descriptor_set: u8, payload: &[u8]) -> Self {
        assert!(payload.len() <= MAX_PAYLOAD_SIZE);
        let mut buf = [0; MAX_PACKET_SIZE];
        buf[0] = SYNC1;
        buf[1] = SYNC2;
        buf[2] = descriptor_set;
        buf[3] = payload.len() as u8;
        buf[HEADER_SIZE..HEADER_SIZE + payload.len()].copy_from_slice(payload);
        Self { buf }
    }

    fn checksum(&mut self) {
        let payload_len = self.payload_len() as usize;
        let chk = Checksum::calc_checksum(&self.buf[0..payload_len + HEADER_SIZE]);
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

    pub fn payload_mut(&mut self) -> &mut [u8] {
        let payload_len = self.buf.len() as usize - CRC_SIZE;
        &mut self.buf[HEADER_SIZE..payload_len]
    }

    pub fn payload_len(&self) -> u8 {
        self.buf[3]
    }

    pub fn set_payload_len(&mut self, len: u8) {
        self.buf[3] = len;
    }

    pub fn descriptor_set(&self) -> u8 {
        self.buf[2]
    }

    pub fn set_descriptor_set(&mut self, descriptor_set: u8) {
        self.buf[2] = descriptor_set;
    }
}
