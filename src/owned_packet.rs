//! Utility for creating, storing and serializing MIP packets
use crate::{
    CRC_SIZE, HEADER_SIZE, MAX_PACKET_SIZE, MAX_PAYLOAD_SIZE, SYNC1, SYNC2, checksum::calc_checksum,
};

#[derive(Clone, Copy, Debug)]
/// A utility struct which both allocates storage for and provides interpretation of a MIP packet
/// buffer
pub struct OwnedPacket {
    buf: [u8; MAX_PACKET_SIZE],
}

impl OwnedPacket {
    /// Create a new message with empty payload
    pub fn new(descriptor_set: u8) -> Self {
        let mut buf = [0; MAX_PACKET_SIZE];
        buf[0] = SYNC1;
        buf[1] = SYNC2;
        buf[2] = descriptor_set;
        buf[3] = 0;

        Self { buf }
    }

    /// Create a new message with a payload
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
        let chk = calc_checksum(&self.buf[0..payload_len + HEADER_SIZE]);
        self.buf[payload_len + HEADER_SIZE..payload_len + HEADER_SIZE + CRC_SIZE]
            .copy_from_slice(&chk.to_be_bytes());
    }

    /// Get the packet as a slice of bytes
    ///
    /// This is the full transmittable packet, including sync bytes and CRC
    pub fn as_slice(&mut self) -> &[u8] {
        self.checksum();
        &self.buf[..self.payload_len() as usize + HEADER_SIZE + CRC_SIZE]
    }

    /// Get the payload of the message
    pub fn payload(&self) -> &[u8] {
        &self.buf[HEADER_SIZE..self.payload_len() as usize + HEADER_SIZE]
    }

    /// Get a mutable buffer to the payload
    ///
    /// WARNING: This will return the entire available buffer space for the payload, ignoring its
    /// current length.
    ///
    /// If you use this to modify the payload, you *must* also call `set_payload_len` to set the
    /// length of the payload you have written.
    pub fn payload_mut(&mut self) -> &mut [u8] {
        let payload_len = self.buf.len() - CRC_SIZE;
        &mut self.buf[HEADER_SIZE..payload_len]
    }

    /// Get the payload length
    pub fn payload_len(&self) -> u8 {
        self.buf[3]
    }

    /// Set the payload length
    pub fn set_payload_len(&mut self, len: u8) {
        self.buf[3] = len;
    }

    /// Get the descriptor set of the packet
    pub fn descriptor_set(&self) -> u8 {
        self.buf[2]
    }

    /// Set the descriptor set of the packet
    pub fn set_descriptor_set(&mut self, descriptor_set: u8) {
        self.buf[2] = descriptor_set;
    }
}
