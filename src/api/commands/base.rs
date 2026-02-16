//! Base commands
//!

use super::{BASE_DESCRIPTOR_SET, CommandField};
use crate::{
    SYNC1, SYNC2,
    api::commands::{CommandResponseData, SerializeError},
    serialize::OwnedMessage,
};

pub struct BasePacket<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> BasePacket<'a> {
    pub fn new(buf: &'a mut [u8]) -> Self {
        if buf.len() < 6 {
            panic!("Minimum BasePacket buffer is 6 bytes");
        }
        buf[0] = SYNC1;
        buf[1] = SYNC2;
        buf[2] = BASE_DESCRIPTOR_SET;
        // buf[3] is reserverd for payload length
        Self { buf, pos: 4 }
    }

    pub fn add_field(&mut self, d: &impl BaseCommandField) -> Result<(), SerializeError> {
        let buf_len = self.buf.len();
        // Reserve last two bytes for CRC
        let cnt = d.serialize(&mut self.buf[self.pos..buf_len - 2])?;
        self.pos += cnt as usize;
        Ok(())
    }

    pub fn as_bytes(&mut self) -> &[u8] {
        self.buf[3] = (self.pos - 4) as u8;
        let mut chk16 = crate::checksum::Checksum::new();
        chk16.update(&self.buf[..self.pos]);
        let chk = chk16.value();
        self.buf[self.pos] = (chk >> 8) as u8;
        self.buf[self.pos + 1] = chk as u8;
        &self.buf[..self.pos + 2]
    }

    pub fn from_fields<T: CommandResponseData>(
        fields: &[&dyn BaseCommandField<Response = T>],
    ) -> Result<OwnedMessage, SerializeError> {
        let mut pkt = OwnedMessage::new(BASE_DESCRIPTOR_SET);
        let payload = pkt.payload_mut();
        let mut payload_size = 0;
        for f in fields {
            payload_size += f.serialize(&mut payload[payload_size as usize..])?;
        }
        pkt.set_payload_len(payload_size as u8);
        Ok(pkt)
    }
}

impl From<BasePacket<'_>> for OwnedMessage {
    fn from(mut value: BasePacket) -> Self {
        OwnedMessage::new_with_payload(BASE_DESCRIPTOR_SET, value.as_bytes())
    }
}

/// Marker trait for base commands
///
/// Grouping commands by their descriptor set with a marker trait allows for APIs which restrict
/// slices of commands to all belong to the same descriptor set, making it a compile time error.
pub trait BaseCommandField: CommandField {}

pub struct Ping {}

impl CommandField for Ping {
    type Response = ();

    fn descriptor_set(&self) -> u8 {
        BASE_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        1
    }

    fn serialize_payload(&self, _buf: &mut [u8]) -> Result<u8, SerializeError> {
        // Payload is empty
        Ok(0)
    }
}

impl BaseCommandField for Ping {}
