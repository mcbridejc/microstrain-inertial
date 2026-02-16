use thiserror::Error;

use crate::{CRC_SIZE, HEADER_SIZE, SYNC1, SYNC2, checksum::Checksum, errors::ParseError};

pub mod base;

pub const BASE_DESCRIPTOR_SET: u8 = 1;

pub const ACKNACK_DESCRIPTOR: u8 = 0xF1;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum AckNack {
    Ack = 0,
    UnknownCommand = 1,
    BadChecksum = 2,
    InvalidParameter = 3,
    CommandFailed = 4,
    CommandTimeout = 5,
    UnknownDescriptorSet = 6,
    UnrecognizedReplyCode = 255,
}

impl From<u8> for AckNack {
    fn from(value: u8) -> Self {
        use AckNack::*;
        match value {
            0 => Ack,
            1 => BadChecksum,
            2 => InvalidParameter,
            3 => CommandFailed,
            4 => CommandTimeout,
            5 => UnknownDescriptorSet,
            _ => UnrecognizedReplyCode,
        }
    }
}

/// A trait for command resposne data structs to implement
pub trait CommandResponseData: Copy + core::fmt::Debug {
    fn from_data(data: &[u8]) -> Result<Self, ParseError>
    where
        Self: Sized;
}

// Types which do not expect a response, use `()` as their response type, so implement the trait for
// it here.
impl CommandResponseData for () {
    fn from_data(_data: &[u8]) -> Result<Self, crate::errors::ParseError> {
        Ok(())
    }
}

/// A trait for command fields to implement for serialization
pub trait CommandField {
    type Response: CommandResponseData;

    /// Returns the descriptor set this command belongs to'
    fn descriptor_set(&self) -> u8;

    /// Returns the descriptor for this command
    fn descriptor(&self) -> u8;

    /// Writes the payload for this command into a buffer
    ///
    /// This should be implemented by each command, and write the payload only, not including the
    /// length and descriptor bytes.
    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError>;

    fn serialize(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        if buf.len() < 2 {
            return Err(SerializeError::OutOfSpace);
        }
        let payload_size = self.serialize_payload(&mut buf[2..])?;
        buf[0] = payload_size + 2;
        buf[1] = self.descriptor();

        Ok(payload_size + 2)
    }
}

/// Every command gets a AckNack field in response; some commands get optional extra data in a
/// second field. This struct combines those two into a single object to return in response to a
/// command.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct CommandResponse<R: Clone + Copy + core::fmt::Debug> {
    /// The ack/nack response, which is always included
    pub acknack: AckNack,
    /// Additional data field which is provided in response to some commands
    pub data: Option<R>,
}

/// Untyped command response for iterating over a command response packet
///
/// We know the format of the acknack field, but the second (optional) data field cannot be
/// interpreted at this point because it varies by command.
pub struct GenericCommandResponse<'a> {
    pub acknack: AckNack,
    pub data: Option<&'a [u8]>,
}

/// Iterator for individual command responses in a response packet
///
/// From https://s3.amazonaws.com/files.microstrain.com/CV7%20Online/dcp_content/introduction/Command%20Overview.htm:
///
///    The reply contains at minimum a standard ACK/NACK field for every command in the originating
///    packet. This provides feedback as to whether the command was successfully executed, or why it
///    failed. The ack/nack reply uses descriptor 0xF1, which is reserved in all command descriptor
///    sets. Some commands can return additional data, for example to query the current setting. This
///    response data comes as an additional field immediately after the corresponding ack field and
///    will never be separated or split into a separate packet.
///
///
pub struct CommandResponseIter<'a> {
    payload: &'a [u8],
    pos: usize,
}

impl<'a> CommandResponseIter<'a> {
    pub fn new(payload: &'a [u8]) -> Self {
        Self { payload, pos: 0 }
    }
}

impl<'a> Iterator for CommandResponseIter<'a> {
    type Item = Result<GenericCommandResponse<'a>, ParseError>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.pos == self.payload.len() {
            return None;
        }
        let remaining_bytes = &self.payload[self.pos..];
        if remaining_bytes.len() < 2 {
            return None;
        }
        let len = remaining_bytes[0] as usize;
        let descriptor = remaining_bytes[1];
        if remaining_bytes.len() < len {
            return Some(Err(ParseError::LenTooShort {
                descriptor_set: 0,
                descriptor,
                need: len,
                got: remaining_bytes.len(),
            }));
        }
        if descriptor != ACKNACK_DESCRIPTOR {
            return Some(Err(ParseError::MissingAck { descriptor }));
        }

        let acknack_payload = &remaining_bytes[2..len];
        if acknack_payload.len() < 2 {
            return Some(Err(ParseError::LenTooShort {
                descriptor_set: 0,
                descriptor,
                need: 2,
                got: acknack_payload.len(),
            }));
        }

        // The command being ack'd
        let _command_descriptor = acknack_payload[0];
        // The ack reply code
        let acknack = acknack_payload[1].into();

        // The acknack may or may not be followed by a response field depending on the command. If
        // there is another field, and it is not an AckNack, it must be the response; consume and
        // return it.
        self.pos += len;
        let remaining_bytes = &self.payload[self.pos..];
        let mut response_data = None;
        if remaining_bytes.len() >= 2 {
            let len = remaining_bytes[0] as usize;
            let descriptor = remaining_bytes[1];
            if descriptor != ACKNACK_DESCRIPTOR {
                self.pos += len;
                response_data = Some(&remaining_bytes[2..len]);
            }
        }

        Some(Ok(GenericCommandResponse {
            acknack,
            data: response_data,
        }))
    }
}

#[derive(Clone, Copy, Debug, Error, PartialEq, Eq)]
pub enum SerializeError {
    #[error("The provided write buffer was too short")]
    OutOfSpace,
    #[error("Cannot combine fields from different descriptor sets into one packet")]
    MixedDescriptorSet,
    #[error("Cannot send a command with no fields")]
    ZeroFields,
}

/// Trait for an object which can be serialized to a full command packet
pub trait CommandSerialize {
    /// Serialize the command to the provided buffer. All transmitted bytes, included SYNC and CRC
    /// should be written to the buffer.
    fn serialize_command(&self, buf: &mut [u8]) -> Result<usize, SerializeError>;

    fn descriptor_set(&self) -> u8;
}

// A command packet is just a list of command fields, so we can implement CommandSerialize for any
// slice of fields
impl<T: CommandResponseData> CommandSerialize for &[&dyn CommandField<Response = T>] {
    fn serialize_command(&self, buf: &mut [u8]) -> Result<usize, SerializeError> {
        if buf.len() < HEADER_SIZE {
            return Err(SerializeError::OutOfSpace);
        }
        buf[0] = SYNC1;
        buf[1] = SYNC2;
        buf[2] = self.descriptor_set();
        // buf[3] will be payload len, but we don't know it yet

        let mut pos = 4;
        for field in *self {
            pos += (*field).serialize(&mut buf[pos..])? as usize;
        }

        // write payload len
        buf[3] = pos as u8 - HEADER_SIZE as u8;

        // Check that there are two bytes left for the CRC
        if buf.len() - pos < CRC_SIZE {
            return Err(SerializeError::OutOfSpace);
        }
        let mut chk = Checksum::new();
        chk.update(&buf[..pos]);
        let chk = chk.value();
        buf[pos] = (chk >> 8) as u8;
        buf[pos + 1] = chk as u8;
        pos += 2;

        Ok(pos)
    }

    fn descriptor_set(&self) -> u8 {
        if self.len() > 0 {
            self[0].descriptor_set()
        } else {
            // Unclear what should happen here? Creating messages with zero fields doesn't make
            // sense to do I think.
            0
        }
    }
}
