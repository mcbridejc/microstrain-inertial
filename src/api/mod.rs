use thiserror::Error;

use crate::api::data::{filter::FilterPacket, sensor::SensorPacket};

#[derive(Clone, Copy, Debug, Error, PartialEq, Eq)]
pub enum Error {
    #[error(
        "Length of data didn't match expected for 0x{descriptor_set:x}:0x{descriptor:x}. Need: {need}. Got: {got}"
    )]
    LenTooShort {
        descriptor_set: u8,
        descriptor: u8,
        need: usize,
        got: usize,
    },
    #[error("Unexpected descriptor 0x{descriptor:x} in set 0x{descriptor_set:x}")]
    UnknownField { descriptor_set: u8, descriptor: u8 },
    #[error("InvalidEnum")]
    InvalidEnum {
        descriptor_set: u8,
        descriptor: u8,
        raw: u64,
    },
    #[error("Unrecognized descriptor set 0x{descriptor_set:x}")]
    UnknownDescriptorSet { descriptor_set: u8 },
}

pub mod data;

/// Encapsulates all the possible packet types, both command and data
pub enum Packet<'a> {
    Command(CommandPacket),
    Data(DataPacket<'a>),
}

impl<'a> Packet<'a> {
    pub fn from_frame(descriptor_set: u8, payload: &'a [u8]) -> Result<Self, Error> {
        match descriptor_set {
            data::sensor::SENSOR_DESCRIPTOR_SET => Ok(Packet::Data(DataPacket::SensorPacket(
                SensorPacket::new(payload),
            ))),
            data::filter::FILTER_DESCRIPTOR_SET => Ok(Packet::Data(DataPacket::FilterPacket(
                FilterPacket::new(payload),
            ))),
            _ => Err(Error::UnknownDescriptorSet { descriptor_set }),
        }
    }
}

pub enum CommandPacket {
    Todo,
}

pub enum DataPacket<'a> {
    SensorPacket(SensorPacket<'a>),
    FilterPacket(FilterPacket<'a>),
}
