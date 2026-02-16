
use crate::{api::data::{filter::FilterPacket, sensor::SensorPacket}, errors::ParseError};

pub mod commands;
pub mod data;

/// Encapsulates all the possible packet types, both command and data
pub enum Packet<'a> {
    Command(CommandPacket),
    Data(DataPacket<'a>),
}


impl<'a> Packet<'a> {
    pub fn from_frame(descriptor_set: u8, payload: &'a [u8]) -> Result<Self, ParseError> {
        match descriptor_set {
            data::sensor::SENSOR_DESCRIPTOR_SET => Ok(Packet::Data(DataPacket::SensorPacket(
                SensorPacket::new(payload),
            ))),
            data::filter::FILTER_DESCRIPTOR_SET => Ok(Packet::Data(DataPacket::FilterPacket(
                FilterPacket::new(payload),
            ))),
            _ => Err(ParseError::UnknownDescriptorSet { descriptor_set }),
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
