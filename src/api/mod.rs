//! Command and data message definitions
//! 
//! MIP packets are broken up into sets, called "descriptor sets". There are two types of descriptor
//! sets, "Command" and "Data". Command descriptor sets include packets sent to the IMU for
//! configuraiton and control, as well as packets sent in response to those commands. Data
//! descriptor sets are only sent by the IMU, and contain the streamed data like sensor readings, or
//! state estimator outputs, etc. 
//! 
//! Each packet, regardless of type, can contain many different fields, each with a different
//! descriptor, so that multiple commands or data fields can be packed into a single packet.
//! Only fields which belong to the same descriptor set can be combined into a packet. 
use crate::{
    api::data::{filter::FilterPacket, sensor::SensorPacket},
    errors::ParseError,
};

pub mod commands;
pub mod data;
pub mod parse;
pub mod types;

/// Can represent all possible DATA packets
pub enum DataPacket<'a> {
    SensorPacket(SensorPacket<'a>),
    FilterPacket(FilterPacket<'a>),
}

impl<'a> DataPacket<'a> {
    /// Create a [`DataPacket`] from a received descriptor set payload
    pub fn from_frame(descriptor_set: u8, payload: &'a [u8]) -> Result<Self, ParseError> {
        match descriptor_set {
            data::sensor::SENSOR_DESCRIPTOR_SET => {
                Ok(DataPacket::SensorPacket(SensorPacket::new(payload)))
            }
            data::filter::FILTER_DESCRIPTOR_SET => {
                Ok(DataPacket::FilterPacket(FilterPacket::new(payload)))
            }
            _ => Err(ParseError::UnknownDescriptorSet { descriptor_set }),
        }
    }
}
