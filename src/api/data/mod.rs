//! Type definitions for interpreting data packets
//!
use {filter::FilterPacket, sensor::SensorPacket, shared::SharedPacket, system::SystemPacket};

pub mod filter;
pub mod sensor;
pub mod shared;
pub mod system;

pub use crate::api::{parse::ReadBuf, types::*};
pub use crate::errors::ParseError;

/// helper function for validating length
#[inline]
fn ensure_len(buf: &&[u8], need: usize, descriptor: (u8, u8)) -> Result<(), ParseError> {
    if buf.len() < need {
        return Err(ParseError::LenTooShort {
            descriptor_set: descriptor.0,
            descriptor: descriptor.1,
            need,
            got: buf.len(),
        });
    }
    Ok(())
}

/// Can represent all possible DATA packets
pub enum DataPacket<'a> {
    SensorPacket(SensorPacket<'a>),
    FilterPacket(FilterPacket<'a>),
    SharedPacket(SharedPacket<'a>),
    SystemPacket(SystemPacket<'a>),
}

impl<'a> DataPacket<'a> {
    /// Create a [`DataPacket`] from a received descriptor set payload
    pub fn from_frame(descriptor_set: u8, payload: &'a [u8]) -> Result<Self, ParseError> {
        match descriptor_set {
            sensor::SENSOR_DESCRIPTOR_SET => {
                Ok(DataPacket::SensorPacket(SensorPacket::new(payload)))
            }
            filter::FILTER_DESCRIPTOR_SET => {
                Ok(DataPacket::FilterPacket(FilterPacket::new(payload)))
            }
            shared::SHARED_DESCRIPTOR_SET => {
                Ok(DataPacket::SharedPacket(SharedPacket::new(payload)))
            }
            system::SYSTEM_DESCRIPTOR_SET => {
                Ok(DataPacket::SystemPacket(SystemPacket::new(payload)))
            }
            _ => Err(ParseError::UnknownDescriptorSet { descriptor_set }),
        }
    }
}
