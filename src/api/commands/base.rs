//! Base (0x01) commands
//!
#![allow(missing_docs)]
use super::{BASE_DESCRIPTOR_SET, CommandField};
use crate::{
    api::commands::{CommandResponseData, SerializeError},
    errors::ParseError,
};

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

macro_rules! impl_base_unit_command {
    ($(#[$meta:meta])* $name:ident, $descriptor:expr, $response:ty) => {
        $(#[$meta])*
        pub struct $name;

        impl CommandField for $name {
            type Response = $response;

            fn descriptor_set(&self) -> u8 {
                BASE_DESCRIPTOR_SET
            }

            fn descriptor(&self) -> u8 {
                $descriptor
            }

            fn serialize_payload(&self, _buf: &mut [u8]) -> Result<u8, SerializeError> {
                Ok(0)
            }
        }

        impl BaseCommandField for $name {}
    };
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[repr(u8)]
pub enum FunctionSelector {
    Write = 0x01,
    Read = 0x02,
    Save = 0x03,
    Load = 0x04,
    Default = 0x05,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct DeviceInfoResponse {
    pub firmware_version: u16,
    pub model_name: [u8; 16],
    pub model_number: [u8; 16],
    pub serial_number: [u8; 16],
    pub lot_number: [u8; 16],
    pub device_options: [u8; 16],
}

impl CommandResponseData for DeviceInfoResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        const NEED: usize = 2 + 16 * 5;
        if data.len() < NEED {
            return Err(ParseError::LenTooShort {
                descriptor_set: BASE_DESCRIPTOR_SET,
                descriptor: 0x03,
                need: NEED,
                got: data.len(),
            });
        }

        let mut idx = 0usize;
        let firmware_version = u16::from_be_bytes([data[idx], data[idx + 1]]);
        idx += 2;

        let mut model_name = [0u8; 16];
        model_name.copy_from_slice(&data[idx..idx + 16]);
        idx += 16;

        let mut model_number = [0u8; 16];
        model_number.copy_from_slice(&data[idx..idx + 16]);
        idx += 16;

        let mut serial_number = [0u8; 16];
        serial_number.copy_from_slice(&data[idx..idx + 16]);
        idx += 16;

        let mut lot_number = [0u8; 16];
        lot_number.copy_from_slice(&data[idx..idx + 16]);
        idx += 16;

        let mut device_options = [0u8; 16];
        device_options.copy_from_slice(&data[idx..idx + 16]);

        Ok(Self {
            firmware_version,
            model_name,
            model_number,
            serial_number,
            lot_number,
            device_options,
        })
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct DeviceDescriptorsResponse {
    pub descriptors: [u16; 127],
    pub len: usize,
}

impl CommandResponseData for DeviceDescriptorsResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() % 2 != 0 {
            return Err(ParseError::LenTooShort {
                descriptor_set: BASE_DESCRIPTOR_SET,
                descriptor: 0x04,
                need: data.len() + 1,
                got: data.len(),
            });
        }
        let count = data.len() / 2;
        if count > 127 {
            return Err(ParseError::LenTooShort {
                descriptor_set: BASE_DESCRIPTOR_SET,
                descriptor: 0x04,
                need: count,
                got: 127,
            });
        }

        let mut descriptors = [0u16; 127];
        for (i, d) in descriptors[..count].iter_mut().enumerate() {
            let offset = 2 * i;
            *d = u16::from_be_bytes([data[offset], data[offset + 1]]);
        }

        Ok(Self {
            descriptors,
            len: count,
        })
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct BuiltInTestResponse {
    pub result: u32,
}

impl CommandResponseData for BuiltInTestResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() < 4 {
            return Err(ParseError::LenTooShort {
                descriptor_set: BASE_DESCRIPTOR_SET,
                descriptor: 0x05,
                need: 4,
                got: data.len(),
            });
        }
        Ok(Self {
            result: u32::from_be_bytes([data[0], data[1], data[2], data[3]]),
        })
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct ContinuousBuiltInTestResponse {
    pub result: [u8; 16],
}

impl CommandResponseData for ContinuousBuiltInTestResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() < 16 {
            return Err(ParseError::LenTooShort {
                descriptor_set: BASE_DESCRIPTOR_SET,
                descriptor: 0x08,
                need: 16,
                got: data.len(),
            });
        }
        let mut result = [0u8; 16];
        result.copy_from_slice(&data[..16]);
        Ok(Self { result })
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CommPortSpeedResponse {
    pub port: u8,
    pub baud: u32,
}

impl CommandResponseData for CommPortSpeedResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() < 5 {
            return Err(ParseError::LenTooShort {
                descriptor_set: BASE_DESCRIPTOR_SET,
                descriptor: 0x09,
                need: 5,
                got: data.len(),
            });
        }
        Ok(Self {
            port: data[0],
            baud: u32::from_be_bytes([data[1], data[2], data[3], data[4]]),
        })
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CommPortSpeed {
    Write { port: u8, baud: u32 },
    Read { port: u8 },
    Save { port: u8 },
    Load { port: u8 },
    Default { port: u8 },
}

impl CommandField for CommPortSpeed {
    type Response = CommPortSpeedResponse;

    fn descriptor_set(&self) -> u8 {
        BASE_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x09
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        let (function_selector, port, baud) = match self {
            Self::Write { port, baud } => (FunctionSelector::Write as u8, *port, *baud),
            Self::Read { port } => (FunctionSelector::Read as u8, *port, 0),
            Self::Save { port } => (FunctionSelector::Save as u8, *port, 0),
            Self::Load { port } => (FunctionSelector::Load as u8, *port, 0),
            Self::Default { port } => (FunctionSelector::Default as u8, *port, 0),
        };
        if buf.len() < 6 {
            return Err(SerializeError::OutOfSpace);
        }
        buf[0] = function_selector;
        buf[1] = port;
        buf[2..6].copy_from_slice(&baud.to_be_bytes());
        Ok(6)
    }
}

impl BaseCommandField for CommPortSpeed {}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[repr(u8)]
pub enum GpsTimeFieldId {
    WeekNumber = 1,
    TimeOfWeek = 2,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct GpsTimeUpdateCommand {
    pub field_id: GpsTimeFieldId,
    pub value: u32,
}

impl CommandField for GpsTimeUpdateCommand {
    type Response = ();

    fn descriptor_set(&self) -> u8 {
        BASE_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x72
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        if buf.len() < 6 {
            return Err(SerializeError::OutOfSpace);
        }
        buf[0] = FunctionSelector::Write as u8;
        buf[1] = self.field_id as u8;
        buf[2..6].copy_from_slice(&self.value.to_be_bytes());
        Ok(6)
    }
}

impl BaseCommandField for GpsTimeUpdateCommand {}

impl_base_unit_command!(SetToIdle, 0x02, ());
impl_base_unit_command!(GetDeviceInformation, 0x03, DeviceInfoResponse);
impl_base_unit_command!(GetDeviceDescriptors, 0x04, DeviceDescriptorsResponse);
impl_base_unit_command!(BuiltInTest, 0x05, BuiltInTestResponse);
impl_base_unit_command!(Resume, 0x06, ());
impl_base_unit_command!(
    GetDeviceDescriptorsExtended,
    0x07,
    DeviceDescriptorsResponse
);
impl_base_unit_command!(ContinuousBuiltInTest, 0x08, ContinuousBuiltInTestResponse);
impl_base_unit_command!(ResetDevice, 0x7E, ());

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gps_time_update_serialization() {
        let cmd = GpsTimeUpdateCommand {
            field_id: GpsTimeFieldId::WeekNumber,
            value: 2300,
        };

        let mut buf = [0u8; 16];
        let count = cmd.serialize(&mut buf).unwrap();
        assert_eq!(8, count);
        assert_eq!(8, buf[0]);
        assert_eq!(0x72, buf[1]);
        assert_eq!(FunctionSelector::Write as u8, buf[2]);
        assert_eq!(GpsTimeFieldId::WeekNumber as u8, buf[3]);
        assert_eq!(2300u32.to_be_bytes(), [buf[4], buf[5], buf[6], buf[7]]);
    }

    #[test]
    fn test_comm_port_speed_write_serialization() {
        let cmd = CommPortSpeed::Write {
            port: 1,
            baud: 115200,
        };

        let mut buf = [0u8; 16];
        let count = cmd.serialize(&mut buf).unwrap();
        assert_eq!(8, count);
        assert_eq!(8, buf[0]);
        assert_eq!(0x09, buf[1]);
        assert_eq!(FunctionSelector::Write as u8, buf[2]);
        assert_eq!(1, buf[3]);
        assert_eq!(115200u32.to_be_bytes(), [buf[4], buf[5], buf[6], buf[7]]);
    }

    #[test]
    fn test_device_info_response_parse() {
        let mut data = [0u8; 82];
        data[0..2].copy_from_slice(&0x1234u16.to_be_bytes());
        data[2..18].copy_from_slice(b"MODEL_NAME_12345");
        data[18..34].copy_from_slice(b"MODEL_NUM_123456");
        data[34..50].copy_from_slice(b"SERIAL_NUM_12345");
        data[50..66].copy_from_slice(b"LOT_NUM__1234567");
        data[66..82].copy_from_slice(b"DEVICE_OPT_12345");

        let parsed = DeviceInfoResponse::from_data(&data).unwrap();
        assert_eq!(0x1234, parsed.firmware_version);
        assert_eq!(b"MODEL_NAME_12345", &parsed.model_name);
    }
}
