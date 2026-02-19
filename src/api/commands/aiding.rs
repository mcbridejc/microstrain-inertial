//! Aiding (0x13) commands.
#![allow(missing_docs)]
use super::base::FunctionSelector;
use super::{AIDING_DESCRIPTOR_SET, CommandField, CommandResponseData, SerializeError};
use crate::api::parse::ReadBuf as _;
use crate::api::types::{Quatf, Vector3f};
use crate::errors::ParseError;

/// Marker trait for Aiding (0x13) commands.
pub trait AidingCommandField: CommandField {}

fn len_too_short(descriptor: u8, need: usize, got: usize) -> ParseError {
    ParseError::LenTooShort {
        descriptor_set: AIDING_DESCRIPTOR_SET,
        descriptor,
        need,
        got,
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[repr(u8)]
pub enum AidingFrameFormat {
    Euler = 1,
    Quaternion = 2,
}

impl AidingFrameFormat {
    fn from_u8(v: u8) -> Option<Self> {
        match v {
            1 => Some(Self::Euler),
            2 => Some(Self::Quaternion),
            _ => None,
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum AidingFrameRotation {
    Euler(Vector3f),
    Quaternion(Quatf),
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[repr(u8)]
pub enum EchoMode {
    SuppressAck = 0,
    Standard = 1,
    Response = 2,
}

impl EchoMode {
    fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(Self::SuppressAck),
            1 => Some(Self::Standard),
            2 => Some(Self::Response),
            _ => None,
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[repr(u8)]
pub enum Timebase {
    InternalReference = 1,
    ExternalTime = 2,
    TimeOfArrival = 3,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct AidingTime {
    pub timebase: Timebase,
    pub reserved: u8,
    pub nanoseconds: u64,
}

impl AidingTime {
    pub const LEN: usize = 10;

    fn serialize(&self, out: &mut [u8]) {
        out[0] = self.timebase as u8;
        out[1] = self.reserved;
        out[2..10].copy_from_slice(&self.nanoseconds.to_be_bytes());
    }
}

macro_rules! impl_aiding_selector_frame_id_command {
    ($name:ident, $descriptor:expr, $selector:expr) => {
        #[derive(Debug, Copy, Clone, PartialEq, Eq)]
        pub struct $name {
            pub frame_id: u8,
        }

        impl CommandField for $name {
            type Response = ();

            fn descriptor_set(&self) -> u8 {
                AIDING_DESCRIPTOR_SET
            }

            fn descriptor(&self) -> u8 {
                $descriptor
            }

            fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
                if buf.len() < 2 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = $selector as u8;
                buf[1] = self.frame_id;
                Ok(2)
            }
        }

        impl AidingCommandField for $name {}
    };
}

macro_rules! impl_aiding_selector_no_param_command {
    ($name:ident, $descriptor:expr, $selector:expr) => {
        #[derive(Debug, Copy, Clone, PartialEq, Eq)]
        pub struct $name;

        impl CommandField for $name {
            type Response = ();

            fn descriptor_set(&self) -> u8 {
                AIDING_DESCRIPTOR_SET
            }

            fn descriptor(&self) -> u8 {
                $descriptor
            }

            fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = $selector as u8;
                Ok(1)
            }
        }

        impl AidingCommandField for $name {}
    };
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct FrameConfigurationResponse {
    pub frame_id: u8,
    pub format: AidingFrameFormat,
    pub tracking_enabled: bool,
    pub translation: Vector3f,
    pub rotation: AidingFrameRotation,
}

impl CommandResponseData for FrameConfigurationResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        let mut b = data;
        if b.len() < 31 {
            return Err(len_too_short(0x01, 31, b.len()));
        }

        let frame_id = b.read_u8();
        let format = AidingFrameFormat::from_u8(b.read_u8()).ok_or(ParseError::UnknownField {
            descriptor_set: AIDING_DESCRIPTOR_SET,
            descriptor: 0x01,
        })?;
        let tracking_enabled = b.read_u8() != 0;
        let translation = Vector3f::read_from(&mut b, (AIDING_DESCRIPTOR_SET, 0x01))?;

        let rotation = match format {
            AidingFrameFormat::Euler => {
                let e = Vector3f::read_from(&mut b, (AIDING_DESCRIPTOR_SET, 0x01))?;
                let _unused = b.read_f32();
                AidingFrameRotation::Euler(e)
            }
            AidingFrameFormat::Quaternion => AidingFrameRotation::Quaternion(Quatf::read_from(
                &mut b,
                (AIDING_DESCRIPTOR_SET, 0x01),
            )?),
        };

        Ok(Self {
            frame_id,
            format,
            tracking_enabled,
            translation,
            rotation,
        })
    }
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct FrameConfigurationWrite {
    pub frame_id: u8,
    pub tracking_enabled: bool,
    pub translation: Vector3f,
    pub rotation: AidingFrameRotation,
}

impl CommandField for FrameConfigurationWrite {
    type Response = FrameConfigurationResponse;

    fn descriptor_set(&self) -> u8 {
        AIDING_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x01
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        if buf.len() < 32 {
            return Err(SerializeError::OutOfSpace);
        }

        let (format, rot_vals) = match self.rotation {
            AidingFrameRotation::Euler(v) => (AidingFrameFormat::Euler, [v.x, v.y, v.z, 0.0]),
            AidingFrameRotation::Quaternion(q) => {
                (AidingFrameFormat::Quaternion, [q.w, q.x, q.y, q.z])
            }
        };

        buf[0] = FunctionSelector::Write as u8;
        buf[1] = self.frame_id;
        buf[2] = format as u8;
        buf[3] = self.tracking_enabled as u8;
        buf[4..8].copy_from_slice(&self.translation.x.to_be_bytes());
        buf[8..12].copy_from_slice(&self.translation.y.to_be_bytes());
        buf[12..16].copy_from_slice(&self.translation.z.to_be_bytes());
        buf[16..20].copy_from_slice(&rot_vals[0].to_be_bytes());
        buf[20..24].copy_from_slice(&rot_vals[1].to_be_bytes());
        buf[24..28].copy_from_slice(&rot_vals[2].to_be_bytes());
        buf[28..32].copy_from_slice(&rot_vals[3].to_be_bytes());
        Ok(32)
    }
}
impl AidingCommandField for FrameConfigurationWrite {}

impl_aiding_selector_frame_id_command!(FrameConfigurationRead, 0x01, FunctionSelector::Read);
impl_aiding_selector_frame_id_command!(FrameConfigurationSave, 0x01, FunctionSelector::Save);
impl_aiding_selector_frame_id_command!(FrameConfigurationLoad, 0x01, FunctionSelector::Load);
impl_aiding_selector_frame_id_command!(FrameConfigurationDefault, 0x01, FunctionSelector::Default);

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct EchoControlResponse {
    pub mode: EchoMode,
}

impl CommandResponseData for EchoControlResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        if data.is_empty() {
            return Err(len_too_short(0x1F, 1, data.len()));
        }
        let mode = EchoMode::from_u8(data[0]).ok_or(ParseError::UnknownField {
            descriptor_set: AIDING_DESCRIPTOR_SET,
            descriptor: 0x1F,
        })?;
        Ok(Self { mode })
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct EchoControlWrite {
    pub mode: EchoMode,
}

impl CommandField for EchoControlWrite {
    type Response = EchoControlResponse;

    fn descriptor_set(&self) -> u8 {
        AIDING_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x1F
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        if buf.len() < 2 {
            return Err(SerializeError::OutOfSpace);
        }
        buf[0] = FunctionSelector::Write as u8;
        buf[1] = self.mode as u8;
        Ok(2)
    }
}
impl AidingCommandField for EchoControlWrite {}

impl_aiding_selector_no_param_command!(EchoControlRead, 0x1F, FunctionSelector::Read);
impl_aiding_selector_no_param_command!(EchoControlSave, 0x1F, FunctionSelector::Save);
impl_aiding_selector_no_param_command!(EchoControlLoad, 0x1F, FunctionSelector::Load);
impl_aiding_selector_no_param_command!(EchoControlDefault, 0x1F, FunctionSelector::Default);

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct TrueHeading {
    pub time: AidingTime,
    pub frame_id: u8,
    pub heading_rad: f32,
    pub uncertainty_rad: f32,
    pub valid_flags: u16,
}

impl CommandField for TrueHeading {
    type Response = ();

    fn descriptor_set(&self) -> u8 {
        AIDING_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x31
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        if buf.len() < 21 {
            return Err(SerializeError::OutOfSpace);
        }
        self.time.serialize(&mut buf[0..10]);
        buf[10] = self.frame_id;
        buf[11..15].copy_from_slice(&self.heading_rad.to_be_bytes());
        buf[15..19].copy_from_slice(&self.uncertainty_rad.to_be_bytes());
        buf[19..21].copy_from_slice(&self.valid_flags.to_be_bytes());
        Ok(21)
    }
}
impl AidingCommandField for TrueHeading {}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct MagneticField {
    pub time: AidingTime,
    pub frame_id: u8,
    pub magnetic_field: Vector3f,
    pub uncertainty: Vector3f,
    pub valid_flags: u16,
}

impl CommandField for MagneticField {
    type Response = ();

    fn descriptor_set(&self) -> u8 {
        AIDING_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x32
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        if buf.len() < 37 {
            return Err(SerializeError::OutOfSpace);
        }
        self.time.serialize(&mut buf[0..10]);
        buf[10] = self.frame_id;
        buf[11..15].copy_from_slice(&self.magnetic_field.x.to_be_bytes());
        buf[15..19].copy_from_slice(&self.magnetic_field.y.to_be_bytes());
        buf[19..23].copy_from_slice(&self.magnetic_field.z.to_be_bytes());
        buf[23..27].copy_from_slice(&self.uncertainty.x.to_be_bytes());
        buf[27..31].copy_from_slice(&self.uncertainty.y.to_be_bytes());
        buf[31..35].copy_from_slice(&self.uncertainty.z.to_be_bytes());
        buf[35..37].copy_from_slice(&self.valid_flags.to_be_bytes());
        Ok(37)
    }
}
impl AidingCommandField for MagneticField {}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_response_type<C: CommandField<Response = R>, R>(_cmd: C) {}

    #[test]
    fn test_frame_configuration_write_serialize() {
        let cmd = FrameConfigurationWrite {
            frame_id: 1,
            tracking_enabled: true,
            translation: Vector3f::new(1.0, 2.0, 3.0),
            rotation: AidingFrameRotation::Euler(Vector3f::new(0.1, 0.2, 0.3)),
        };
        let mut buf = [0u8; 48];
        let cnt = cmd.serialize(&mut buf).unwrap();
        assert_eq!(34, cnt);
        assert_eq!(0x01, buf[1]);
        assert_eq!(FunctionSelector::Write as u8, buf[2]);
    }

    #[test]
    fn test_frame_configuration_read_serialize() {
        let cmd = FrameConfigurationRead { frame_id: 3 };
        let mut buf = [0u8; 8];
        let cnt = cmd.serialize(&mut buf).unwrap();
        assert_eq!(4, cnt);
        assert_eq!(FunctionSelector::Read as u8, buf[2]);
        assert_eq!(3, buf[3]);
    }

    #[test]
    fn test_echo_control_write_response_parse() {
        let parsed = EchoControlResponse::from_data(&[EchoMode::Response as u8]).unwrap();
        assert_eq!(EchoMode::Response, parsed.mode);
    }

    #[test]
    fn test_true_heading_serialize() {
        let cmd = TrueHeading {
            time: AidingTime {
                timebase: Timebase::ExternalTime,
                reserved: 1,
                nanoseconds: 123,
            },
            frame_id: 2,
            heading_rad: 0.4,
            uncertainty_rad: 0.1,
            valid_flags: 1,
        };
        let mut buf = [0u8; 32];
        let cnt = cmd.serialize(&mut buf).unwrap();
        assert_eq!(23, cnt);
        assert_eq!(0x31, buf[1]);
    }

    #[test]
    fn test_response_type_distinction() {
        assert_response_type::<FrameConfigurationWrite, FrameConfigurationResponse>(
            FrameConfigurationWrite {
                frame_id: 1,
                tracking_enabled: false,
                translation: Vector3f::new(0.0, 0.0, 0.0),
                rotation: AidingFrameRotation::Quaternion(Quatf::new(1.0, 0.0, 0.0, 0.0)),
            },
        );
        assert_response_type::<FrameConfigurationRead, ()>(FrameConfigurationRead { frame_id: 1 });
    }
}
