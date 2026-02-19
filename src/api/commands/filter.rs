//! Filter (0x0D) commands.
#![allow(missing_docs)]
use crate::api::{parse::ReadBuf as _, types::Vector3f};
use crate::errors::ParseError;

use super::base::FunctionSelector;
use super::{CommandField, CommandResponseData, FILTER_DESCRIPTOR_SET, SerializeError};

/// Marker trait for Filter (0x0D) commands.
pub trait FilterCommandField: CommandField {}

fn len_too_short(descriptor: u8, need: usize, got: usize) -> ParseError {
    ParseError::LenTooShort {
        descriptor_set: FILTER_DESCRIPTOR_SET,
        descriptor,
        need,
        got,
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[repr(u8)]
pub enum HeadingType {
    True = 1,
    Magnetic = 2,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[repr(u8)]
pub enum DeclinationSource {
    None = 1,
    Wmm = 2,
    Manual = 3,
}

impl DeclinationSource {
    fn from_u8(v: u8) -> Option<Self> {
        match v {
            1 => Some(Self::None),
            2 => Some(Self::Wmm),
            3 => Some(Self::Manual),
            _ => None,
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[repr(u16)]
pub enum AidingSource {
    GnssPosVel = 0,
    GnssHeading = 1,
    Altimeter = 2,
    Speed = 3,
    Mag = 4,
    ExternalHeading = 5,
    ExternalAltimeter = 6,
    ExternalMag = 7,
    BodyFrameVel = 8,
    All = 0xFFFF,
}

impl AidingSource {
    fn from_u16(v: u16) -> Option<Self> {
        match v {
            0 => Some(Self::GnssPosVel),
            1 => Some(Self::GnssHeading),
            2 => Some(Self::Altimeter),
            3 => Some(Self::Speed),
            4 => Some(Self::Mag),
            5 => Some(Self::ExternalHeading),
            6 => Some(Self::ExternalAltimeter),
            7 => Some(Self::ExternalMag),
            8 => Some(Self::BodyFrameVel),
            0xFFFF => Some(Self::All),
            _ => None,
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[repr(u8)]
pub enum AdaptiveFilterLevel {
    Off = 0,
    Conservative = 1,
    Moderate = 2,
    Aggressive = 3,
}

impl AdaptiveFilterLevel {
    fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(Self::Off),
            1 => Some(Self::Conservative),
            2 => Some(Self::Moderate),
            3 => Some(Self::Aggressive),
            _ => None,
        }
    }
}

macro_rules! impl_filter_unit_command {
    ($(#[$meta:meta])* $name:ident, $descriptor:expr) => {
        $(#[$meta])*
        #[derive(Debug, Copy, Clone, PartialEq, Eq)]
        pub struct $name;

        impl CommandField for $name {
            type Response = ();

            fn descriptor_set(&self) -> u8 {
                FILTER_DESCRIPTOR_SET
            }

            fn descriptor(&self) -> u8 {
                $descriptor
            }

            fn serialize_payload(&self, _buf: &mut [u8]) -> Result<u8, SerializeError> {
                Ok(0)
            }
        }

        impl FilterCommandField for $name {}
    };
}

macro_rules! impl_filter_selector_no_param_command {
    ($name:ident, $descriptor:expr, $selector:expr) => {
        #[derive(Debug, Copy, Clone, PartialEq, Eq)]
        pub struct $name;

        impl CommandField for $name {
            type Response = ();

            fn descriptor_set(&self) -> u8 {
                FILTER_DESCRIPTOR_SET
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

        impl FilterCommandField for $name {}
    };
}

macro_rules! impl_filter_selector_source_command {
    ($name:ident, $descriptor:expr, $selector:expr) => {
        #[derive(Debug, Copy, Clone, PartialEq, Eq)]
        pub struct $name {
            pub source: AidingSource,
        }

        impl CommandField for $name {
            type Response = ();

            fn descriptor_set(&self) -> u8 {
                FILTER_DESCRIPTOR_SET
            }

            fn descriptor(&self) -> u8 {
                $descriptor
            }

            fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
                if buf.len() < 3 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = $selector as u8;
                buf[1..3].copy_from_slice(&(self.source as u16).to_be_bytes());
                Ok(3)
            }
        }

        impl FilterCommandField for $name {}
    };
}

impl_filter_unit_command!(ResetNavigationFilter, 0x01);

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct SetInitialHeadingControl {
    pub heading_rad: f32,
}

impl CommandField for SetInitialHeadingControl {
    type Response = ();

    fn descriptor_set(&self) -> u8 {
        FILTER_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x03
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        if buf.len() < 4 {
            return Err(SerializeError::OutOfSpace);
        }
        buf[0..4].copy_from_slice(&self.heading_rad.to_be_bytes());
        Ok(4)
    }
}
impl FilterCommandField for SetInitialHeadingControl {}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct ExternalHeadingUpdate {
    pub heading_rad: f32,
    pub heading_uncertainty_rad: f32,
    pub heading_type: HeadingType,
}

impl CommandField for ExternalHeadingUpdate {
    type Response = ();

    fn descriptor_set(&self) -> u8 {
        FILTER_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x17
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        if buf.len() < 9 {
            return Err(SerializeError::OutOfSpace);
        }
        buf[0..4].copy_from_slice(&self.heading_rad.to_be_bytes());
        buf[4..8].copy_from_slice(&self.heading_uncertainty_rad.to_be_bytes());
        buf[8] = self.heading_type as u8;
        Ok(9)
    }
}
impl FilterCommandField for ExternalHeadingUpdate {}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct ExternalHeadingUpdateWithTime {
    pub gps_time_s: f64,
    pub gps_week: u16,
    pub heading_rad: f32,
    pub heading_uncertainty_rad: f32,
    pub heading_type: HeadingType,
}

impl CommandField for ExternalHeadingUpdateWithTime {
    type Response = ();

    fn descriptor_set(&self) -> u8 {
        FILTER_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x1F
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        if buf.len() < 19 {
            return Err(SerializeError::OutOfSpace);
        }
        buf[0..8].copy_from_slice(&self.gps_time_s.to_be_bytes());
        buf[8..10].copy_from_slice(&self.gps_week.to_be_bytes());
        buf[10..14].copy_from_slice(&self.heading_rad.to_be_bytes());
        buf[14..18].copy_from_slice(&self.heading_uncertainty_rad.to_be_bytes());
        buf[18] = self.heading_type as u8;
        Ok(19)
    }
}
impl FilterCommandField for ExternalHeadingUpdateWithTime {}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct AccelNoiseStandardDeviationResponse {
    pub noise: Vector3f,
}

impl CommandResponseData for AccelNoiseStandardDeviationResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        let mut b = data;
        Ok(Self {
            noise: Vector3f::read_from(&mut b, (FILTER_DESCRIPTOR_SET, 0x1A))?,
        })
    }
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct AccelNoiseStandardDeviationWrite {
    pub noise: Vector3f,
}

impl CommandField for AccelNoiseStandardDeviationWrite {
    type Response = AccelNoiseStandardDeviationResponse;

    fn descriptor_set(&self) -> u8 {
        FILTER_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x1A
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        if buf.len() < 13 {
            return Err(SerializeError::OutOfSpace);
        }
        buf[0] = FunctionSelector::Write as u8;
        buf[1..5].copy_from_slice(&self.noise.x.to_be_bytes());
        buf[5..9].copy_from_slice(&self.noise.y.to_be_bytes());
        buf[9..13].copy_from_slice(&self.noise.z.to_be_bytes());
        Ok(13)
    }
}
impl FilterCommandField for AccelNoiseStandardDeviationWrite {}

impl_filter_selector_no_param_command!(
    AccelNoiseStandardDeviationRead,
    0x1A,
    FunctionSelector::Read
);
impl_filter_selector_no_param_command!(
    AccelNoiseStandardDeviationSave,
    0x1A,
    FunctionSelector::Save
);
impl_filter_selector_no_param_command!(
    AccelNoiseStandardDeviationLoad,
    0x1A,
    FunctionSelector::Load
);
impl_filter_selector_no_param_command!(
    AccelNoiseStandardDeviationDefault,
    0x1A,
    FunctionSelector::Default
);

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct GyroNoiseStandardDeviationResponse {
    pub noise: Vector3f,
}

impl CommandResponseData for GyroNoiseStandardDeviationResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        let mut b = data;
        Ok(Self {
            noise: Vector3f::read_from(&mut b, (FILTER_DESCRIPTOR_SET, 0x1B))?,
        })
    }
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct GyroNoiseStandardDeviationWrite {
    pub noise: Vector3f,
}

impl CommandField for GyroNoiseStandardDeviationWrite {
    type Response = GyroNoiseStandardDeviationResponse;

    fn descriptor_set(&self) -> u8 {
        FILTER_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x1B
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        if buf.len() < 13 {
            return Err(SerializeError::OutOfSpace);
        }
        buf[0] = FunctionSelector::Write as u8;
        buf[1..5].copy_from_slice(&self.noise.x.to_be_bytes());
        buf[5..9].copy_from_slice(&self.noise.y.to_be_bytes());
        buf[9..13].copy_from_slice(&self.noise.z.to_be_bytes());
        Ok(13)
    }
}
impl FilterCommandField for GyroNoiseStandardDeviationWrite {}

impl_filter_selector_no_param_command!(
    GyroNoiseStandardDeviationRead,
    0x1B,
    FunctionSelector::Read
);
impl_filter_selector_no_param_command!(
    GyroNoiseStandardDeviationSave,
    0x1B,
    FunctionSelector::Save
);
impl_filter_selector_no_param_command!(
    GyroNoiseStandardDeviationLoad,
    0x1B,
    FunctionSelector::Load
);
impl_filter_selector_no_param_command!(
    GyroNoiseStandardDeviationDefault,
    0x1B,
    FunctionSelector::Default
);

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct AccelBiasModelParametersResponse {
    pub beta: Vector3f,
    pub noise: Vector3f,
}

impl CommandResponseData for AccelBiasModelParametersResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        let mut b = data;
        Ok(Self {
            beta: Vector3f::read_from(&mut b, (FILTER_DESCRIPTOR_SET, 0x1C))?,
            noise: Vector3f::read_from(&mut b, (FILTER_DESCRIPTOR_SET, 0x1C))?,
        })
    }
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct AccelBiasModelParametersWrite {
    pub beta: Vector3f,
    pub noise: Vector3f,
}

impl CommandField for AccelBiasModelParametersWrite {
    type Response = AccelBiasModelParametersResponse;

    fn descriptor_set(&self) -> u8 {
        FILTER_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x1C
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        if buf.len() < 25 {
            return Err(SerializeError::OutOfSpace);
        }
        buf[0] = FunctionSelector::Write as u8;
        buf[1..5].copy_from_slice(&self.beta.x.to_be_bytes());
        buf[5..9].copy_from_slice(&self.beta.y.to_be_bytes());
        buf[9..13].copy_from_slice(&self.beta.z.to_be_bytes());
        buf[13..17].copy_from_slice(&self.noise.x.to_be_bytes());
        buf[17..21].copy_from_slice(&self.noise.y.to_be_bytes());
        buf[21..25].copy_from_slice(&self.noise.z.to_be_bytes());
        Ok(25)
    }
}
impl FilterCommandField for AccelBiasModelParametersWrite {}

impl_filter_selector_no_param_command!(AccelBiasModelParametersRead, 0x1C, FunctionSelector::Read);
impl_filter_selector_no_param_command!(AccelBiasModelParametersSave, 0x1C, FunctionSelector::Save);
impl_filter_selector_no_param_command!(AccelBiasModelParametersLoad, 0x1C, FunctionSelector::Load);
impl_filter_selector_no_param_command!(
    AccelBiasModelParametersDefault,
    0x1C,
    FunctionSelector::Default
);

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct GyroBiasModelParametersResponse {
    pub beta: Vector3f,
    pub noise: Vector3f,
}

impl CommandResponseData for GyroBiasModelParametersResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        let mut b = data;
        Ok(Self {
            beta: Vector3f::read_from(&mut b, (FILTER_DESCRIPTOR_SET, 0x1D))?,
            noise: Vector3f::read_from(&mut b, (FILTER_DESCRIPTOR_SET, 0x1D))?,
        })
    }
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct GyroBiasModelParametersWrite {
    pub beta: Vector3f,
    pub noise: Vector3f,
}

impl CommandField for GyroBiasModelParametersWrite {
    type Response = GyroBiasModelParametersResponse;

    fn descriptor_set(&self) -> u8 {
        FILTER_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x1D
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        if buf.len() < 25 {
            return Err(SerializeError::OutOfSpace);
        }
        buf[0] = FunctionSelector::Write as u8;
        buf[1..5].copy_from_slice(&self.beta.x.to_be_bytes());
        buf[5..9].copy_from_slice(&self.beta.y.to_be_bytes());
        buf[9..13].copy_from_slice(&self.beta.z.to_be_bytes());
        buf[13..17].copy_from_slice(&self.noise.x.to_be_bytes());
        buf[17..21].copy_from_slice(&self.noise.y.to_be_bytes());
        buf[21..25].copy_from_slice(&self.noise.z.to_be_bytes());
        Ok(25)
    }
}
impl FilterCommandField for GyroBiasModelParametersWrite {}

impl_filter_selector_no_param_command!(GyroBiasModelParametersRead, 0x1D, FunctionSelector::Read);
impl_filter_selector_no_param_command!(GyroBiasModelParametersSave, 0x1D, FunctionSelector::Save);
impl_filter_selector_no_param_command!(GyroBiasModelParametersLoad, 0x1D, FunctionSelector::Load);
impl_filter_selector_no_param_command!(
    GyroBiasModelParametersDefault,
    0x1D,
    FunctionSelector::Default
);

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct ZeroAngularRateUpdateControlResponse {
    pub enable: bool,
    pub threshold_rad_s: f32,
}

impl CommandResponseData for ZeroAngularRateUpdateControlResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() < 5 {
            return Err(len_too_short(0x20, 5, data.len()));
        }
        let mut b = data;
        Ok(Self {
            enable: b.read_u8() != 0,
            threshold_rad_s: b.read_f32(),
        })
    }
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct ZeroAngularRateUpdateControlWrite {
    pub enable: bool,
    pub threshold_rad_s: f32,
}

impl CommandField for ZeroAngularRateUpdateControlWrite {
    type Response = ZeroAngularRateUpdateControlResponse;

    fn descriptor_set(&self) -> u8 {
        FILTER_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x20
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        if buf.len() < 6 {
            return Err(SerializeError::OutOfSpace);
        }
        buf[0] = FunctionSelector::Write as u8;
        buf[1] = self.enable as u8;
        buf[2..6].copy_from_slice(&self.threshold_rad_s.to_be_bytes());
        Ok(6)
    }
}
impl FilterCommandField for ZeroAngularRateUpdateControlWrite {}

impl_filter_selector_no_param_command!(
    ZeroAngularRateUpdateControlRead,
    0x20,
    FunctionSelector::Read
);
impl_filter_selector_no_param_command!(
    ZeroAngularRateUpdateControlSave,
    0x20,
    FunctionSelector::Save
);
impl_filter_selector_no_param_command!(
    ZeroAngularRateUpdateControlLoad,
    0x20,
    FunctionSelector::Load
);
impl_filter_selector_no_param_command!(
    ZeroAngularRateUpdateControlDefault,
    0x20,
    FunctionSelector::Default
);

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct GravityNoiseStandardDeviationResponse {
    pub noise: Vector3f,
}

impl CommandResponseData for GravityNoiseStandardDeviationResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        let mut b = data;
        Ok(Self {
            noise: Vector3f::read_from(&mut b, (FILTER_DESCRIPTOR_SET, 0x28))?,
        })
    }
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct GravityNoiseStandardDeviationWrite {
    pub noise: Vector3f,
}

impl CommandField for GravityNoiseStandardDeviationWrite {
    type Response = GravityNoiseStandardDeviationResponse;

    fn descriptor_set(&self) -> u8 {
        FILTER_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x28
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        if buf.len() < 13 {
            return Err(SerializeError::OutOfSpace);
        }
        buf[0] = FunctionSelector::Write as u8;
        buf[1..5].copy_from_slice(&self.noise.x.to_be_bytes());
        buf[5..9].copy_from_slice(&self.noise.y.to_be_bytes());
        buf[9..13].copy_from_slice(&self.noise.z.to_be_bytes());
        Ok(13)
    }
}
impl FilterCommandField for GravityNoiseStandardDeviationWrite {}

impl_filter_selector_no_param_command!(
    GravityNoiseStandardDeviationRead,
    0x28,
    FunctionSelector::Read
);
impl_filter_selector_no_param_command!(
    GravityNoiseStandardDeviationSave,
    0x28,
    FunctionSelector::Save
);
impl_filter_selector_no_param_command!(
    GravityNoiseStandardDeviationLoad,
    0x28,
    FunctionSelector::Load
);
impl_filter_selector_no_param_command!(
    GravityNoiseStandardDeviationDefault,
    0x28,
    FunctionSelector::Default
);

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct MagFieldDeclinationSourceControlResponse {
    pub source: DeclinationSource,
    pub declination_rad: f32,
}

impl CommandResponseData for MagFieldDeclinationSourceControlResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() < 5 {
            return Err(len_too_short(0x43, 5, data.len()));
        }
        let mut b = data;
        let source = DeclinationSource::from_u8(b.read_u8()).ok_or(ParseError::UnknownField {
            descriptor_set: FILTER_DESCRIPTOR_SET,
            descriptor: 0x43,
        })?;
        Ok(Self {
            source,
            declination_rad: b.read_f32(),
        })
    }
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct MagFieldDeclinationSourceControlWrite {
    pub source: DeclinationSource,
    pub declination_rad: f32,
}

impl CommandField for MagFieldDeclinationSourceControlWrite {
    type Response = MagFieldDeclinationSourceControlResponse;

    fn descriptor_set(&self) -> u8 {
        FILTER_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x43
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        if buf.len() < 6 {
            return Err(SerializeError::OutOfSpace);
        }
        buf[0] = FunctionSelector::Write as u8;
        buf[1] = self.source as u8;
        buf[2..6].copy_from_slice(&self.declination_rad.to_be_bytes());
        Ok(6)
    }
}
impl FilterCommandField for MagFieldDeclinationSourceControlWrite {}

impl_filter_selector_no_param_command!(
    MagFieldDeclinationSourceControlRead,
    0x43,
    FunctionSelector::Read
);
impl_filter_selector_no_param_command!(
    MagFieldDeclinationSourceControlSave,
    0x43,
    FunctionSelector::Save
);
impl_filter_selector_no_param_command!(
    MagFieldDeclinationSourceControlLoad,
    0x43,
    FunctionSelector::Load
);
impl_filter_selector_no_param_command!(
    MagFieldDeclinationSourceControlDefault,
    0x43,
    FunctionSelector::Default
);

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct AidingMeasurementControlResponse {
    pub source: AidingSource,
    pub enable: bool,
}

impl CommandResponseData for AidingMeasurementControlResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() < 3 {
            return Err(len_too_short(0x50, 3, data.len()));
        }
        let mut b = data;
        let source = AidingSource::from_u16(b.read_u16()).ok_or(ParseError::UnknownField {
            descriptor_set: FILTER_DESCRIPTOR_SET,
            descriptor: 0x50,
        })?;
        Ok(Self {
            source,
            enable: b.read_u8() != 0,
        })
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct AidingMeasurementControlWrite {
    pub source: AidingSource,
    pub enable: bool,
}

impl CommandField for AidingMeasurementControlWrite {
    type Response = AidingMeasurementControlResponse;

    fn descriptor_set(&self) -> u8 {
        FILTER_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x50
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        if buf.len() < 4 {
            return Err(SerializeError::OutOfSpace);
        }
        buf[0] = FunctionSelector::Write as u8;
        buf[1..3].copy_from_slice(&(self.source as u16).to_be_bytes());
        buf[3] = self.enable as u8;
        Ok(4)
    }
}
impl FilterCommandField for AidingMeasurementControlWrite {}

impl_filter_selector_source_command!(AidingMeasurementControlRead, 0x50, FunctionSelector::Read);
impl_filter_selector_source_command!(AidingMeasurementControlSave, 0x50, FunctionSelector::Save);
impl_filter_selector_source_command!(AidingMeasurementControlLoad, 0x50, FunctionSelector::Load);
impl_filter_selector_source_command!(
    AidingMeasurementControlDefault,
    0x50,
    FunctionSelector::Default
);

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct AdaptiveFilterControlResponse {
    pub level: AdaptiveFilterLevel,
    pub time_limit_ms: u16,
}

impl CommandResponseData for AdaptiveFilterControlResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() < 3 {
            return Err(len_too_short(0x53, 3, data.len()));
        }
        let mut b = data;
        let level = AdaptiveFilterLevel::from_u8(b.read_u8()).ok_or(ParseError::UnknownField {
            descriptor_set: FILTER_DESCRIPTOR_SET,
            descriptor: 0x53,
        })?;
        Ok(Self {
            level,
            time_limit_ms: b.read_u16(),
        })
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct AdaptiveFilterControlWrite {
    pub level: AdaptiveFilterLevel,
    pub time_limit_ms: u16,
}

impl CommandField for AdaptiveFilterControlWrite {
    type Response = AdaptiveFilterControlResponse;

    fn descriptor_set(&self) -> u8 {
        FILTER_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x53
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        if buf.len() < 4 {
            return Err(SerializeError::OutOfSpace);
        }
        buf[0] = FunctionSelector::Write as u8;
        buf[1] = self.level as u8;
        buf[2..4].copy_from_slice(&self.time_limit_ms.to_be_bytes());
        Ok(4)
    }
}
impl FilterCommandField for AdaptiveFilterControlWrite {}

impl_filter_selector_no_param_command!(AdaptiveFilterControlRead, 0x53, FunctionSelector::Read);
impl_filter_selector_no_param_command!(AdaptiveFilterControlSave, 0x53, FunctionSelector::Save);
impl_filter_selector_no_param_command!(AdaptiveFilterControlLoad, 0x53, FunctionSelector::Load);
impl_filter_selector_no_param_command!(
    AdaptiveFilterControlDefault,
    0x53,
    FunctionSelector::Default
);

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_response_type<C: CommandField<Response = R>, R>(_cmd: C) {}

    #[test]
    fn test_set_initial_heading_control_serialize() {
        let cmd = SetInitialHeadingControl { heading_rad: 1.25 };
        let mut buf = [0u8; 16];
        let cnt = cmd.serialize(&mut buf).unwrap();
        assert_eq!(6, cnt);
        assert_eq!(0x03, buf[1]);
        assert_eq!(1.25f32.to_be_bytes(), buf[2..6]);
    }

    #[test]
    fn test_external_heading_update_serialize() {
        let cmd = ExternalHeadingUpdate {
            heading_rad: 0.1,
            heading_uncertainty_rad: 0.2,
            heading_type: HeadingType::True,
        };
        let mut buf = [0u8; 16];
        let cnt = cmd.serialize(&mut buf).unwrap();
        assert_eq!(11, cnt);
        assert_eq!(0x17, buf[1]);
        assert_eq!(HeadingType::True as u8, buf[10]);
    }

    #[test]
    fn test_external_heading_update_with_time_serialize() {
        let cmd = ExternalHeadingUpdateWithTime {
            gps_time_s: 123.25,
            gps_week: 2300,
            heading_rad: -0.5,
            heading_uncertainty_rad: 0.05,
            heading_type: HeadingType::Magnetic,
        };
        let mut buf = [0u8; 32];
        let cnt = cmd.serialize(&mut buf).unwrap();
        assert_eq!(21, cnt);
        assert_eq!(0x1F, buf[1]);
        assert_eq!(123.25f64.to_be_bytes(), buf[2..10]);
        assert_eq!(2300u16.to_be_bytes(), buf[10..12]);
        assert_eq!(HeadingType::Magnetic as u8, buf[20]);
    }

    #[test]
    fn test_selector_suite_vector_shape() {
        let write = AccelNoiseStandardDeviationWrite {
            noise: Vector3f::new(1.0, 2.0, 3.0),
        };
        let mut buf = [0u8; 32];
        let cnt = write.serialize(&mut buf).unwrap();
        assert_eq!(15, cnt);
        assert_eq!(FunctionSelector::Write as u8, buf[2]);

        let read = AccelNoiseStandardDeviationRead;
        let save = AccelNoiseStandardDeviationSave;
        let load = AccelNoiseStandardDeviationLoad;
        let default = AccelNoiseStandardDeviationDefault;
        assert_eq!(3, read.serialize(&mut buf).unwrap());
        assert_eq!(FunctionSelector::Read as u8, buf[2]);
        assert_eq!(3, save.serialize(&mut buf).unwrap());
        assert_eq!(FunctionSelector::Save as u8, buf[2]);
        assert_eq!(3, load.serialize(&mut buf).unwrap());
        assert_eq!(FunctionSelector::Load as u8, buf[2]);
        assert_eq!(3, default.serialize(&mut buf).unwrap());
        assert_eq!(FunctionSelector::Default as u8, buf[2]);
    }

    #[test]
    fn test_selector_suite_two_vector_shape() {
        let write = AccelBiasModelParametersWrite {
            beta: Vector3f::new(0.1, 0.2, 0.3),
            noise: Vector3f::new(1.0, 1.1, 1.2),
        };
        let mut buf = [0u8; 48];
        let cnt = write.serialize(&mut buf).unwrap();
        assert_eq!(27, cnt);
        assert_eq!(FunctionSelector::Write as u8, buf[2]);

        let read = AccelBiasModelParametersRead;
        let save = AccelBiasModelParametersSave;
        let load = AccelBiasModelParametersLoad;
        let default = AccelBiasModelParametersDefault;
        assert_eq!(3, read.serialize(&mut buf).unwrap());
        assert_eq!(FunctionSelector::Read as u8, buf[2]);
        assert_eq!(3, save.serialize(&mut buf).unwrap());
        assert_eq!(FunctionSelector::Save as u8, buf[2]);
        assert_eq!(3, load.serialize(&mut buf).unwrap());
        assert_eq!(FunctionSelector::Load as u8, buf[2]);
        assert_eq!(3, default.serialize(&mut buf).unwrap());
        assert_eq!(FunctionSelector::Default as u8, buf[2]);
    }

    #[test]
    fn test_selector_suite_bool_float_shape() {
        let write = ZeroAngularRateUpdateControlWrite {
            enable: true,
            threshold_rad_s: 0.25,
        };
        let mut buf = [0u8; 32];
        let cnt = write.serialize(&mut buf).unwrap();
        assert_eq!(8, cnt);
        assert_eq!(FunctionSelector::Write as u8, buf[2]);

        let read = ZeroAngularRateUpdateControlRead;
        let save = ZeroAngularRateUpdateControlSave;
        let load = ZeroAngularRateUpdateControlLoad;
        let default = ZeroAngularRateUpdateControlDefault;
        assert_eq!(3, read.serialize(&mut buf).unwrap());
        assert_eq!(FunctionSelector::Read as u8, buf[2]);
        assert_eq!(3, save.serialize(&mut buf).unwrap());
        assert_eq!(FunctionSelector::Save as u8, buf[2]);
        assert_eq!(3, load.serialize(&mut buf).unwrap());
        assert_eq!(FunctionSelector::Load as u8, buf[2]);
        assert_eq!(3, default.serialize(&mut buf).unwrap());
        assert_eq!(FunctionSelector::Default as u8, buf[2]);
    }

    #[test]
    fn test_selector_suite_enum_float_shape() {
        let write = MagFieldDeclinationSourceControlWrite {
            source: DeclinationSource::Manual,
            declination_rad: 0.7,
        };
        let mut buf = [0u8; 32];
        let cnt = write.serialize(&mut buf).unwrap();
        assert_eq!(8, cnt);
        assert_eq!(FunctionSelector::Write as u8, buf[2]);

        let read = MagFieldDeclinationSourceControlRead;
        let save = MagFieldDeclinationSourceControlSave;
        let load = MagFieldDeclinationSourceControlLoad;
        let default = MagFieldDeclinationSourceControlDefault;
        assert_eq!(3, read.serialize(&mut buf).unwrap());
        assert_eq!(FunctionSelector::Read as u8, buf[2]);
        assert_eq!(3, save.serialize(&mut buf).unwrap());
        assert_eq!(FunctionSelector::Save as u8, buf[2]);
        assert_eq!(3, load.serialize(&mut buf).unwrap());
        assert_eq!(FunctionSelector::Load as u8, buf[2]);
        assert_eq!(3, default.serialize(&mut buf).unwrap());
        assert_eq!(FunctionSelector::Default as u8, buf[2]);
    }

    #[test]
    fn test_selector_suite_source_bool_shape() {
        let write = AidingMeasurementControlWrite {
            source: AidingSource::ExternalHeading,
            enable: true,
        };
        let mut buf = [0u8; 32];
        let cnt = write.serialize(&mut buf).unwrap();
        assert_eq!(6, cnt);
        assert_eq!(FunctionSelector::Write as u8, buf[2]);
        assert_eq!(
            (AidingSource::ExternalHeading as u16).to_be_bytes(),
            buf[3..5]
        );

        let read = AidingMeasurementControlRead {
            source: AidingSource::ExternalHeading,
        };
        let save = AidingMeasurementControlSave {
            source: AidingSource::ExternalHeading,
        };
        let load = AidingMeasurementControlLoad {
            source: AidingSource::ExternalHeading,
        };
        let default = AidingMeasurementControlDefault {
            source: AidingSource::ExternalHeading,
        };
        assert_eq!(5, read.serialize(&mut buf).unwrap());
        assert_eq!(FunctionSelector::Read as u8, buf[2]);
        assert_eq!(5, save.serialize(&mut buf).unwrap());
        assert_eq!(FunctionSelector::Save as u8, buf[2]);
        assert_eq!(5, load.serialize(&mut buf).unwrap());
        assert_eq!(FunctionSelector::Load as u8, buf[2]);
        assert_eq!(5, default.serialize(&mut buf).unwrap());
        assert_eq!(FunctionSelector::Default as u8, buf[2]);
    }

    #[test]
    fn test_selector_suite_u8_u16_shape() {
        let write = AdaptiveFilterControlWrite {
            level: AdaptiveFilterLevel::Aggressive,
            time_limit_ms: 500,
        };
        let mut buf = [0u8; 16];
        let cnt = write.serialize(&mut buf).unwrap();
        assert_eq!(6, cnt);
        assert_eq!(FunctionSelector::Write as u8, buf[2]);

        let read = AdaptiveFilterControlRead;
        let save = AdaptiveFilterControlSave;
        let load = AdaptiveFilterControlLoad;
        let default = AdaptiveFilterControlDefault;
        assert_eq!(3, read.serialize(&mut buf).unwrap());
        assert_eq!(FunctionSelector::Read as u8, buf[2]);
        assert_eq!(3, save.serialize(&mut buf).unwrap());
        assert_eq!(FunctionSelector::Save as u8, buf[2]);
        assert_eq!(3, load.serialize(&mut buf).unwrap());
        assert_eq!(FunctionSelector::Load as u8, buf[2]);
        assert_eq!(3, default.serialize(&mut buf).unwrap());
        assert_eq!(FunctionSelector::Default as u8, buf[2]);
    }

    #[test]
    fn test_write_response_parsing_success() {
        let v = [
            1.0f32.to_be_bytes(),
            2.0f32.to_be_bytes(),
            3.0f32.to_be_bytes(),
        ]
        .concat();
        assert_eq!(
            Vector3f::new(1.0, 2.0, 3.0),
            AccelNoiseStandardDeviationResponse::from_data(&v)
                .unwrap()
                .noise
        );
        assert_eq!(
            Vector3f::new(1.0, 2.0, 3.0),
            GyroNoiseStandardDeviationResponse::from_data(&v)
                .unwrap()
                .noise
        );
        assert_eq!(
            Vector3f::new(1.0, 2.0, 3.0),
            GravityNoiseStandardDeviationResponse::from_data(&v)
                .unwrap()
                .noise
        );

        let two_v = [
            0.1f32.to_be_bytes(),
            0.2f32.to_be_bytes(),
            0.3f32.to_be_bytes(),
            1.1f32.to_be_bytes(),
            1.2f32.to_be_bytes(),
            1.3f32.to_be_bytes(),
        ]
        .concat();
        let accel_bias = AccelBiasModelParametersResponse::from_data(&two_v).unwrap();
        assert_eq!(Vector3f::new(0.1, 0.2, 0.3), accel_bias.beta);
        assert_eq!(Vector3f::new(1.1, 1.2, 1.3), accel_bias.noise);
        let gyro_bias = GyroBiasModelParametersResponse::from_data(&two_v).unwrap();
        assert_eq!(Vector3f::new(0.1, 0.2, 0.3), gyro_bias.beta);
        assert_eq!(Vector3f::new(1.1, 1.2, 1.3), gyro_bias.noise);

        let zaru = [
            1u8,
            0.25f32.to_be_bytes()[0],
            0.25f32.to_be_bytes()[1],
            0.25f32.to_be_bytes()[2],
            0.25f32.to_be_bytes()[3],
        ];
        let zaru = ZeroAngularRateUpdateControlResponse::from_data(&zaru).unwrap();
        assert!(zaru.enable);
        assert_eq!(0.25, zaru.threshold_rad_s);

        let dec = [
            DeclinationSource::Wmm as u8,
            (-0.8f32).to_be_bytes()[0],
            (-0.8f32).to_be_bytes()[1],
            (-0.8f32).to_be_bytes()[2],
            (-0.8f32).to_be_bytes()[3],
        ];
        let dec = MagFieldDeclinationSourceControlResponse::from_data(&dec).unwrap();
        assert_eq!(DeclinationSource::Wmm, dec.source);
        assert_eq!(-0.8, dec.declination_rad);

        let aid = [0u8, 5u8, 1u8];
        let aid = AidingMeasurementControlResponse::from_data(&aid).unwrap();
        assert_eq!(AidingSource::ExternalHeading, aid.source);
        assert!(aid.enable);

        let adp = [AdaptiveFilterLevel::Moderate as u8, 0x01, 0xF4];
        let adp = AdaptiveFilterControlResponse::from_data(&adp).unwrap();
        assert_eq!(AdaptiveFilterLevel::Moderate, adp.level);
        assert_eq!(500, adp.time_limit_ms);
    }

    #[test]
    fn test_write_response_parsing_short_len() {
        assert!(matches!(
            AccelNoiseStandardDeviationResponse::from_data(&[]),
            Err(ParseError::LenTooShort { .. })
        ));
        assert!(matches!(
            GyroNoiseStandardDeviationResponse::from_data(&[]),
            Err(ParseError::LenTooShort { .. })
        ));
        assert!(matches!(
            AccelBiasModelParametersResponse::from_data(&[]),
            Err(ParseError::LenTooShort { .. })
        ));
        assert!(matches!(
            GyroBiasModelParametersResponse::from_data(&[]),
            Err(ParseError::LenTooShort { .. })
        ));
        assert!(matches!(
            ZeroAngularRateUpdateControlResponse::from_data(&[]),
            Err(ParseError::LenTooShort { .. })
        ));
        assert!(matches!(
            GravityNoiseStandardDeviationResponse::from_data(&[]),
            Err(ParseError::LenTooShort { .. })
        ));
        assert!(matches!(
            MagFieldDeclinationSourceControlResponse::from_data(&[]),
            Err(ParseError::LenTooShort { .. })
        ));
        assert!(matches!(
            AidingMeasurementControlResponse::from_data(&[]),
            Err(ParseError::LenTooShort { .. })
        ));
        assert!(matches!(
            AdaptiveFilterControlResponse::from_data(&[]),
            Err(ParseError::LenTooShort { .. })
        ));
    }

    #[test]
    fn test_response_type_distinction() {
        assert_response_type::<AccelNoiseStandardDeviationWrite, AccelNoiseStandardDeviationResponse>(
            AccelNoiseStandardDeviationWrite {
                noise: Vector3f::new(0.0, 0.0, 0.0),
            },
        );
        assert_response_type::<AccelNoiseStandardDeviationRead, ()>(
            AccelNoiseStandardDeviationRead,
        );
    }
}
