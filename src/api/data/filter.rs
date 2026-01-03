//! Filter descriptor set (0x82) data fields (full coverage).

use crate::api::data::{Matrix3f, Quatf, Vector3d, Vector3f};

use super::{ensure_len, Error, ReadBuf};

pub const FILTER_DESCRIPTOR_SET: u8 = 0x82;

/// A parsed Filter (0x82) data field.
#[derive(Debug, Clone)]
pub enum FilterField {
    LlhPosition(LlhPosition),                                     // 0x01
    VelocityNed(VelocityNed),                                     // 0x02
    AttitudeQuaternion(AttitudeQuaternion),                       // 0x03
    AttitudeDcm(AttitudeDcm),                                     // 0x04
    EulerAngles(EulerAngles),                                     // 0x05
    GyroBias(GyroBias),                                           // 0x06
    AccelBias(AccelBias),                                         // 0x07
    LlhPositionUncertainty(LlhPositionUncertainty),               // 0x08
    NedVelocityUncertainty(NedVelocityUncertainty),               // 0x09
    EulerAnglesUncertainty(EulerAnglesUncertainty),               // 0x0A
    GyroBiasUncertainty(GyroBiasUncertainty),                     // 0x0B
    AccelBiasUncertainty(AccelBiasUncertainty),                   // 0x0C
    LinearAccel(LinearAccel),                                     // 0x0D
    CompAngularRate(CompAngularRate),                             // 0x0E
    Status(Status),                                               // 0x10
    Timestamp(Timestamp),                                         // 0x11
    QuaternionAttitudeUncertainty(QuaternionAttitudeUncertainty), // 0x12
    GravityVector(GravityVector),                                 // 0x13
    MagneticModel(MagneticModel),                                 // 0x15
    CompensatedAcceleration(CompensatedAcceleration),             // 0x1C
    PressureAltitude(PressureAltitude),                           // 0x21
    MultiAntennaOffsetCorrection(MultiAntennaOffsetCorrection),   // 0x34
    MultiAntennaOffsetCorrectionUncertainty(MultiAntennaOffsetCorrectionUncertainty), // 0x35
    EcefPositionUncertainty(EcefPositionUncertainty),             // 0x36
    EcefVelocityUncertainty(EcefVelocityUncertainty),             // 0x37
    EcefPosition(EcefPosition),                                   // 0x40
    EcefVelocity(EcefVelocity),                                   // 0x41
    NedRelativePosition(NedRelativePosition),                     // 0x42
    GnssPositionAidingStatus(GnssPositionAidingStatus),           // 0x43
    AidingMeasurementSummary(AidingMeasurementSummary),           // 0x46
    OdometerScaleFactorError(OdometerScaleFactorError),           // 0x47
    OdometerScaleFactorErrorUncertainty(OdometerScaleFactorErrorUncertainty), // 0x48
    GnssDualAntennaStatus(GnssDualAntennaStatus),                 // 0x49
    AidingFrameConfigurationError(AidingFrameConfigurationError), // 0x50
    AidingFrameConfigurationErrorUncertainty(AidingFrameConfigurationErrorUncertainty), // 0x51

    /// Any unrecognized field descriptor for set 0x82.
    Unknown {
        descriptor: u8,
    },
}

impl FilterField {
    /// Parse a single Filter (0x82) field given the *field descriptor* and its raw field payload bytes.
    pub fn parse(descriptor: u8, bytes: &[u8]) -> Result<Self, Error> {
        Ok(match descriptor {
            LlhPosition::DESCRIPTOR => Self::LlhPosition(LlhPosition::from_bytes(bytes)?),
            VelocityNed::DESCRIPTOR => Self::VelocityNed(VelocityNed::from_bytes(bytes)?),
            AttitudeQuaternion::DESCRIPTOR => {
                Self::AttitudeQuaternion(AttitudeQuaternion::from_bytes(bytes)?)
            }
            AttitudeDcm::DESCRIPTOR => Self::AttitudeDcm(AttitudeDcm::from_bytes(bytes)?),
            EulerAngles::DESCRIPTOR => Self::EulerAngles(EulerAngles::from_bytes(bytes)?),
            GyroBias::DESCRIPTOR => Self::GyroBias(GyroBias::from_bytes(bytes)?),
            AccelBias::DESCRIPTOR => Self::AccelBias(AccelBias::from_bytes(bytes)?),

            LlhPositionUncertainty::DESCRIPTOR => {
                Self::LlhPositionUncertainty(LlhPositionUncertainty::from_bytes(bytes)?)
            }
            NedVelocityUncertainty::DESCRIPTOR => {
                Self::NedVelocityUncertainty(NedVelocityUncertainty::from_bytes(bytes)?)
            }
            EulerAnglesUncertainty::DESCRIPTOR => {
                Self::EulerAnglesUncertainty(EulerAnglesUncertainty::from_bytes(bytes)?)
            }
            GyroBiasUncertainty::DESCRIPTOR => {
                Self::GyroBiasUncertainty(GyroBiasUncertainty::from_bytes(bytes)?)
            }
            AccelBiasUncertainty::DESCRIPTOR => {
                Self::AccelBiasUncertainty(AccelBiasUncertainty::from_bytes(bytes)?)
            }
            LinearAccel::DESCRIPTOR => Self::LinearAccel(LinearAccel::from_bytes(bytes)?),
            CompAngularRate::DESCRIPTOR => {
                Self::CompAngularRate(CompAngularRate::from_bytes(bytes)?)
            }

            Status::DESCRIPTOR => Self::Status(Status::from_bytes(bytes)?),
            Timestamp::DESCRIPTOR => Self::Timestamp(Timestamp::from_bytes(bytes)?),
            QuaternionAttitudeUncertainty::DESCRIPTOR => Self::QuaternionAttitudeUncertainty(
                QuaternionAttitudeUncertainty::from_bytes(bytes)?,
            ),
            GravityVector::DESCRIPTOR => Self::GravityVector(GravityVector::from_bytes(bytes)?),
            MagneticModel::DESCRIPTOR => Self::MagneticModel(MagneticModel::from_bytes(bytes)?),

            CompensatedAcceleration::DESCRIPTOR => {
                Self::CompensatedAcceleration(CompensatedAcceleration::from_bytes(bytes)?)
            }
            PressureAltitude::DESCRIPTOR => {
                Self::PressureAltitude(PressureAltitude::from_bytes(bytes)?)
            }

            MultiAntennaOffsetCorrection::DESCRIPTOR => {
                Self::MultiAntennaOffsetCorrection(MultiAntennaOffsetCorrection::from_bytes(bytes)?)
            }
            MultiAntennaOffsetCorrectionUncertainty::DESCRIPTOR => {
                Self::MultiAntennaOffsetCorrectionUncertainty(
                    MultiAntennaOffsetCorrectionUncertainty::from_bytes(bytes)?,
                )
            }
            EcefPositionUncertainty::DESCRIPTOR => {
                Self::EcefPositionUncertainty(EcefPositionUncertainty::from_bytes(bytes)?)
            }
            EcefVelocityUncertainty::DESCRIPTOR => {
                Self::EcefVelocityUncertainty(EcefVelocityUncertainty::from_bytes(bytes)?)
            }

            EcefPosition::DESCRIPTOR => Self::EcefPosition(EcefPosition::from_bytes(bytes)?),
            EcefVelocity::DESCRIPTOR => Self::EcefVelocity(EcefVelocity::from_bytes(bytes)?),
            NedRelativePosition::DESCRIPTOR => {
                Self::NedRelativePosition(NedRelativePosition::from_bytes(bytes)?)
            }
            GnssPositionAidingStatus::DESCRIPTOR => {
                Self::GnssPositionAidingStatus(GnssPositionAidingStatus::from_bytes(bytes)?)
            }

            AidingMeasurementSummary::DESCRIPTOR => {
                Self::AidingMeasurementSummary(AidingMeasurementSummary::from_bytes(bytes)?)
            }
            OdometerScaleFactorError::DESCRIPTOR => {
                Self::OdometerScaleFactorError(OdometerScaleFactorError::from_bytes(bytes)?)
            }
            OdometerScaleFactorErrorUncertainty::DESCRIPTOR => {
                Self::OdometerScaleFactorErrorUncertainty(
                    OdometerScaleFactorErrorUncertainty::from_bytes(bytes)?,
                )
            }
            GnssDualAntennaStatus::DESCRIPTOR => {
                Self::GnssDualAntennaStatus(GnssDualAntennaStatus::from_bytes(bytes)?)
            }

            AidingFrameConfigurationError::DESCRIPTOR => Self::AidingFrameConfigurationError(
                AidingFrameConfigurationError::from_bytes(bytes)?,
            ),
            AidingFrameConfigurationErrorUncertainty::DESCRIPTOR => {
                Self::AidingFrameConfigurationErrorUncertainty(
                    AidingFrameConfigurationErrorUncertainty::from_bytes(bytes)?,
                )
            }

            other => Self::Unknown { descriptor: other },
        })
    }
}

// -------------------------
// Typed bitfields / enums (kept “raw” where device-specific)
// -------------------------

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct ValidFlags(pub u16);

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct StatusFlags(pub u16);

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct AidingStatusFlags(pub u16);

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct GnssDualAntennaStatusFlags(pub u16);

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct AidingIndicatorFlags(pub u8);

#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum DualAntennaFixType {
    FixNone = 0,
    FixDaFloat = 1,
    FixDaFixed = 2,
    Unknown = 0xFF,
}
impl DualAntennaFixType {
    #[inline]
    pub fn from_u8(v: u8) -> Self {
        match v {
            0 => Self::FixNone,
            1 => Self::FixDaFloat,
            2 => Self::FixDaFixed,
            _ => Self::Unknown,
        }
    }
}

// -------------------------
// Field structs (full set)
// -------------------------

/// (0x82,0x01) LLH Position
#[derive(Debug, Copy, Clone)]
pub struct LlhPosition {
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub ellipsoid_height_m: f64,
    pub valid_flags: ValidFlags,
}
impl LlhPosition {
    pub const DESCRIPTOR: u8 = 0x01;
    pub const LEN: usize = 8 + 8 + 8 + 2; // 26

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::LEN, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let mut b = bytes;
        Ok(Self {
            latitude_deg: b.read_f64(),
            longitude_deg: b.read_f64(),
            ellipsoid_height_m: b.read_f64(),
            valid_flags: ValidFlags(b.read_u16()),
        })
    }
}

/// (0x82,0x02) Velocity NED
#[derive(Debug, Copy, Clone)]
pub struct VelocityNed {
    pub north_m_s: f32,
    pub east_m_s: f32,
    pub down_m_s: f32,
    pub valid_flags: ValidFlags,
}
impl VelocityNed {
    pub const DESCRIPTOR: u8 = 0x02;
    pub const LEN: usize = 4 + 4 + 4 + 2; // 14

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::LEN, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let mut b = bytes;
        Ok(Self {
            north_m_s: b.read_f32(),
            east_m_s: b.read_f32(),
            down_m_s: b.read_f32(),
            valid_flags: ValidFlags(b.read_u16()),
        })
    }
}

/// (0x82,0x03) Attitude Quaternion
#[derive(Debug, Copy, Clone)]
pub struct AttitudeQuaternion {
    pub q: Quatf,
    pub valid_flags: ValidFlags,
}
impl AttitudeQuaternion {
    pub const DESCRIPTOR: u8 = 0x03;
    pub const LEN: usize = Quatf::LEN + 2; // 18

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::LEN, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let mut b = bytes;
        Ok(Self {
            q: Quatf::read_from(&mut b, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?,
            valid_flags: ValidFlags(b.read_u16()),
        })
    }
}

/// (0x82,0x04) Attitude DCM (row-major)
#[derive(Debug, Copy, Clone)]
pub struct AttitudeDcm {
    pub dcm: Matrix3f,
    pub valid_flags: ValidFlags,
}
impl AttitudeDcm {
    pub const DESCRIPTOR: u8 = 0x04;
    pub const LEN: usize = Matrix3f::LEN + 2; // 38

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::LEN, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let mut b = bytes;
        Ok(Self {
            dcm: Matrix3f::read_from(&mut b, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?,
            valid_flags: ValidFlags(b.read_u16()),
        })
    }
}

/// (0x82,0x05) Euler Angles (3-2-1 / yaw-pitch-roll order, but reported as roll/pitch/yaw)
#[derive(Debug, Copy, Clone)]
pub struct EulerAngles {
    pub roll_rad: f32,
    pub pitch_rad: f32,
    pub yaw_rad: f32,
    pub valid_flags: ValidFlags,
}
impl EulerAngles {
    pub const DESCRIPTOR: u8 = 0x05;
    pub const LEN: usize = 4 + 4 + 4 + 2; // 14

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::LEN, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let mut b = bytes;
        Ok(Self {
            roll_rad: b.read_f32(),
            pitch_rad: b.read_f32(),
            yaw_rad: b.read_f32(),
            valid_flags: ValidFlags(b.read_u16()),
        })
    }
}

/// (0x82,0x06) Gyro Bias (sensor frame)
#[derive(Debug, Copy, Clone)]
pub struct GyroBias {
    pub bias_rad_s: Vector3f,
    pub valid_flags: ValidFlags,
}
impl GyroBias {
    pub const DESCRIPTOR: u8 = 0x06;
    pub const LEN: usize = Vector3f::LEN + 2; // 14

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::LEN, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let mut b = bytes;
        Ok(Self {
            bias_rad_s: Vector3f::read_from(&mut b, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?,
            valid_flags: ValidFlags(b.read_u16()),
        })
    }
}

/// (0x82,0x07) Accel Bias (sensor frame)
#[derive(Debug, Copy, Clone)]
pub struct AccelBias {
    pub bias_m_s2: Vector3f,
    pub valid_flags: ValidFlags,
}
impl AccelBias {
    pub const DESCRIPTOR: u8 = 0x07;
    pub const LEN: usize = Vector3f::LEN + 2; // 14

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::LEN, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let mut b = bytes;
        Ok(Self {
            bias_m_s2: Vector3f::read_from(&mut b, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?,
            valid_flags: ValidFlags(b.read_u16()),
        })
    }
}

/// (0x82,0x08) LLH Position Uncertainty (1-sigma, NED)
#[derive(Debug, Copy, Clone)]
pub struct LlhPositionUncertainty {
    pub north_m: f32,
    pub east_m: f32,
    pub down_m: f32,
    pub valid_flags: ValidFlags,
}
impl LlhPositionUncertainty {
    pub const DESCRIPTOR: u8 = 0x08;
    pub const LEN: usize = 4 + 4 + 4 + 2; // 14

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::LEN, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let mut b = bytes;
        Ok(Self {
            north_m: b.read_f32(),
            east_m: b.read_f32(),
            down_m: b.read_f32(),
            valid_flags: ValidFlags(b.read_u16()),
        })
    }
}

/// (0x82,0x09) NED Velocity Uncertainty (1-sigma)
#[derive(Debug, Copy, Clone)]
pub struct NedVelocityUncertainty {
    pub north_m_s: f32,
    pub east_m_s: f32,
    pub down_m_s: f32,
    pub valid_flags: ValidFlags,
}
impl NedVelocityUncertainty {
    pub const DESCRIPTOR: u8 = 0x09;
    pub const LEN: usize = 4 + 4 + 4 + 2; // 14

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::LEN, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let mut b = bytes;
        Ok(Self {
            north_m_s: b.read_f32(),
            east_m_s: b.read_f32(),
            down_m_s: b.read_f32(),
            valid_flags: ValidFlags(b.read_u16()),
        })
    }
}

/// (0x82,0x0A) Euler Angles Uncertainty (1-sigma)
#[derive(Debug, Copy, Clone)]
pub struct EulerAnglesUncertainty {
    pub roll_rad: f32,
    pub pitch_rad: f32,
    pub yaw_rad: f32,
    pub valid_flags: ValidFlags,
}
impl EulerAnglesUncertainty {
    pub const DESCRIPTOR: u8 = 0x0A;
    pub const LEN: usize = 4 + 4 + 4 + 2; // 14

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::LEN, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let mut b = bytes;
        Ok(Self {
            roll_rad: b.read_f32(),
            pitch_rad: b.read_f32(),
            yaw_rad: b.read_f32(),
            valid_flags: ValidFlags(b.read_u16()),
        })
    }
}

/// (0x82,0x0B) Gyro Bias Uncertainty (1-sigma)
#[derive(Debug, Copy, Clone)]
pub struct GyroBiasUncertainty {
    pub bias_uncert_rad_s: Vector3f,
    pub valid_flags: ValidFlags,
}
impl GyroBiasUncertainty {
    pub const DESCRIPTOR: u8 = 0x0B;
    pub const LEN: usize = Vector3f::LEN + 2; // 14

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::LEN, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let mut b = bytes;
        Ok(Self {
            bias_uncert_rad_s: Vector3f::read_from(&mut b, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?,
            valid_flags: ValidFlags(b.read_u16()),
        })
    }
}

/// (0x82,0x0C) Accel Bias Uncertainty (1-sigma)
#[derive(Debug, Copy, Clone)]
pub struct AccelBiasUncertainty {
    pub bias_uncert_m_s2: Vector3f,
    pub valid_flags: ValidFlags,
}
impl AccelBiasUncertainty {
    pub const DESCRIPTOR: u8 = 0x0C;
    pub const LEN: usize = Vector3f::LEN + 2; // 14

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::LEN, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let mut b = bytes;
        Ok(Self {
            bias_uncert_m_s2: Vector3f::read_from(&mut b, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?,
            valid_flags: ValidFlags(b.read_u16()),
        })
    }
}

/// (0x82,0x0D) Linear Accel (vehicle frame, gravity removed)
#[derive(Debug, Copy, Clone)]
pub struct LinearAccel {
    pub accel_m_s2: Vector3f,
    pub valid_flags: ValidFlags,
}
impl LinearAccel {
    pub const DESCRIPTOR: u8 = 0x0D;
    pub const LEN: usize = Vector3f::LEN + 2; // 14

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::LEN, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let mut b = bytes;
        Ok(Self {
            accel_m_s2: Vector3f::read_from(&mut b, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?,
            valid_flags: ValidFlags(b.read_u16()),
        })
    }
}

/// (0x82,0x0E) Comp Angular Rate (vehicle frame)
#[derive(Debug, Copy, Clone)]
pub struct CompAngularRate {
    pub gyro_rad_s: Vector3f,
    pub valid_flags: ValidFlags,
}
impl CompAngularRate {
    pub const DESCRIPTOR: u8 = 0x0E;
    pub const LEN: usize = Vector3f::LEN + 2; // 14

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::LEN, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let mut b = bytes;
        Ok(Self {
            gyro_rad_s: Vector3f::read_from(&mut b, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?,
            valid_flags: ValidFlags(b.read_u16()),
        })
    }
}

/// (0x82,0x10) Status (device-specific enums/bitfields)
#[derive(Debug, Copy, Clone)]
pub struct Status {
    pub filter_state: u16,
    pub dynamics_mode: u16,
    pub status_flags: StatusFlags,
}
impl Status {
    pub const DESCRIPTOR: u8 = 0x10;
    pub const LEN: usize = 2 + 2 + 2; // 6

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::LEN, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let mut b = bytes;
        Ok(Self {
            filter_state: b.read_u16(),
            dynamics_mode: b.read_u16(),
            status_flags: StatusFlags(b.read_u16()),
        })
    }
}

/// (0x82,0x11) Timestamp (GPS TOW + week)
#[derive(Debug, Copy, Clone)]
pub struct Timestamp {
    pub tow_s: f64,
    pub week_number: u16,
    pub valid_flags: ValidFlags,
}
impl Timestamp {
    pub const DESCRIPTOR: u8 = 0x11;
    pub const LEN: usize = 8 + 2 + 2; // 12

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::LEN, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let mut b = bytes;
        Ok(Self {
            tow_s: b.read_f64(),
            week_number: b.read_u16(),
            valid_flags: ValidFlags(b.read_u16()),
        })
    }
}

/// (0x82,0x12) Quaternion Attitude Uncertainty
#[derive(Debug, Copy, Clone)]
pub struct QuaternionAttitudeUncertainty {
    pub q: Quatf,
    pub valid_flags: ValidFlags,
}
impl QuaternionAttitudeUncertainty {
    pub const DESCRIPTOR: u8 = 0x12;
    pub const LEN: usize = Quatf::LEN + 2; // 18

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::LEN, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let mut b = bytes;
        Ok(Self {
            q: Quatf::read_from(&mut b, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?,
            valid_flags: ValidFlags(b.read_u16()),
        })
    }
}

/// (0x82,0x13) Gravity Vector (vehicle frame)
#[derive(Debug, Copy, Clone)]
pub struct GravityVector {
    pub gravity_m_s2: Vector3f,
    pub valid_flags: ValidFlags,
}
impl GravityVector {
    pub const DESCRIPTOR: u8 = 0x13;
    pub const LEN: usize = Vector3f::LEN + 2; // 14

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::LEN, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let mut b = bytes;
        Ok(Self {
            gravity_m_s2: Vector3f::read_from(&mut b, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?,
            valid_flags: ValidFlags(b.read_u16()),
        })
    }
}

/// (0x82,0x15) Magnetic Model (WMM outputs)
#[derive(Debug, Copy, Clone)]
pub struct MagneticModel {
    pub intensity_north_gauss: f32,
    pub intensity_east_gauss: f32,
    pub intensity_down_gauss: f32,
    pub inclination_rad: f32,
    pub declination_rad: f32,
    pub valid_flags: ValidFlags,
}
impl MagneticModel {
    pub const DESCRIPTOR: u8 = 0x15;
    pub const LEN: usize = 4 * 5 + 2; // 22

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::LEN, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let mut b = bytes;
        Ok(Self {
            intensity_north_gauss: b.read_f32(),
            intensity_east_gauss: b.read_f32(),
            intensity_down_gauss: b.read_f32(),
            inclination_rad: b.read_f32(),
            declination_rad: b.read_f32(),
            valid_flags: ValidFlags(b.read_u16()),
        })
    }
}

/// (0x82,0x1C) Compensated Acceleration (vehicle frame)
#[derive(Debug, Copy, Clone)]
pub struct CompensatedAcceleration {
    pub accel_m_s2: Vector3f,
    pub valid_flags: ValidFlags,
}
impl CompensatedAcceleration {
    pub const DESCRIPTOR: u8 = 0x1C;
    pub const LEN: usize = Vector3f::LEN + 2; // 14

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::LEN, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let mut b = bytes;
        Ok(Self {
            accel_m_s2: Vector3f::read_from(&mut b, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?,
            valid_flags: ValidFlags(b.read_u16()),
        })
    }
}

/// (0x82,0x21) Pressure Altitude
#[derive(Debug, Copy, Clone)]
pub struct PressureAltitude {
    pub pressure_altitude_m: f32,
    pub valid_flags: ValidFlags,
}
impl PressureAltitude {
    pub const DESCRIPTOR: u8 = 0x21;
    pub const LEN: usize = 4 + 2; // 6

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::LEN, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let mut b = bytes;
        Ok(Self {
            pressure_altitude_m: b.read_f32(),
            valid_flags: ValidFlags(b.read_u16()),
        })
    }
}

/// (0x82,0x34) Multi Antenna Offset Correction
#[derive(Debug, Copy, Clone)]
pub struct MultiAntennaOffsetCorrection {
    pub receiver_id: u8,
    pub offset_m: Vector3f,
    pub valid_flags: ValidFlags,
}
impl MultiAntennaOffsetCorrection {
    pub const DESCRIPTOR: u8 = 0x34;
    pub const LEN: usize = 1 + Vector3f::LEN + 2; // 15

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::LEN, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let mut b = bytes;
        Ok(Self {
            receiver_id: b.read_u8(),
            offset_m: Vector3f::read_from(&mut b, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?,
            valid_flags: ValidFlags(b.read_u16()),
        })
    }
}

/// (0x82,0x35) Multi Antenna Offset Correction Uncertainty (1-sigma)
#[derive(Debug, Copy, Clone)]
pub struct MultiAntennaOffsetCorrectionUncertainty {
    pub receiver_id: u8,
    pub offset_uncert_m: Vector3f,
    pub valid_flags: ValidFlags,
}
impl MultiAntennaOffsetCorrectionUncertainty {
    pub const DESCRIPTOR: u8 = 0x35;
    pub const LEN: usize = 1 + Vector3f::LEN + 2; // 15

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::LEN, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let mut b = bytes;
        Ok(Self {
            receiver_id: b.read_u8(),
            offset_uncert_m: Vector3f::read_from(&mut b, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?,
            valid_flags: ValidFlags(b.read_u16()),
        })
    }
}

/// (0x82,0x36) ECEF Position Uncertainty (1-sigma)
#[derive(Debug, Copy, Clone)]
pub struct EcefPositionUncertainty {
    pub pos_uncert_m: Vector3f,
    pub valid_flags: ValidFlags,
}
impl EcefPositionUncertainty {
    pub const DESCRIPTOR: u8 = 0x36;
    pub const LEN: usize = Vector3f::LEN + 2; // 14

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::LEN, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let mut b = bytes;
        Ok(Self {
            pos_uncert_m: Vector3f::read_from(&mut b, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?,
            valid_flags: ValidFlags(b.read_u16()),
        })
    }
}

/// (0x82,0x37) ECEF Velocity Uncertainty (1-sigma)
#[derive(Debug, Copy, Clone)]
pub struct EcefVelocityUncertainty {
    pub vel_uncert_m_s: Vector3f,
    pub valid_flags: ValidFlags,
}
impl EcefVelocityUncertainty {
    pub const DESCRIPTOR: u8 = 0x37;
    pub const LEN: usize = Vector3f::LEN + 2; // 14

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::LEN, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let mut b = bytes;
        Ok(Self {
            vel_uncert_m_s: Vector3f::read_from(&mut b, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?,
            valid_flags: ValidFlags(b.read_u16()),
        })
    }
}

/// (0x82,0x40) ECEF Position
#[derive(Debug, Copy, Clone)]
pub struct EcefPosition {
    pub position_ecef_m: Vector3d,
    pub valid_flags: ValidFlags,
}
impl EcefPosition {
    pub const DESCRIPTOR: u8 = 0x40;
    pub const LEN: usize = Vector3d::LEN + 2; // 26

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::LEN, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let mut b = bytes;
        Ok(Self {
            position_ecef_m: Vector3d::read_from(&mut b, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?,
            valid_flags: ValidFlags(b.read_u16()),
        })
    }
}

/// (0x82,0x41) ECEF Velocity
#[derive(Debug, Copy, Clone)]
pub struct EcefVelocity {
    pub velocity_ecef_m_s: Vector3f,
    pub valid_flags: ValidFlags,
}
impl EcefVelocity {
    pub const DESCRIPTOR: u8 = 0x41;
    pub const LEN: usize = Vector3f::LEN + 2; // 14

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::LEN, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let mut b = bytes;
        Ok(Self {
            velocity_ecef_m_s: Vector3f::read_from(&mut b, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?,
            valid_flags: ValidFlags(b.read_u16()),
        })
    }
}

/// (0x82,0x42) NED Relative Position (w.r.t configured reference position)
#[derive(Debug, Copy, Clone)]
pub struct NedRelativePosition {
    pub relative_position_ned_m: Vector3d,
    pub valid_flags: ValidFlags,
}
impl NedRelativePosition {
    pub const DESCRIPTOR: u8 = 0x42;
    pub const LEN: usize = Vector3d::LEN + 2; // 26

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::LEN, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let mut b = bytes;
        Ok(Self {
            relative_position_ned_m: Vector3d::read_from(&mut b, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?,
            valid_flags: ValidFlags(b.read_u16()),
        })
    }
}

/// (0x82,0x43) GNSS Position Aiding Status
#[derive(Debug, Copy, Clone)]
pub struct GnssPositionAidingStatus {
    pub receiver_id: u8,
    pub time_of_week_s: f32,
    pub status: AidingStatusFlags,
    pub reserved: [u8; 8],
}
impl GnssPositionAidingStatus {
    pub const DESCRIPTOR: u8 = 0x43;
    pub const LEN: usize = 1 + 4 + 2 + 8; // 15

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::LEN, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let mut b = bytes;
        Ok(Self {
            receiver_id: b.read_u8(),
            time_of_week_s: b.read_f32(),
            status: AidingStatusFlags(b.read_u16()),
            reserved: b.read_bytes::<8>(),
        })
    }
}

/// (0x82,0x46) Aiding Measurement Summary
#[derive(Debug, Copy, Clone)]
pub struct AidingMeasurementSummary {
    pub time_of_week_s: f32,
    pub source: u8,
    pub aiding_type: u8,
    pub indicator: AidingIndicatorFlags,
}
impl AidingMeasurementSummary {
    pub const DESCRIPTOR: u8 = 0x46;
    pub const LEN: usize = 4 + 1 + 1 + 1; // 7

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::LEN, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let mut b = bytes;
        Ok(Self {
            time_of_week_s: b.read_f32(),
            source: b.read_u8(),
            aiding_type: b.read_u8(),
            indicator: AidingIndicatorFlags(b.read_u8()),
        })
    }
}

/// (0x82,0x47) Odometer Scale Factor Error
#[derive(Debug, Copy, Clone)]
pub struct OdometerScaleFactorError {
    pub scale_factor_error: f32,
    pub valid_flags: ValidFlags,
}
impl OdometerScaleFactorError {
    pub const DESCRIPTOR: u8 = 0x47;
    pub const LEN: usize = 4 + 2; // 6

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::LEN, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let mut b = bytes;
        Ok(Self {
            scale_factor_error: b.read_f32(),
            valid_flags: ValidFlags(b.read_u16()),
        })
    }
}

/// (0x82,0x48) Odometer Scale Factor Error Uncertainty
#[derive(Debug, Copy, Clone)]
pub struct OdometerScaleFactorErrorUncertainty {
    pub scale_factor_error_uncertainty: f32,
    pub valid_flags: ValidFlags,
}
impl OdometerScaleFactorErrorUncertainty {
    pub const DESCRIPTOR: u8 = 0x48;
    pub const LEN: usize = 4 + 2; // 6

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::LEN, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let mut b = bytes;
        Ok(Self {
            scale_factor_error_uncertainty: b.read_f32(),
            valid_flags: ValidFlags(b.read_u16()),
        })
    }
}

/// (0x82,0x49) GNSS Dual Antenna Status
#[derive(Debug, Copy, Clone)]
pub struct GnssDualAntennaStatus {
    pub time_of_week_s: f32,
    pub heading_rad: f32,
    pub heading_unc_rad: f32,
    pub fix_type: DualAntennaFixType,
    pub status_flags: GnssDualAntennaStatusFlags,
    pub valid_flags: ValidFlags,
}
impl GnssDualAntennaStatus {
    pub const DESCRIPTOR: u8 = 0x49;
    pub const LEN: usize = 4 + 4 + 4 + 1 + 2 + 2; // 17

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::LEN, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let mut b = bytes;
        Ok(Self {
            time_of_week_s: b.read_f32(),
            heading_rad: b.read_f32(),
            heading_unc_rad: b.read_f32(),
            fix_type: DualAntennaFixType::from_u8(b.read_u8()),
            status_flags: GnssDualAntennaStatusFlags(b.read_u16()),
            valid_flags: ValidFlags(b.read_u16()),
        })
    }
}

/// (0x82,0x50) Aiding Frame Configuration Error
#[derive(Debug, Copy, Clone)]
pub struct AidingFrameConfigurationError {
    pub frame_id: u8,
    pub translation_m: Vector3f,
    pub attitude: Quatf,
}
impl AidingFrameConfigurationError {
    pub const DESCRIPTOR: u8 = 0x50;
    pub const LEN: usize = 1 + Vector3f::LEN + Quatf::LEN; // 29

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::LEN, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let mut b = bytes;
        Ok(Self {
            frame_id: b.read_u8(),
            translation_m: Vector3f::read_from(&mut b, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?,
            attitude: Quatf::read_from(&mut b, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?,
        })
    }
}

/// (0x82,0x51) Aiding Frame Configuration Error Uncertainty
#[derive(Debug, Copy, Clone)]
pub struct AidingFrameConfigurationErrorUncertainty {
    pub frame_id: u8,
    pub translation_unc_m: Vector3f,
    pub attitude_unc_rad: Vector3f,
}
impl AidingFrameConfigurationErrorUncertainty {
    pub const DESCRIPTOR: u8 = 0x51;
    pub const LEN: usize = 1 + Vector3f::LEN + Vector3f::LEN; // 25

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::LEN, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let mut b = bytes;
        Ok(Self {
            frame_id: b.read_u8(),
            translation_unc_m: Vector3f::read_from(&mut b, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?,
            attitude_unc_rad: Vector3f::read_from(&mut b, (FILTER_DESCRIPTOR_SET, Self::DESCRIPTOR))?,
        })
    }
}

