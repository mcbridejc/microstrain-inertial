//! Sensor descriptor set (0x80) data fields.

use crate::api::data::{Matrix3f, Quatf, Vector3f};

use super::{Error, ReadBuf, ensure_len};

pub const SENSOR_DESCRIPTOR_SET: u8 = 0x80;

pub struct SensorPacket<'a> {
    payload: &'a [u8],
}

impl<'a> SensorPacket<'a> {
    pub fn new(payload: &'a [u8]) -> Self {
        Self { payload }
    }

    pub fn fields(&self) -> SensorFieldIter<'a> {
        SensorFieldIter {
            remaining: self.payload,
        }
    }
}

pub struct SensorFieldIter<'a> {
    remaining: &'a [u8],
}

impl<'a> Iterator for SensorFieldIter<'a> {
    type Item = Result<SensorField, Error>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.remaining.is_empty() {
            return None;
        }

        if self.remaining.len() < 2 {
            let err = Error::LenTooShort {
                descriptor_set: SENSOR_DESCRIPTOR_SET,
                descriptor: 0,
                need: 2,
                got: self.remaining.len(),
            };
            self.remaining = &[];
            return Some(Err(err));
        }

        let mut buf = self.remaining;
        let field_len = buf.read_u8() as usize;
        let descriptor = buf.read_u8();

        if buf.len() < field_len {
            let err = Error::LenTooShort {
                descriptor_set: SENSOR_DESCRIPTOR_SET,
                descriptor,
                need: field_len,
                got: buf.len(),
            };
            self.remaining = &[];
            return Some(Err(err));
        }

        let (payload, rest) = buf.split_at(field_len);
        self.remaining = rest;
        Some(SensorField::parse(descriptor, payload))
    }
}

/// A parsed Sensor (0x80) data field.
#[derive(Debug, Clone, Copy)]
pub enum SensorField {
    RawAccel(RawAccel),                           // 0x01
    RawGyro(RawGyro),                             // 0x02
    RawMag(RawMag),                               // 0x03
    ScaledAccel(ScaledAccel),                     // 0x04
    ScaledGyro(ScaledGyro),                       // 0x05
    ScaledMag(ScaledMag),                         // 0x06
    DeltaTheta(DeltaTheta),                       // 0x07
    DeltaVelocity(DeltaVelocity),                 // 0x08
    CompOrientationMatrix(CompOrientationMatrix), // 0x09
    CompQuaternion(CompQuaternion),               // 0x0A
    CompEulerAngles(CompEulerAngles),             // 0x0C

    // Deprecated (still seen on some devices/firmwares)
    CompOrientationUpdateMatrix(CompOrientationUpdateMatrix), // 0x0B (deprecated)
    OrientationRawTemp(OrientationRawTemp),                   // 0x0D (deprecated on some)
    InternalTimestamp(InternalTimestamp),                     // 0x0E (deprecated on some)
    PpsTimestamp(PpsTimestamp),                               // 0x0F (deprecated on some)

    NorthVector(NorthVector),         // 0x10
    UpVector(UpVector),               // 0x11
    GpsTimestamp(GpsTimestamp),       // 0x12
    TemperatureAbs(TemperatureAbs),   // 0x14
    RawPressure(RawPressure),         // 0x16
    ScaledPressure(ScaledPressure),   // 0x17
    OverrangeStatus(OverrangeStatus), // 0x18
    OdometerData(OdometerData),       // 0x40

    /// Any unrecognized field descriptor for set 0x80.
    ///
    /// This keeps your parser forward-compatible with newer firmware / products.
    Unknown {
        descriptor: u8,
    },
}

impl SensorField {
    /// Parse a single Sensor (0x80) field given the *field descriptor* and its raw field payload bytes.
    pub fn parse(descriptor: u8, bytes: &[u8]) -> Result<Self, Error> {
        Ok(match descriptor {
            RawAccel::DESCRIPTOR => Self::RawAccel(RawAccel::from_bytes(bytes)?),
            RawGyro::DESCRIPTOR => Self::RawGyro(RawGyro::from_bytes(bytes)?),
            RawMag::DESCRIPTOR => Self::RawMag(RawMag::from_bytes(bytes)?),
            ScaledAccel::DESCRIPTOR => Self::ScaledAccel(ScaledAccel::from_bytes(bytes)?),
            ScaledGyro::DESCRIPTOR => Self::ScaledGyro(ScaledGyro::from_bytes(bytes)?),
            ScaledMag::DESCRIPTOR => Self::ScaledMag(ScaledMag::from_bytes(bytes)?),
            DeltaTheta::DESCRIPTOR => Self::DeltaTheta(DeltaTheta::from_bytes(bytes)?),
            DeltaVelocity::DESCRIPTOR => Self::DeltaVelocity(DeltaVelocity::from_bytes(bytes)?),
            CompOrientationMatrix::DESCRIPTOR => {
                Self::CompOrientationMatrix(CompOrientationMatrix::from_bytes(bytes)?)
            }
            CompQuaternion::DESCRIPTOR => Self::CompQuaternion(CompQuaternion::from_bytes(bytes)?),
            CompEulerAngles::DESCRIPTOR => {
                Self::CompEulerAngles(CompEulerAngles::from_bytes(bytes)?)
            }

            CompOrientationUpdateMatrix::DESCRIPTOR => {
                Self::CompOrientationUpdateMatrix(CompOrientationUpdateMatrix::from_bytes(bytes)?)
            }
            OrientationRawTemp::DESCRIPTOR => {
                Self::OrientationRawTemp(OrientationRawTemp::from_bytes(bytes)?)
            }
            InternalTimestamp::DESCRIPTOR => {
                Self::InternalTimestamp(InternalTimestamp::from_bytes(bytes)?)
            }
            PpsTimestamp::DESCRIPTOR => Self::PpsTimestamp(PpsTimestamp::from_bytes(bytes)?),

            NorthVector::DESCRIPTOR => Self::NorthVector(NorthVector::from_bytes(bytes)?),
            UpVector::DESCRIPTOR => Self::UpVector(UpVector::from_bytes(bytes)?),
            GpsTimestamp::DESCRIPTOR => Self::GpsTimestamp(GpsTimestamp::from_bytes(bytes)?),
            TemperatureAbs::DESCRIPTOR => Self::TemperatureAbs(TemperatureAbs::from_bytes(bytes)?),
            RawPressure::DESCRIPTOR => Self::RawPressure(RawPressure::from_bytes(bytes)?),
            ScaledPressure::DESCRIPTOR => Self::ScaledPressure(ScaledPressure::from_bytes(bytes)?),
            OverrangeStatus::DESCRIPTOR => {
                Self::OverrangeStatus(OverrangeStatus::from_bytes(bytes)?)
            }
            OdometerData::DESCRIPTOR => Self::OdometerData(OdometerData::from_bytes(bytes)?),

            other => Self::Unknown { descriptor: other },
        })
    }
}

// -------------------------
// Field structs
// -------------------------

/// (0x80, 0x01) Raw Accel
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct RawAccel {
    /// Native sensor counts (represented as float in the MicroStrain C API type `mip_vector3f`).
    pub raw_accel: Vector3f,
}
impl RawAccel {
    pub const DESCRIPTOR: u8 = 0x01;

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        let mut b = bytes;
        let raw_accel = Vector3f::read_from(&mut b, (SENSOR_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        Ok(Self { raw_accel })
    }
}

/// (0x80, 0x02) Raw Gyro
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct RawGyro {
    pub raw_gyro: Vector3f,
}
impl RawGyro {
    pub const DESCRIPTOR: u8 = 0x02;

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        let mut b = bytes;
        let raw_gyro = Vector3f::read_from(&mut b, (SENSOR_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        Ok(Self { raw_gyro })
    }
}

/// (0x80, 0x03) Raw Mag
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct RawMag {
    pub raw_mag: Vector3f,
}
impl RawMag {
    pub const DESCRIPTOR: u8 = 0x03;

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        let mut b = bytes;
        let raw_mag = Vector3f::read_from(&mut b, (SENSOR_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        Ok(Self { raw_mag })
    }
}

/// (0x80, 0x04) Scaled Accel (g)
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct ScaledAccel {
    pub scaled_accel: Vector3f,
}
impl ScaledAccel {
    pub const DESCRIPTOR: u8 = 0x04;

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        let mut b = bytes;
        let scaled_accel = Vector3f::read_from(&mut b, (SENSOR_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        Ok(Self { scaled_accel })
    }
}

/// (0x80, 0x05) Scaled Gyro (rad/s)
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct ScaledGyro {
    pub scaled_gyro: Vector3f,
}
impl ScaledGyro {
    pub const DESCRIPTOR: u8 = 0x05;

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        let mut b = bytes;
        let scaled_gyro = Vector3f::read_from(&mut b, (SENSOR_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        Ok(Self { scaled_gyro })
    }
}

/// (0x80, 0x06) Scaled Mag (Gauss)
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct ScaledMag {
    pub scaled_mag: Vector3f,
}
impl ScaledMag {
    pub const DESCRIPTOR: u8 = 0x06;

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        let mut b = bytes;
        let scaled_mag = Vector3f::read_from(&mut b, (SENSOR_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        Ok(Self { scaled_mag })
    }
}

/// (0x80, 0x07) Delta Theta (rad)
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct DeltaTheta {
    pub delta_theta: Vector3f,
}
impl DeltaTheta {
    pub const DESCRIPTOR: u8 = 0x07;

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        let mut b = bytes;
        let delta_theta = Vector3f::read_from(&mut b, (SENSOR_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        Ok(Self { delta_theta })
    }
}

/// (0x80, 0x08) Delta Velocity (g*sec)
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct DeltaVelocity {
    pub delta_velocity: Vector3f,
}
impl DeltaVelocity {
    pub const DESCRIPTOR: u8 = 0x08;

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        let mut b = bytes;
        let delta_velocity =
            Vector3f::read_from(&mut b, (SENSOR_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        Ok(Self { delta_velocity })
    }
}

/// (0x80, 0x09) Comp Orientation Matrix (row-major)
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct CompOrientationMatrix {
    pub m: Matrix3f,
}
impl CompOrientationMatrix {
    pub const DESCRIPTOR: u8 = 0x09;

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        let mut b = bytes;
        let m = Matrix3f::read_from(&mut b, (SENSOR_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        Ok(Self { m })
    }
}

/// (0x80, 0x0A) Comp Quaternion (w, x, y, z)
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct CompQuaternion {
    pub q: Quatf,
}
impl CompQuaternion {
    pub const DESCRIPTOR: u8 = 0x0A;

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        let mut b = bytes;
        let q = Quatf::read_from(&mut b, (SENSOR_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        Ok(Self { q })
    }
}

/// (0x80, 0x0B) Comp Orientation Update Matrix (deprecated) - float[9]
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct CompOrientationUpdateMatrix {
    pub m: Matrix3f,
}
impl CompOrientationUpdateMatrix {
    pub const DESCRIPTOR: u8 = 0x0B;

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        let mut b = bytes;
        let m = Matrix3f::read_from(&mut b, (SENSOR_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        Ok(Self { m })
    }
}

/// (0x80, 0x0C) Comp Euler Angles (roll, pitch, yaw) in radians
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct CompEulerAngles {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}
impl CompEulerAngles {
    pub const DESCRIPTOR: u8 = 0x0C;

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        let mut b = bytes;
        ensure_len(&b, 12, (SENSOR_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let roll = b.read_f32();
        let pitch = b.read_f32();
        let yaw = b.read_f32();
        Ok(Self { roll, pitch, yaw })
    }
}

/// (0x80, 0x0D) Orientation Raw Temp (deprecated) - u16[4]
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct OrientationRawTemp {
    pub raw_temp: [u16; 4],
}
impl OrientationRawTemp {
    pub const DESCRIPTOR: u8 = 0x0D;

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        let mut b = bytes;
        ensure_len(&b, 8, (SENSOR_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let raw_temp = [b.read_u16(), b.read_u16(), b.read_u16(), b.read_u16()];
        Ok(Self { raw_temp })
    }
}

/// (0x80, 0x0E) Internal Timestamp (deprecated) - u32 counts
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct InternalTimestamp {
    pub counts: u32,
}
impl InternalTimestamp {
    pub const DESCRIPTOR: u8 = 0x0E;

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        let mut b = bytes;
        ensure_len(&b, 4, (SENSOR_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        Ok(Self {
            counts: b.read_u32(),
        })
    }
}

/// (0x80, 0x0F) PPS Timestamp (deprecated) - u32 seconds + u32 useconds
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct PpsTimestamp {
    pub seconds: u32,
    pub useconds: u32,
}
impl PpsTimestamp {
    pub const DESCRIPTOR: u8 = 0x0F;

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        let mut b = bytes;
        ensure_len(&b, 8, (SENSOR_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let seconds = b.read_u32();
        let useconds = b.read_u32();
        Ok(Self { seconds, useconds })
    }
}

/// (0x80, 0x10) North Vector (Gauss)
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct NorthVector {
    pub north: Vector3f,
}
impl NorthVector {
    pub const DESCRIPTOR: u8 = 0x10;

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        let mut b = bytes;
        let north = Vector3f::read_from(&mut b, (SENSOR_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        Ok(Self { north })
    }
}

/// (0x80, 0x11) Up Vector (Gs)
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct UpVector {
    pub up: Vector3f,
}
impl UpVector {
    pub const DESCRIPTOR: u8 = 0x11;

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        let mut b = bytes;
        let up = Vector3f::read_from(&mut b, (SENSOR_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        Ok(Self { up })
    }
}

/// (0x80, 0x12) GPS Timestamp
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct GpsTimestamp {
    /// GPS time-of-week [seconds]
    pub tow: f64,
    /// GPS week number since 1980 [weeks]
    pub week_number: u16,
    pub valid_flags: GpsTimestampValidFlags,
}
impl GpsTimestamp {
    pub const DESCRIPTOR: u8 = 0x12;

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        let mut b = bytes;
        // f64 + u16 + u16 = 12
        ensure_len(&b, 12, (SENSOR_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let tow = b.read_f64();
        let week_number = b.read_u16();
        let valid_flags = GpsTimestampValidFlags(b.read_u16());
        Ok(Self {
            tow,
            week_number,
            valid_flags,
        })
    }
}

/// Valid flags for (0x80,0x12) GPS Timestamp.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct GpsTimestampValidFlags(pub u16);
impl GpsTimestampValidFlags {
    pub const NONE: Self = Self(0x0000);
    pub const PPS_VALID: Self = Self(0x0001);
    pub const TIME_REFRESH: Self = Self(0x0002);
    pub const TIME_INITIALIZED: Self = Self(0x0004);
    pub const TOW_VALID: Self = Self(0x0008);
    pub const WEEK_NUMBER_VALID: Self = Self(0x0010);
    pub const ALL: Self = Self(0x001F);

    #[inline]
    pub fn contains(self, mask: Self) -> bool {
        (self.0 & mask.0) == mask.0
    }
}

/// (0x80, 0x14) Temperature Abs (degC)
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct TemperatureAbs {
    pub min_temp: f32,
    pub max_temp: f32,
    pub mean_temp: f32,
}
impl TemperatureAbs {
    pub const DESCRIPTOR: u8 = 0x14;

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        let mut b = bytes;
        ensure_len(&b, 12, (SENSOR_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let min_temp = b.read_f32();
        let max_temp = b.read_f32();
        let mean_temp = b.read_f32();
        Ok(Self {
            min_temp,
            max_temp,
            mean_temp,
        })
    }
}

/// (0x80, 0x16) Raw Pressure
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct RawPressure {
    pub raw_pressure: f32,
}
impl RawPressure {
    pub const DESCRIPTOR: u8 = 0x16;

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        let mut b = bytes;
        ensure_len(&b, 4, (SENSOR_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        Ok(Self {
            raw_pressure: b.read_f32(),
        })
    }
}

/// (0x80, 0x17) Scaled Pressure (mBar)
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct ScaledPressure {
    pub scaled_pressure: f32,
}
impl ScaledPressure {
    pub const DESCRIPTOR: u8 = 0x17;

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        let mut b = bytes;
        ensure_len(&b, 4, (SENSOR_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        Ok(Self {
            scaled_pressure: b.read_f32(),
        })
    }
}

/// (0x80, 0x18) Overrange Status
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct OverrangeStatus {
    pub status: OverrangeStatusFlags,
}
impl OverrangeStatus {
    pub const DESCRIPTOR: u8 = 0x18;

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        let mut b = bytes;
        ensure_len(&b, 2, (SENSOR_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        Ok(Self {
            status: OverrangeStatusFlags(b.read_u16()),
        })
    }
}

/// Bitflags for (0x80,0x18) Overrange Status.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct OverrangeStatusFlags(pub u16);
impl OverrangeStatusFlags {
    pub const NONE: Self = Self(0x0000);

    pub const ACCEL_X: Self = Self(0x0001);
    pub const ACCEL_Y: Self = Self(0x0002);
    pub const ACCEL_Z: Self = Self(0x0004);

    pub const GYRO_X: Self = Self(0x0010);
    pub const GYRO_Y: Self = Self(0x0020);
    pub const GYRO_Z: Self = Self(0x0040);

    pub const MAG_X: Self = Self(0x0100);
    pub const MAG_Y: Self = Self(0x0200);
    pub const MAG_Z: Self = Self(0x0400);

    pub const PRESS: Self = Self(0x1000);

    pub const ALL: Self = Self(0x1777);

    #[inline]
    pub fn contains(self, mask: Self) -> bool {
        (self.0 & mask.0) == mask.0
    }
}

/// (0x80, 0x40) Odometer Data
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct OdometerData {
    /// Average speed over the time interval [m/s] (may be negative for quadrature encoders)
    pub speed: f32,
    /// Uncertainty of velocity [m/s]
    pub uncertainty: f32,
    /// Valid flags (bit0 indicates configured)
    pub valid_flags: u16,
}
impl OdometerData {
    pub const DESCRIPTOR: u8 = 0x40;

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        let mut b = bytes;
        ensure_len(&b, 10, (SENSOR_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        let speed = b.read_f32();
        let uncertainty = b.read_f32();
        let valid_flags = b.read_u16();
        Ok(Self {
            speed,
            uncertainty,
            valid_flags,
        })
    }
}

// use super::Error;

// const SENSOR_DESCRIPTOR_SET: u8 = 0x80;

// pub enum SensorField {
//     ScaledAccel(ScaledAccel),
//     // Other types go here
// }

// /// Scaled Accel message data
// pub struct ScaledAccel {
//     /// g
//     pub x: f32,
//     /// g
//     pub y: f32,
//     /// g
//     pub z: f32,
// }

// impl ScaledAccel {
//     const DESCRIPTOR: u8 = 0x04;

//     pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
//         if bytes.len() < 12 {
//             return Err(Error::LenTooShort {
//                 descriptor_set: SENSOR_DESCRIPTOR_SET,
//                 descriptor: Self::DESCRIPTOR,
//             });
//         }
//         let x = f32::from_be_bytes(bytes[0..4].try_into().unwrap());
//         let y = f32::from_be_bytes(bytes[4..8].try_into().unwrap());
//         let z = f32::from_be_bytes(bytes[8..12].try_into().unwrap());
//         Ok(Self { x, y, z })
//     }
// }
