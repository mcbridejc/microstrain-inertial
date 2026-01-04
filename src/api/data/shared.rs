//! Shared descriptor set (0xFF) data fields.

use super::{Error, ReadBuf};

pub const SHARED_DESCRIPTOR_SET: u8 = 0xFF;

/// A parsed Shared (0xFF) data field.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SharedField {
    EventSource(EventSource),               // 0xD0
    Ticks(Ticks),                           // 0xD1
    DeltaTicks(DeltaTicks),                 // 0xD2
    GpsTimestamp(GpsTimestamp),             // 0xD3
    DeltaTime(DeltaTime),                   // 0xD4
    ReferenceTimestamp(ReferenceTimestamp), // 0xD5
    ReferenceTimeDelta(ReferenceTimeDelta), // 0xD6
    ExternalTimestamp(ExternalTimestamp),   // 0xD7
    ExternalTimeDelta(ExternalTimeDelta),   // 0xD8
}

impl SharedField {
    /// Parse a single Shared (0xFF) field given the *field descriptor* and its raw field payload bytes.
    pub fn parse(descriptor: u8, bytes: &[u8]) -> Result<Self, Error> {
        match descriptor {
            EventSource::DESCRIPTOR => Ok(Self::EventSource(EventSource::from_bytes(bytes)?)),
            Ticks::DESCRIPTOR => Ok(Self::Ticks(Ticks::from_bytes(bytes)?)),
            DeltaTicks::DESCRIPTOR => Ok(Self::DeltaTicks(DeltaTicks::from_bytes(bytes)?)),
            GpsTimestamp::DESCRIPTOR => Ok(Self::GpsTimestamp(GpsTimestamp::from_bytes(bytes)?)),
            DeltaTime::DESCRIPTOR => Ok(Self::DeltaTime(DeltaTime::from_bytes(bytes)?)),
            ReferenceTimestamp::DESCRIPTOR => Ok(Self::ReferenceTimestamp(
                ReferenceTimestamp::from_bytes(bytes)?,
            )),
            ReferenceTimeDelta::DESCRIPTOR => Ok(Self::ReferenceTimeDelta(
                ReferenceTimeDelta::from_bytes(bytes)?,
            )),
            ExternalTimestamp::DESCRIPTOR => Ok(Self::ExternalTimestamp(
                ExternalTimestamp::from_bytes(bytes)?,
            )),
            ExternalTimeDelta::DESCRIPTOR => Ok(Self::ExternalTimeDelta(
                ExternalTimeDelta::from_bytes(bytes)?,
            )),
            other => Err(Error::UnknownField {
                descriptor_set: SHARED_DESCRIPTOR_SET,
                descriptor: other,
            }),
        }
    }
}

// -------------------------
// Field structs
// -------------------------

/// (0xFF, 0xD0) Event Source
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct EventSource {
    pub trigger_id: u8,
}
impl EventSource {
    pub const DESCRIPTOR: u8 = 0xD0;
    pub const LEN: usize = 1;

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::DESCRIPTOR, Self::LEN)?;
        let mut b = bytes;
        Ok(Self {
            trigger_id: b.read_u8(),
        })
    }
}

/// (0xFF, 0xD1) Ticks
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Ticks {
    pub ticks: u32,
}
impl Ticks {
    pub const DESCRIPTOR: u8 = 0xD1;
    pub const LEN: usize = 4;

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::DESCRIPTOR, Self::LEN)?;
        let mut b = bytes;
        Ok(Self {
            ticks: b.read_u32(),
        })
    }
}

/// (0xFF, 0xD2) Delta Ticks
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct DeltaTicks {
    pub delta_ticks: u32,
}
impl DeltaTicks {
    pub const DESCRIPTOR: u8 = 0xD2;
    pub const LEN: usize = 4;

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::DESCRIPTOR, Self::LEN)?;
        let mut b = bytes;
        Ok(Self {
            delta_ticks: b.read_u32(),
        })
    }
}

/// (0xFF, 0xD3) GPS Timestamp
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct GpsTimestamp {
    /// GPS Time of Week [seconds]
    pub tow_s: f64,
    /// GPS Week Number [weeks since 1980-01-06]
    pub week_number: u16,
    /// Valid flags for timestamp components
    pub valid_flags: GpsTimestampValidFlags,
}
impl GpsTimestamp {
    pub const DESCRIPTOR: u8 = 0xD3;
    pub const LEN: usize = 8 + 2 + 2; // 12

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::DESCRIPTOR, Self::LEN)?;
        let mut b = bytes;
        Ok(Self {
            tow_s: b.read_f64(),
            week_number: b.read_u16(),
            valid_flags: GpsTimestampValidFlags(b.read_u16()),
        })
    }
}

/// Bitflags for (0xFF,0xD3) GPS Timestamp valid flags.
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

/// (0xFF, 0xD4) Delta Time
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct DeltaTime {
    /// Time delta [seconds]
    pub dt_s: f64,
}
impl DeltaTime {
    pub const DESCRIPTOR: u8 = 0xD4;
    pub const LEN: usize = 8;

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::DESCRIPTOR, Self::LEN)?;
        let mut b = bytes;
        Ok(Self { dt_s: b.read_f64() })
    }
}

/// (0xFF, 0xD5) Reference Timestamp
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct ReferenceTimestamp {
    pub ticks: u64,
}
impl ReferenceTimestamp {
    pub const DESCRIPTOR: u8 = 0xD5;
    pub const LEN: usize = 8;

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::DESCRIPTOR, Self::LEN)?;
        let mut b = bytes;
        Ok(Self {
            ticks: b.read_u64(),
        })
    }
}

/// (0xFF, 0xD6) Reference Time Delta
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct ReferenceTimeDelta {
    /// Time delta [seconds]
    pub dt_s: f64,
}
impl ReferenceTimeDelta {
    pub const DESCRIPTOR: u8 = 0xD6;
    pub const LEN: usize = 8;

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::DESCRIPTOR, Self::LEN)?;
        let mut b = bytes;
        Ok(Self { dt_s: b.read_f64() })
    }
}

/// (0xFF, 0xD7) External Timestamp
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct ExternalTimestamp {
    /// Timestamp (often nanoseconds, device/config dependent)
    pub time: u64,
    pub valid_flags: ExternalTimestampValidFlags,
}
impl ExternalTimestamp {
    pub const DESCRIPTOR: u8 = 0xD7;
    pub const LEN: usize = 8 + 2; // 10

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::DESCRIPTOR, Self::LEN)?;
        let mut b = bytes;
        Ok(Self {
            time: b.read_u64(),
            valid_flags: ExternalTimestampValidFlags(b.read_u16()),
        })
    }
}

/// Bitflags for (0xFF,0xD7) External Timestamp valid flags.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct ExternalTimestampValidFlags(pub u16);
impl ExternalTimestampValidFlags {
    pub const NONE: Self = Self(0x0000);
    pub const TIME_VALID: Self = Self(0x0001);

    #[inline]
    pub fn contains(self, mask: Self) -> bool {
        (self.0 & mask.0) == mask.0
    }
}

/// (0xFF, 0xD8) External Time Delta
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct ExternalTimeDelta {
    /// Signed delta (often nanoseconds, device/config dependent)
    pub dt: i64,
    pub valid_flags: ExternalTimeDeltaValidFlags,
}
impl ExternalTimeDelta {
    pub const DESCRIPTOR: u8 = 0xD8;
    pub const LEN: usize = 8 + 2; // 10

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
        ensure_len(&bytes, Self::DESCRIPTOR, Self::LEN)?;
        let mut b = bytes;
        Ok(Self {
            dt: b.read_i64(),
            valid_flags: ExternalTimeDeltaValidFlags(b.read_u16()),
        })
    }
}

/// Bitflags for (0xFF,0xD8) External Time Delta valid flags.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct ExternalTimeDeltaValidFlags(pub u16);
impl ExternalTimeDeltaValidFlags {
    pub const NONE: Self = Self(0x0000);
    pub const DT_VALID: Self = Self(0x0001);

    #[inline]
    pub fn contains(self, mask: Self) -> bool {
        (self.0 & mask.0) == mask.0
    }
}

// -------------------------
// helpers
// -------------------------

#[inline]
fn ensure_len(buf: &[u8], descriptor: u8, need: usize) -> Result<(), Error> {
    if buf.len() < need {
        return Err(Error::LenTooShort {
            descriptor_set: SHARED_DESCRIPTOR_SET,
            descriptor,
            need,
            got: buf.len(),
        });
    }
    Ok(())
}
