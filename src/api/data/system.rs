//! System descriptor set (0xA0) data fields.
#![allow(missing_docs)]

use crate::api::data::shared::SharedField;
use crate::api::fields::{FieldIter, FieldParse};

use super::{ParseError, ReadBuf, ensure_len};

/// Descriptor set for system data packets
pub const SYSTEM_DESCRIPTOR_SET: u8 = 0xA0;

/// A wrapper for System data packets
pub struct SystemPacket<'a> {
    payload: &'a [u8],
}

impl<'a> SystemPacket<'a> {
    /// Create a new System packet from a payload
    ///
    /// It is *assumed* that the payload comes from a system data packet, and contains all fields
    /// belonging to the system descriptor set
    pub fn new(payload: &'a [u8]) -> Self {
        Self { payload }
    }

    /// Get an iterator for fields contained in the packet
    pub fn fields(&self) -> FieldIter<'a, SystemField> {
        FieldIter::new(self.payload)
    }
}

/// A parsed System (0xA0) data field.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SystemField {
    BuiltInTest(BuiltInTest),       // 0x01
    TimeSyncStatus(TimeSyncStatus), // 0x02
    GpioState(GpioState),           // 0x03
    Shared(SharedField),
    /// Any unrecognized field descriptor for set 0xA0.
    Unknown {
        descriptor: u8,
    },
}

impl FieldParse for SystemField {
    const DESCRIPTOR_SET: u8 = SYSTEM_DESCRIPTOR_SET;

    fn parse(descriptor: u8, bytes: &[u8]) -> Result<Self, ParseError> {
        Ok(match descriptor {
            BuiltInTest::DESCRIPTOR => Self::BuiltInTest(BuiltInTest::from_bytes(bytes)?),
            TimeSyncStatus::DESCRIPTOR => Self::TimeSyncStatus(TimeSyncStatus::from_bytes(bytes)?),
            GpioState::DESCRIPTOR => Self::GpioState(GpioState::from_bytes(bytes)?),
            other => {
                // Try to parse it as shared, and if that fails, it is an unknown field.
                match SharedField::parse(descriptor, bytes) {
                    Ok(field) => Self::Shared(field),
                    Err(_) => Self::Unknown { descriptor: other },
                }
            }
        })
    }
}

/// (0xA0, 0x01) Built In Test
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct BuiltInTest {
    /// Device-specific BIT result bytes (128 bits), least-significant-byte first.
    pub result: [u8; 16],
}
impl BuiltInTest {
    pub const DESCRIPTOR: u8 = 0x01;
    pub const LEN: usize = 16;

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, ParseError> {
        let mut b = bytes;
        ensure_len(&b, Self::LEN, (SYSTEM_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        Ok(Self {
            result: b.read_bytes::<16>(),
        })
    }
}

/// (0xA0, 0x02) Time Sync Status
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct TimeSyncStatus {
    /// True when sync with the PPS signal is currently valid.
    pub time_sync: bool,
    /// Elapsed time in seconds since last PPS was received (max 255).
    pub last_pps_rcvd_s: u8,
}
impl TimeSyncStatus {
    pub const DESCRIPTOR: u8 = 0x02;
    pub const LEN: usize = 2;

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, ParseError> {
        let mut b = bytes;
        ensure_len(&b, Self::LEN, (SYSTEM_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        Ok(Self {
            time_sync: b.read_u8() != 0,
            last_pps_rcvd_s: b.read_u8(),
        })
    }
}

/// (0xA0, 0x03) GPIO State
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct GpioState {
    /// GPIO state bits where bit0..bit3 correspond to pin1..pin4.
    pub states: GpioStateFlags,
}
impl GpioState {
    pub const DESCRIPTOR: u8 = 0x03;
    pub const LEN: usize = 1;

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, ParseError> {
        let mut b = bytes;
        ensure_len(&b, Self::LEN, (SYSTEM_DESCRIPTOR_SET, Self::DESCRIPTOR))?;
        Ok(Self {
            states: GpioStateFlags(b.read_u8()),
        })
    }
}

/// Bitflags for (0xA0,0x03) GPIO State.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct GpioStateFlags(pub u8);
impl GpioStateFlags {
    pub const NONE: Self = Self(0x00);
    pub const PIN1: Self = Self(0x01);
    pub const PIN2: Self = Self(0x02);
    pub const PIN3: Self = Self(0x04);
    pub const PIN4: Self = Self(0x08);

    pub const ALL: Self = Self(0x0F);

    #[inline]
    pub fn contains(self, mask: Self) -> bool {
        (self.0 & mask.0) == mask.0
    }
}
