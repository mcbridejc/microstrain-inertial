//! 3DM (0x0C) commands.

use crate::api::types::{Matrix3f, Quatf, Vector3f};
use crate::errors::ParseError;

use super::base::FunctionSelector;
use super::{CommandField, CommandResponseData, IMU_3DM_DESCRIPTOR_SET, SerializeError};

/// Marker trait for 3DM (0x0C) commands.
pub trait Imu3dmCommandField: CommandField {}

const MAX_DESCRIPTOR_RATES: usize = 83;
const MAX_U8_PAIRS: usize = 126;
const MAX_PARAM_BYTES: usize = 250;
const MAX_RANGE_ENTRIES: usize = 50;

#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub struct DescriptorRate {
    pub descriptor: u8,
    pub decimation: u16,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub struct SupportedEventInfo {
    pub event_type: u8,
    pub count: u8,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub struct TriggerStatusEntry {
    pub trigger_type: u8,
    pub status: u8,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub struct ActionStatusEntry {
    pub action_type: u8,
    pub trigger_id: u8,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct SensorRangeEntry {
    pub setting: u8,
    pub range: f32,
}

impl Default for SensorRangeEntry {
    fn default() -> Self {
        Self {
            setting: 0,
            range: 0.0,
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[repr(u8)]
pub enum FactoryStreamingAction {
    Overwrite = 0,
    Merge = 1,
    Add = 2,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[repr(u8)]
pub enum SupportedEventsQuery {
    TriggerTypes = 1,
    ActionTypes = 2,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[repr(u8)]
pub enum EventMode {
    Disabled = 0,
    Enabled = 1,
    Test = 2,
    TestPulse = 3,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[repr(u8)]
pub enum PpsSource {
    Disabled = 0,
    Receiver1 = 1,
    Receiver2 = 2,
    Gpio = 3,
    Generated = 4,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[repr(u8)]
pub enum SensorRangeType {
    All = 0,
    Accel = 1,
    Gyro = 2,
    Mag = 3,
    Press = 4,
}

fn len_too_short(descriptor: u8, need: usize, got: usize) -> ParseError {
    ParseError::LenTooShort {
        descriptor_set: IMU_3DM_DESCRIPTOR_SET,
        descriptor,
        need,
        got,
    }
}

fn parse_u16_be(data: &[u8]) -> u16 {
    u16::from_be_bytes([data[0], data[1]])
}

fn parse_u32_be(data: &[u8]) -> u32 {
    u32::from_be_bytes([data[0], data[1], data[2], data[3]])
}

fn parse_f32_be(data: &[u8]) -> f32 {
    f32::from_be_bytes([data[0], data[1], data[2], data[3]])
}

fn write_f32_be(value: f32, out: &mut [u8]) {
    out.copy_from_slice(&value.to_be_bytes());
}

fn write_vector3f(v: Vector3f, out: &mut [u8]) {
    write_f32_be(v.x, &mut out[0..4]);
    write_f32_be(v.y, &mut out[4..8]);
    write_f32_be(v.z, &mut out[8..12]);
}

fn parse_vector3f(data: &[u8]) -> Vector3f {
    Vector3f {
        x: parse_f32_be(&data[0..4]),
        y: parse_f32_be(&data[4..8]),
        z: parse_f32_be(&data[8..12]),
    }
}

fn write_quatf(q: Quatf, out: &mut [u8]) {
    write_f32_be(q.w, &mut out[0..4]);
    write_f32_be(q.x, &mut out[4..8]);
    write_f32_be(q.y, &mut out[8..12]);
    write_f32_be(q.z, &mut out[12..16]);
}

fn parse_quatf(data: &[u8]) -> Quatf {
    Quatf {
        w: parse_f32_be(&data[0..4]),
        x: parse_f32_be(&data[4..8]),
        y: parse_f32_be(&data[8..12]),
        z: parse_f32_be(&data[12..16]),
    }
}

fn write_matrix3f(m: Matrix3f, out: &mut [u8]) {
    for i in 0..9 {
        write_f32_be(m.data[i], &mut out[i * 4..i * 4 + 4]);
    }
}

fn parse_matrix3f(data: &[u8]) -> Matrix3f {
    let mut vals = [0f32; 9];
    for i in 0..9 {
        vals[i] = parse_f32_be(&data[i * 4..i * 4 + 4]);
    }
    Matrix3f { data: vals }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct PollData<'a> {
    pub descriptor_set: u8,
    pub suppress_ack: bool,
    pub descriptors: &'a [u8],
}

impl CommandField for PollData<'_> {
    type Response = ();

    fn descriptor_set(&self) -> u8 {
        IMU_3DM_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x0D
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        let need = 3 + self.descriptors.len();
        if buf.len() < need || need > u8::MAX as usize {
            return Err(SerializeError::OutOfSpace);
        }
        buf[0] = self.descriptor_set;
        buf[1] = self.suppress_ack as u8;
        buf[2] = self.descriptors.len() as u8;
        buf[3..need].copy_from_slice(self.descriptors);
        Ok(need as u8)
    }
}
impl Imu3dmCommandField for PollData<'_> {}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct GetDataBaseRate {
    pub descriptor_set: u8,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct GetDataBaseRateResponse {
    pub descriptor_set: u8,
    pub rate_hz: u16,
}

impl CommandResponseData for GetDataBaseRateResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() < 3 {
            return Err(len_too_short(0x0E, 3, data.len()));
        }
        Ok(Self {
            descriptor_set: data[0],
            rate_hz: parse_u16_be(&data[1..3]),
        })
    }
}

impl CommandField for GetDataBaseRate {
    type Response = GetDataBaseRateResponse;

    fn descriptor_set(&self) -> u8 {
        IMU_3DM_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x0E
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        if buf.is_empty() {
            return Err(SerializeError::OutOfSpace);
        }
        buf[0] = self.descriptor_set;
        Ok(1)
    }
}
impl Imu3dmCommandField for GetDataBaseRate {}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum MessageFormat<'a> {
    Write {
        descriptor_set: u8,
        descriptors: &'a [DescriptorRate],
    },
    Read {
        descriptor_set: u8,
    },
    Save {
        descriptor_set: u8,
    },
    Load {
        descriptor_set: u8,
    },
    Default {
        descriptor_set: u8,
    },
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct MessageFormatResponse {
    pub descriptor_set: u8,
    pub descriptors: [DescriptorRate; MAX_DESCRIPTOR_RATES],
    pub len: usize,
}

impl CommandResponseData for MessageFormatResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() < 2 {
            return Err(len_too_short(0x0F, 2, data.len()));
        }
        let count = data[1] as usize;
        let need = 2 + count * 3;
        if data.len() < need {
            return Err(len_too_short(0x0F, need, data.len()));
        }
        if count > MAX_DESCRIPTOR_RATES {
            return Err(ParseError::UnknownField {
                descriptor_set: IMU_3DM_DESCRIPTOR_SET,
                descriptor: 0x0F,
            });
        }

        let mut descriptors = [DescriptorRate::default(); MAX_DESCRIPTOR_RATES];
        for (i, slot) in descriptors.iter_mut().enumerate().take(count) {
            let b = 2 + i * 3;
            *slot = DescriptorRate {
                descriptor: data[b],
                decimation: parse_u16_be(&data[b + 1..b + 3]),
            };
        }

        Ok(Self {
            descriptor_set: data[0],
            descriptors,
            len: count,
        })
    }
}

impl CommandField for MessageFormat<'_> {
    type Response = MessageFormatResponse;

    fn descriptor_set(&self) -> u8 {
        IMU_3DM_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x0F
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        match self {
            Self::Write {
                descriptor_set,
                descriptors,
            } => {
                let need = 3 + descriptors.len() * 3;
                if buf.len() < need || descriptors.len() > u8::MAX as usize {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Write as u8;
                buf[1] = *descriptor_set;
                buf[2] = descriptors.len() as u8;
                for (i, entry) in descriptors.iter().enumerate() {
                    let b = 3 + i * 3;
                    buf[b] = entry.descriptor;
                    buf[b + 1..b + 3].copy_from_slice(&entry.decimation.to_be_bytes());
                }
                Ok(need as u8)
            }
            Self::Read { descriptor_set } => {
                if buf.len() < 2 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Read as u8;
                buf[1] = *descriptor_set;
                Ok(2)
            }
            Self::Save { descriptor_set } => {
                if buf.len() < 2 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Save as u8;
                buf[1] = *descriptor_set;
                Ok(2)
            }
            Self::Load { descriptor_set } => {
                if buf.len() < 2 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Load as u8;
                buf[1] = *descriptor_set;
                Ok(2)
            }
            Self::Default { descriptor_set } => {
                if buf.len() < 2 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Default as u8;
                buf[1] = *descriptor_set;
                Ok(2)
            }
        }
    }
}
impl Imu3dmCommandField for MessageFormat<'_> {}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct FactoryStreaming {
    pub action: FactoryStreamingAction,
    pub reserved: u8,
}

impl CommandField for FactoryStreaming {
    type Response = ();

    fn descriptor_set(&self) -> u8 {
        IMU_3DM_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x10
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        if buf.len() < 2 {
            return Err(SerializeError::OutOfSpace);
        }
        buf[0] = self.action as u8;
        buf[1] = self.reserved;
        Ok(2)
    }
}
impl Imu3dmCommandField for FactoryStreaming {}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum DataStreamControl {
    Write { descriptor_set: u8, enable: bool },
    Read { descriptor_set: u8 },
    Save { descriptor_set: u8 },
    Load { descriptor_set: u8 },
    Default { descriptor_set: u8 },
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct DataStreamControlResponse {
    pub descriptor_set: u8,
    pub enabled: bool,
}

impl CommandResponseData for DataStreamControlResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() < 2 {
            return Err(len_too_short(0x11, 2, data.len()));
        }
        Ok(Self {
            descriptor_set: data[0],
            enabled: data[1] != 0,
        })
    }
}

impl CommandField for DataStreamControl {
    type Response = DataStreamControlResponse;

    fn descriptor_set(&self) -> u8 {
        IMU_3DM_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x11
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        match self {
            Self::Write {
                descriptor_set,
                enable,
            } => {
                if buf.len() < 3 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Write as u8;
                buf[1] = *descriptor_set;
                buf[2] = *enable as u8;
                Ok(3)
            }
            Self::Read { descriptor_set } => {
                if buf.len() < 2 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Read as u8;
                buf[1] = *descriptor_set;
                Ok(2)
            }
            Self::Save { descriptor_set } => {
                if buf.len() < 2 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Save as u8;
                buf[1] = *descriptor_set;
                Ok(2)
            }
            Self::Load { descriptor_set } => {
                if buf.len() < 2 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Load as u8;
                buf[1] = *descriptor_set;
                Ok(2)
            }
            Self::Default { descriptor_set } => {
                if buf.len() < 2 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Default as u8;
                buf[1] = *descriptor_set;
                Ok(2)
            }
        }
    }
}
impl Imu3dmCommandField for DataStreamControl {}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum PpsSourceControl {
    Write { source: PpsSource },
    Read,
    Save,
    Load,
    Default,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct PpsSourceControlResponse {
    pub source: u8,
}

impl CommandResponseData for PpsSourceControlResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        if data.is_empty() {
            return Err(len_too_short(0x28, 1, data.len()));
        }
        Ok(Self { source: data[0] })
    }
}

impl CommandField for PpsSourceControl {
    type Response = PpsSourceControlResponse;

    fn descriptor_set(&self) -> u8 {
        IMU_3DM_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x28
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        match self {
            Self::Write { source } => {
                if buf.len() < 2 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Write as u8;
                buf[1] = *source as u8;
                Ok(2)
            }
            Self::Read => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Read as u8;
                Ok(1)
            }
            Self::Save => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Save as u8;
                Ok(1)
            }
            Self::Load => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Load as u8;
                Ok(1)
            }
            Self::Default => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Default as u8;
                Ok(1)
            }
        }
    }
}
impl Imu3dmCommandField for PpsSourceControl {}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct GetSupportedEvents {
    pub query: SupportedEventsQuery,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct GetSupportedEventsResponse {
    pub query: u8,
    pub max_instances: u8,
    pub entries: [SupportedEventInfo; MAX_U8_PAIRS],
    pub len: usize,
}

impl CommandResponseData for GetSupportedEventsResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() < 3 {
            return Err(len_too_short(0x2A, 3, data.len()));
        }
        let n = data[2] as usize;
        let need = 3 + n * 2;
        if data.len() < need {
            return Err(len_too_short(0x2A, need, data.len()));
        }
        if n > MAX_U8_PAIRS {
            return Err(ParseError::UnknownField {
                descriptor_set: IMU_3DM_DESCRIPTOR_SET,
                descriptor: 0x2A,
            });
        }

        let mut entries = [SupportedEventInfo::default(); MAX_U8_PAIRS];
        for (i, slot) in entries.iter_mut().enumerate().take(n) {
            let b = 3 + i * 2;
            *slot = SupportedEventInfo {
                event_type: data[b],
                count: data[b + 1],
            };
        }

        Ok(Self {
            query: data[0],
            max_instances: data[1],
            entries,
            len: n,
        })
    }
}

impl CommandField for GetSupportedEvents {
    type Response = GetSupportedEventsResponse;

    fn descriptor_set(&self) -> u8 {
        IMU_3DM_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x2A
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        if buf.is_empty() {
            return Err(SerializeError::OutOfSpace);
        }
        buf[0] = self.query as u8;
        Ok(1)
    }
}
impl Imu3dmCommandField for GetSupportedEvents {}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum EventControl {
    Write { instance: u8, mode: EventMode },
    Read { instance: u8 },
    Save { instance: u8 },
    Load { instance: u8 },
    Default { instance: u8 },
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct EventControlResponse {
    pub instance: u8,
    pub mode: u8,
}

impl CommandResponseData for EventControlResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() < 2 {
            return Err(len_too_short(0x2B, 2, data.len()));
        }
        Ok(Self {
            instance: data[0],
            mode: data[1],
        })
    }
}

impl CommandField for EventControl {
    type Response = EventControlResponse;

    fn descriptor_set(&self) -> u8 {
        IMU_3DM_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x2B
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        match self {
            Self::Write { instance, mode } => {
                if buf.len() < 3 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Write as u8;
                buf[1] = *instance;
                buf[2] = *mode as u8;
                Ok(3)
            }
            Self::Read { instance } => {
                if buf.len() < 2 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Read as u8;
                buf[1] = *instance;
                Ok(2)
            }
            Self::Save { instance } => {
                if buf.len() < 2 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Save as u8;
                buf[1] = *instance;
                Ok(2)
            }
            Self::Load { instance } => {
                if buf.len() < 2 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Load as u8;
                buf[1] = *instance;
                Ok(2)
            }
            Self::Default { instance } => {
                if buf.len() < 2 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Default as u8;
                buf[1] = *instance;
                Ok(2)
            }
        }
    }
}
impl Imu3dmCommandField for EventControl {}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct GetEventTriggerStatus<'a> {
    pub requested_instances: &'a [u8],
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct GetEventTriggerStatusResponse {
    pub entries: [TriggerStatusEntry; MAX_U8_PAIRS],
    pub len: usize,
}

impl CommandResponseData for GetEventTriggerStatusResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        if data.is_empty() {
            return Err(len_too_short(0x2C, 1, data.len()));
        }
        let n = data[0] as usize;
        let need = 1 + n * 2;
        if data.len() < need {
            return Err(len_too_short(0x2C, need, data.len()));
        }
        if n > MAX_U8_PAIRS {
            return Err(ParseError::UnknownField {
                descriptor_set: IMU_3DM_DESCRIPTOR_SET,
                descriptor: 0x2C,
            });
        }
        let mut entries = [TriggerStatusEntry::default(); MAX_U8_PAIRS];
        for (i, slot) in entries.iter_mut().enumerate().take(n) {
            let b = 1 + i * 2;
            *slot = TriggerStatusEntry {
                trigger_type: data[b],
                status: data[b + 1],
            };
        }
        Ok(Self { entries, len: n })
    }
}

impl CommandField for GetEventTriggerStatus<'_> {
    type Response = GetEventTriggerStatusResponse;

    fn descriptor_set(&self) -> u8 {
        IMU_3DM_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x2C
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        let need = 1 + self.requested_instances.len();
        if buf.len() < need || self.requested_instances.len() > u8::MAX as usize {
            return Err(SerializeError::OutOfSpace);
        }
        buf[0] = self.requested_instances.len() as u8;
        buf[1..need].copy_from_slice(self.requested_instances);
        Ok(need as u8)
    }
}
impl Imu3dmCommandField for GetEventTriggerStatus<'_> {}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct GetEventActionStatus<'a> {
    pub requested_instances: &'a [u8],
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct GetEventActionStatusResponse {
    pub entries: [ActionStatusEntry; MAX_U8_PAIRS],
    pub len: usize,
}

impl CommandResponseData for GetEventActionStatusResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        if data.is_empty() {
            return Err(len_too_short(0x2D, 1, data.len()));
        }
        let n = data[0] as usize;
        let need = 1 + n * 2;
        if data.len() < need {
            return Err(len_too_short(0x2D, need, data.len()));
        }
        if n > MAX_U8_PAIRS {
            return Err(ParseError::UnknownField {
                descriptor_set: IMU_3DM_DESCRIPTOR_SET,
                descriptor: 0x2D,
            });
        }
        let mut entries = [ActionStatusEntry::default(); MAX_U8_PAIRS];
        for (i, slot) in entries.iter_mut().enumerate().take(n) {
            let b = 1 + i * 2;
            *slot = ActionStatusEntry {
                action_type: data[b],
                trigger_id: data[b + 1],
            };
        }
        Ok(Self { entries, len: n })
    }
}

impl CommandField for GetEventActionStatus<'_> {
    type Response = GetEventActionStatusResponse;

    fn descriptor_set(&self) -> u8 {
        IMU_3DM_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x2D
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        let need = 1 + self.requested_instances.len();
        if buf.len() < need || self.requested_instances.len() > u8::MAX as usize {
            return Err(SerializeError::OutOfSpace);
        }
        buf[0] = self.requested_instances.len() as u8;
        buf[1..need].copy_from_slice(self.requested_instances);
        Ok(need as u8)
    }
}
impl Imu3dmCommandField for GetEventActionStatus<'_> {}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum EventTriggerConfiguration<'a> {
    Write {
        instance: u8,
        trigger_type: u8,
        parameters: &'a [u8],
    },
    Read {
        instance: u8,
    },
    Save {
        instance: u8,
    },
    Load {
        instance: u8,
    },
    Default {
        instance: u8,
    },
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct EventTriggerConfigurationResponse {
    pub instance: u8,
    pub trigger_type: u8,
    pub parameters: [u8; MAX_PARAM_BYTES],
    pub len: usize,
}

impl CommandResponseData for EventTriggerConfigurationResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() < 2 {
            return Err(len_too_short(0x2E, 2, data.len()));
        }
        let p_len = data.len() - 2;
        if p_len > MAX_PARAM_BYTES {
            return Err(ParseError::UnknownField {
                descriptor_set: IMU_3DM_DESCRIPTOR_SET,
                descriptor: 0x2E,
            });
        }
        let mut parameters = [0u8; MAX_PARAM_BYTES];
        parameters[..p_len].copy_from_slice(&data[2..]);
        Ok(Self {
            instance: data[0],
            trigger_type: data[1],
            parameters,
            len: p_len,
        })
    }
}

impl CommandField for EventTriggerConfiguration<'_> {
    type Response = EventTriggerConfigurationResponse;

    fn descriptor_set(&self) -> u8 {
        IMU_3DM_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x2E
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        match self {
            Self::Write {
                instance,
                trigger_type,
                parameters,
            } => {
                let need = 3 + parameters.len();
                if buf.len() < need || need > u8::MAX as usize {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Write as u8;
                buf[1] = *instance;
                buf[2] = *trigger_type;
                buf[3..need].copy_from_slice(parameters);
                Ok(need as u8)
            }
            Self::Read { instance } => {
                if buf.len() < 2 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Read as u8;
                buf[1] = *instance;
                Ok(2)
            }
            Self::Save { instance } => {
                if buf.len() < 2 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Save as u8;
                buf[1] = *instance;
                Ok(2)
            }
            Self::Load { instance } => {
                if buf.len() < 2 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Load as u8;
                buf[1] = *instance;
                Ok(2)
            }
            Self::Default { instance } => {
                if buf.len() < 2 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Default as u8;
                buf[1] = *instance;
                Ok(2)
            }
        }
    }
}
impl Imu3dmCommandField for EventTriggerConfiguration<'_> {}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum EventActionConfiguration<'a> {
    Write {
        instance: u8,
        trigger_id: u8,
        action_type: u8,
        parameters: &'a [u8],
    },
    Read {
        instance: u8,
    },
    Save {
        instance: u8,
    },
    Load {
        instance: u8,
    },
    Default {
        instance: u8,
    },
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct EventActionConfigurationResponse {
    pub instance: u8,
    pub trigger_id: u8,
    pub action_type: u8,
    pub parameters: [u8; MAX_PARAM_BYTES],
    pub len: usize,
}

impl CommandResponseData for EventActionConfigurationResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() < 3 {
            return Err(len_too_short(0x2F, 3, data.len()));
        }
        let p_len = data.len() - 3;
        if p_len > MAX_PARAM_BYTES {
            return Err(ParseError::UnknownField {
                descriptor_set: IMU_3DM_DESCRIPTOR_SET,
                descriptor: 0x2F,
            });
        }
        let mut parameters = [0u8; MAX_PARAM_BYTES];
        parameters[..p_len].copy_from_slice(&data[3..]);
        Ok(Self {
            instance: data[0],
            trigger_id: data[1],
            action_type: data[2],
            parameters,
            len: p_len,
        })
    }
}

impl CommandField for EventActionConfiguration<'_> {
    type Response = EventActionConfigurationResponse;

    fn descriptor_set(&self) -> u8 {
        IMU_3DM_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x2F
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        match self {
            Self::Write {
                instance,
                trigger_id,
                action_type,
                parameters,
            } => {
                let need = 4 + parameters.len();
                if buf.len() < need || need > u8::MAX as usize {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Write as u8;
                buf[1] = *instance;
                buf[2] = *trigger_id;
                buf[3] = *action_type;
                buf[4..need].copy_from_slice(parameters);
                Ok(need as u8)
            }
            Self::Read { instance } => {
                if buf.len() < 2 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Read as u8;
                buf[1] = *instance;
                Ok(2)
            }
            Self::Save { instance } => {
                if buf.len() < 2 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Save as u8;
                buf[1] = *instance;
                Ok(2)
            }
            Self::Load { instance } => {
                if buf.len() < 2 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Load as u8;
                buf[1] = *instance;
                Ok(2)
            }
            Self::Default { instance } => {
                if buf.len() < 2 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Default as u8;
                buf[1] = *instance;
                Ok(2)
            }
        }
    }
}
impl Imu3dmCommandField for EventActionConfiguration<'_> {}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum DeviceStartupSettings {
    Save,
    Load,
    Default,
}

impl CommandField for DeviceStartupSettings {
    type Response = ();

    fn descriptor_set(&self) -> u8 {
        IMU_3DM_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x30
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        if buf.is_empty() {
            return Err(SerializeError::OutOfSpace);
        }
        buf[0] = match self {
            Self::Save => FunctionSelector::Save as u8,
            Self::Load => FunctionSelector::Load as u8,
            Self::Default => FunctionSelector::Default as u8,
        };
        Ok(1)
    }
}
impl Imu3dmCommandField for DeviceStartupSettings {}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum SensorToVehicleFrameTransformationEuler {
    Write {
        roll_rad: f32,
        pitch_rad: f32,
        yaw_rad: f32,
    },
    Read,
    Save,
    Load,
    Default,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct SensorToVehicleFrameTransformationEulerResponse {
    pub roll_rad: f32,
    pub pitch_rad: f32,
    pub yaw_rad: f32,
}

impl CommandResponseData for SensorToVehicleFrameTransformationEulerResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() < 12 {
            return Err(len_too_short(0x31, 12, data.len()));
        }
        Ok(Self {
            roll_rad: parse_f32_be(&data[0..4]),
            pitch_rad: parse_f32_be(&data[4..8]),
            yaw_rad: parse_f32_be(&data[8..12]),
        })
    }
}

impl CommandField for SensorToVehicleFrameTransformationEuler {
    type Response = SensorToVehicleFrameTransformationEulerResponse;

    fn descriptor_set(&self) -> u8 {
        IMU_3DM_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x31
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        match self {
            Self::Write {
                roll_rad,
                pitch_rad,
                yaw_rad,
            } => {
                if buf.len() < 13 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Write as u8;
                write_f32_be(*roll_rad, &mut buf[1..5]);
                write_f32_be(*pitch_rad, &mut buf[5..9]);
                write_f32_be(*yaw_rad, &mut buf[9..13]);
                Ok(13)
            }
            Self::Read => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Read as u8;
                Ok(1)
            }
            Self::Save => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Save as u8;
                Ok(1)
            }
            Self::Load => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Load as u8;
                Ok(1)
            }
            Self::Default => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Default as u8;
                Ok(1)
            }
        }
    }
}
impl Imu3dmCommandField for SensorToVehicleFrameTransformationEuler {}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum SensorToVehicleFrameTransformationQuaternion {
    Write { q: Quatf },
    Read,
    Save,
    Load,
    Default,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct SensorToVehicleFrameTransformationQuaternionResponse {
    pub q: Quatf,
}

impl CommandResponseData for SensorToVehicleFrameTransformationQuaternionResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() < 16 {
            return Err(len_too_short(0x32, 16, data.len()));
        }
        Ok(Self {
            q: parse_quatf(data),
        })
    }
}

impl CommandField for SensorToVehicleFrameTransformationQuaternion {
    type Response = SensorToVehicleFrameTransformationQuaternionResponse;

    fn descriptor_set(&self) -> u8 {
        IMU_3DM_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x32
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        match self {
            Self::Write { q } => {
                if buf.len() < 17 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Write as u8;
                write_quatf(*q, &mut buf[1..17]);
                Ok(17)
            }
            Self::Read => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Read as u8;
                Ok(1)
            }
            Self::Save => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Save as u8;
                Ok(1)
            }
            Self::Load => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Load as u8;
                Ok(1)
            }
            Self::Default => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Default as u8;
                Ok(1)
            }
        }
    }
}
impl Imu3dmCommandField for SensorToVehicleFrameTransformationQuaternion {}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum SensorToVehicleFrameTransformationDirectionCosineMatrix {
    Write { dcm: Matrix3f },
    Read,
    Save,
    Load,
    Default,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct SensorToVehicleFrameTransformationDirectionCosineMatrixResponse {
    pub dcm: Matrix3f,
}

impl CommandResponseData for SensorToVehicleFrameTransformationDirectionCosineMatrixResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() < 36 {
            return Err(len_too_short(0x33, 36, data.len()));
        }
        Ok(Self {
            dcm: parse_matrix3f(data),
        })
    }
}

impl CommandField for SensorToVehicleFrameTransformationDirectionCosineMatrix {
    type Response = SensorToVehicleFrameTransformationDirectionCosineMatrixResponse;

    fn descriptor_set(&self) -> u8 {
        IMU_3DM_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x33
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        match self {
            Self::Write { dcm } => {
                if buf.len() < 37 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Write as u8;
                write_matrix3f(*dcm, &mut buf[1..37]);
                Ok(37)
            }
            Self::Read => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Read as u8;
                Ok(1)
            }
            Self::Save => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Save as u8;
                Ok(1)
            }
            Self::Load => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Load as u8;
                Ok(1)
            }
            Self::Default => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Default as u8;
                Ok(1)
            }
        }
    }
}
impl Imu3dmCommandField for SensorToVehicleFrameTransformationDirectionCosineMatrix {}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum AccelerometerBiasConfiguration {
    Write { bias_g: Vector3f },
    Read,
    Save,
    Load,
    Default,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct AccelerometerBiasConfigurationResponse {
    pub bias_g: Vector3f,
}

impl CommandResponseData for AccelerometerBiasConfigurationResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() < 12 {
            return Err(len_too_short(0x37, 12, data.len()));
        }
        Ok(Self {
            bias_g: parse_vector3f(data),
        })
    }
}

impl CommandField for AccelerometerBiasConfiguration {
    type Response = AccelerometerBiasConfigurationResponse;

    fn descriptor_set(&self) -> u8 {
        IMU_3DM_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x37
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        match self {
            Self::Write { bias_g } => {
                if buf.len() < 13 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Write as u8;
                write_vector3f(*bias_g, &mut buf[1..13]);
                Ok(13)
            }
            Self::Read => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Read as u8;
                Ok(1)
            }
            Self::Save => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Save as u8;
                Ok(1)
            }
            Self::Load => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Load as u8;
                Ok(1)
            }
            Self::Default => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Default as u8;
                Ok(1)
            }
        }
    }
}
impl Imu3dmCommandField for AccelerometerBiasConfiguration {}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum GyroscopeBiasConfiguration {
    Write { bias_rad_s: Vector3f },
    Read,
    Save,
    Load,
    Default,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct GyroscopeBiasConfigurationResponse {
    pub bias_rad_s: Vector3f,
}

impl CommandResponseData for GyroscopeBiasConfigurationResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() < 12 {
            return Err(len_too_short(0x38, 12, data.len()));
        }
        Ok(Self {
            bias_rad_s: parse_vector3f(data),
        })
    }
}

impl CommandField for GyroscopeBiasConfiguration {
    type Response = GyroscopeBiasConfigurationResponse;

    fn descriptor_set(&self) -> u8 {
        IMU_3DM_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x38
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        match self {
            Self::Write { bias_rad_s } => {
                if buf.len() < 13 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Write as u8;
                write_vector3f(*bias_rad_s, &mut buf[1..13]);
                Ok(13)
            }
            Self::Read => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Read as u8;
                Ok(1)
            }
            Self::Save => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Save as u8;
                Ok(1)
            }
            Self::Load => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Load as u8;
                Ok(1)
            }
            Self::Default => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Default as u8;
                Ok(1)
            }
        }
    }
}
impl Imu3dmCommandField for GyroscopeBiasConfiguration {}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CaptureGyroscopeBias {
    pub averaging_time_ms: u16,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct CaptureGyroscopeBiasResponse {
    pub bias_rad_s: Vector3f,
}

impl CommandResponseData for CaptureGyroscopeBiasResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() < 12 {
            return Err(len_too_short(0x39, 12, data.len()));
        }
        Ok(Self {
            bias_rad_s: parse_vector3f(data),
        })
    }
}

impl CommandField for CaptureGyroscopeBias {
    type Response = CaptureGyroscopeBiasResponse;

    fn descriptor_set(&self) -> u8 {
        IMU_3DM_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x39
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        if buf.len() < 2 {
            return Err(SerializeError::OutOfSpace);
        }
        buf[..2].copy_from_slice(&self.averaging_time_ms.to_be_bytes());
        Ok(2)
    }
}
impl Imu3dmCommandField for CaptureGyroscopeBias {}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum MagnetometerHardIronOffset {
    Write { offset_gauss: Vector3f },
    Read,
    Save,
    Load,
    Default,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct MagnetometerHardIronOffsetResponse {
    pub offset_gauss: Vector3f,
}

impl CommandResponseData for MagnetometerHardIronOffsetResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() < 12 {
            return Err(len_too_short(0x3A, 12, data.len()));
        }
        Ok(Self {
            offset_gauss: parse_vector3f(data),
        })
    }
}

impl CommandField for MagnetometerHardIronOffset {
    type Response = MagnetometerHardIronOffsetResponse;

    fn descriptor_set(&self) -> u8 {
        IMU_3DM_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x3A
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        match self {
            Self::Write { offset_gauss } => {
                if buf.len() < 13 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Write as u8;
                write_vector3f(*offset_gauss, &mut buf[1..13]);
                Ok(13)
            }
            Self::Read => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Read as u8;
                Ok(1)
            }
            Self::Save => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Save as u8;
                Ok(1)
            }
            Self::Load => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Load as u8;
                Ok(1)
            }
            Self::Default => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Default as u8;
                Ok(1)
            }
        }
    }
}
impl Imu3dmCommandField for MagnetometerHardIronOffset {}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum MagnetometerSoftIronMatrix {
    Write { matrix: Matrix3f },
    Read,
    Save,
    Load,
    Default,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct MagnetometerSoftIronMatrixResponse {
    pub matrix: Matrix3f,
}

impl CommandResponseData for MagnetometerSoftIronMatrixResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() < 36 {
            return Err(len_too_short(0x3B, 36, data.len()));
        }
        Ok(Self {
            matrix: parse_matrix3f(data),
        })
    }
}

impl CommandField for MagnetometerSoftIronMatrix {
    type Response = MagnetometerSoftIronMatrixResponse;

    fn descriptor_set(&self) -> u8 {
        IMU_3DM_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x3B
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        match self {
            Self::Write { matrix } => {
                if buf.len() < 37 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Write as u8;
                write_matrix3f(*matrix, &mut buf[1..37]);
                Ok(37)
            }
            Self::Read => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Read as u8;
                Ok(1)
            }
            Self::Save => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Save as u8;
                Ok(1)
            }
            Self::Load => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Load as u8;
                Ok(1)
            }
            Self::Default => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Default as u8;
                Ok(1)
            }
        }
    }
}
impl Imu3dmCommandField for MagnetometerSoftIronMatrix {}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum UartBaudrate {
    Write { baud: u32 },
    Read,
    Save,
    Load,
    Default,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct UartBaudrateResponse {
    pub baud: u32,
}

impl CommandResponseData for UartBaudrateResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() < 4 {
            return Err(len_too_short(0x40, 4, data.len()));
        }
        Ok(Self {
            baud: parse_u32_be(data),
        })
    }
}

impl CommandField for UartBaudrate {
    type Response = UartBaudrateResponse;

    fn descriptor_set(&self) -> u8 {
        IMU_3DM_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x40
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        match self {
            Self::Write { baud } => {
                if buf.len() < 5 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Write as u8;
                buf[1..5].copy_from_slice(&baud.to_be_bytes());
                Ok(5)
            }
            Self::Read => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Read as u8;
                Ok(1)
            }
            Self::Save => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Save as u8;
                Ok(1)
            }
            Self::Load => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Load as u8;
                Ok(1)
            }
            Self::Default => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Default as u8;
                Ok(1)
            }
        }
    }
}
impl Imu3dmCommandField for UartBaudrate {}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum GpioConfiguration {
    Write {
        pin: u8,
        feature: u8,
        behavior: u8,
        pin_mode: u8,
    },
    Read {
        pin: u8,
    },
    Save {
        pin: u8,
    },
    Load {
        pin: u8,
    },
    Default {
        pin: u8,
    },
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct GpioConfigurationResponse {
    pub pin: u8,
    pub feature: u8,
    pub behavior: u8,
    pub pin_mode: u8,
}

impl CommandResponseData for GpioConfigurationResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() < 4 {
            return Err(len_too_short(0x41, 4, data.len()));
        }
        Ok(Self {
            pin: data[0],
            feature: data[1],
            behavior: data[2],
            pin_mode: data[3],
        })
    }
}

impl CommandField for GpioConfiguration {
    type Response = GpioConfigurationResponse;

    fn descriptor_set(&self) -> u8 {
        IMU_3DM_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x41
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        match self {
            Self::Write {
                pin,
                feature,
                behavior,
                pin_mode,
            } => {
                if buf.len() < 5 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Write as u8;
                buf[1] = *pin;
                buf[2] = *feature;
                buf[3] = *behavior;
                buf[4] = *pin_mode;
                Ok(5)
            }
            Self::Read { pin } => {
                if buf.len() < 2 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Read as u8;
                buf[1] = *pin;
                Ok(2)
            }
            Self::Save { pin } => {
                if buf.len() < 2 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Save as u8;
                buf[1] = *pin;
                Ok(2)
            }
            Self::Load { pin } => {
                if buf.len() < 2 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Load as u8;
                buf[1] = *pin;
                Ok(2)
            }
            Self::Default { pin } => {
                if buf.len() < 2 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Default as u8;
                buf[1] = *pin;
                Ok(2)
            }
        }
    }
}
impl Imu3dmCommandField for GpioConfiguration {}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum GpioState {
    Write { pin: u8, state: bool },
    Read { pin: u8 },
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct GpioStateResponse {
    pub pin: u8,
    pub state: bool,
}

impl CommandResponseData for GpioStateResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() < 2 {
            return Err(len_too_short(0x42, 2, data.len()));
        }
        Ok(Self {
            pin: data[0],
            state: data[1] != 0,
        })
    }
}

impl CommandField for GpioState {
    type Response = GpioStateResponse;

    fn descriptor_set(&self) -> u8 {
        IMU_3DM_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x42
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        match self {
            Self::Write { pin, state } => {
                if buf.len() < 3 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Write as u8;
                buf[1] = *pin;
                buf[2] = *state as u8;
                Ok(3)
            }
            Self::Read { pin } => {
                if buf.len() < 2 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Read as u8;
                buf[1] = *pin;
                Ok(2)
            }
        }
    }
}
impl Imu3dmCommandField for GpioState {}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum AdvancedLowPassFilterSettings {
    Write {
        target_descriptor: u8,
        enable: bool,
        manual: bool,
        frequency_hz: u16,
        reserved: u8,
    },
    Read {
        target_descriptor: u8,
    },
    Save {
        target_descriptor: u8,
    },
    Load {
        target_descriptor: u8,
    },
    Default {
        target_descriptor: u8,
    },
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct AdvancedLowPassFilterSettingsResponse {
    pub target_descriptor: u8,
    pub enable: bool,
    pub manual: bool,
    pub frequency_hz: u16,
    pub reserved: u8,
}

impl CommandResponseData for AdvancedLowPassFilterSettingsResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() < 6 {
            return Err(len_too_short(0x50, 6, data.len()));
        }
        Ok(Self {
            target_descriptor: data[0],
            enable: data[1] != 0,
            manual: data[2] != 0,
            frequency_hz: parse_u16_be(&data[3..5]),
            reserved: data[5],
        })
    }
}

impl CommandField for AdvancedLowPassFilterSettings {
    type Response = AdvancedLowPassFilterSettingsResponse;

    fn descriptor_set(&self) -> u8 {
        IMU_3DM_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x50
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        match self {
            Self::Write {
                target_descriptor,
                enable,
                manual,
                frequency_hz,
                reserved,
            } => {
                if buf.len() < 7 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Write as u8;
                buf[1] = *target_descriptor;
                buf[2] = *enable as u8;
                buf[3] = *manual as u8;
                buf[4..6].copy_from_slice(&frequency_hz.to_be_bytes());
                buf[6] = *reserved;
                Ok(7)
            }
            Self::Read { target_descriptor } => {
                if buf.len() < 2 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Read as u8;
                buf[1] = *target_descriptor;
                Ok(2)
            }
            Self::Save { target_descriptor } => {
                if buf.len() < 2 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Save as u8;
                buf[1] = *target_descriptor;
                Ok(2)
            }
            Self::Load { target_descriptor } => {
                if buf.len() < 2 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Load as u8;
                buf[1] = *target_descriptor;
                Ok(2)
            }
            Self::Default { target_descriptor } => {
                if buf.len() < 2 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Default as u8;
                buf[1] = *target_descriptor;
                Ok(2)
            }
        }
    }
}
impl Imu3dmCommandField for AdvancedLowPassFilterSettings {}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum ComplementaryFilterConfiguration {
    Write {
        pitch_roll_enable: bool,
        heading_enable: bool,
        pitch_roll_time_constant_s: f32,
        heading_time_constant_s: f32,
    },
    Read,
    Save,
    Load,
    Default,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct ComplementaryFilterConfigurationResponse {
    pub pitch_roll_enable: bool,
    pub heading_enable: bool,
    pub pitch_roll_time_constant_s: f32,
    pub heading_time_constant_s: f32,
}

impl CommandResponseData for ComplementaryFilterConfigurationResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() < 10 {
            return Err(len_too_short(0x51, 10, data.len()));
        }
        Ok(Self {
            pitch_roll_enable: data[0] != 0,
            heading_enable: data[1] != 0,
            pitch_roll_time_constant_s: parse_f32_be(&data[2..6]),
            heading_time_constant_s: parse_f32_be(&data[6..10]),
        })
    }
}

impl CommandField for ComplementaryFilterConfiguration {
    type Response = ComplementaryFilterConfigurationResponse;

    fn descriptor_set(&self) -> u8 {
        IMU_3DM_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x51
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        match self {
            Self::Write {
                pitch_roll_enable,
                heading_enable,
                pitch_roll_time_constant_s,
                heading_time_constant_s,
            } => {
                if buf.len() < 11 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Write as u8;
                buf[1] = *pitch_roll_enable as u8;
                buf[2] = *heading_enable as u8;
                write_f32_be(*pitch_roll_time_constant_s, &mut buf[3..7]);
                write_f32_be(*heading_time_constant_s, &mut buf[7..11]);
                Ok(11)
            }
            Self::Read => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Read as u8;
                Ok(1)
            }
            Self::Save => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Save as u8;
                Ok(1)
            }
            Self::Load => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Load as u8;
                Ok(1)
            }
            Self::Default => {
                if buf.is_empty() {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Default as u8;
                Ok(1)
            }
        }
    }
}
impl Imu3dmCommandField for ComplementaryFilterConfiguration {}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum SensorRange {
    Write {
        sensor: SensorRangeType,
        setting: u8,
    },
    Read {
        sensor: SensorRangeType,
    },
    Save {
        sensor: SensorRangeType,
    },
    Load {
        sensor: SensorRangeType,
    },
    Default {
        sensor: SensorRangeType,
    },
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct SensorRangeResponse {
    pub sensor: u8,
    pub setting: u8,
}

impl CommandResponseData for SensorRangeResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() < 2 {
            return Err(len_too_short(0x52, 2, data.len()));
        }
        Ok(Self {
            sensor: data[0],
            setting: data[1],
        })
    }
}

impl CommandField for SensorRange {
    type Response = SensorRangeResponse;

    fn descriptor_set(&self) -> u8 {
        IMU_3DM_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x52
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        match self {
            Self::Write { sensor, setting } => {
                if buf.len() < 3 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Write as u8;
                buf[1] = *sensor as u8;
                buf[2] = *setting;
                Ok(3)
            }
            Self::Read { sensor } => {
                if buf.len() < 2 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Read as u8;
                buf[1] = *sensor as u8;
                Ok(2)
            }
            Self::Save { sensor } => {
                if buf.len() < 2 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Save as u8;
                buf[1] = *sensor as u8;
                Ok(2)
            }
            Self::Load { sensor } => {
                if buf.len() < 2 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Load as u8;
                buf[1] = *sensor as u8;
                Ok(2)
            }
            Self::Default { sensor } => {
                if buf.len() < 2 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Default as u8;
                buf[1] = *sensor as u8;
                Ok(2)
            }
        }
    }
}
impl Imu3dmCommandField for SensorRange {}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct GetCalibratedSensorRanges {
    pub sensor: SensorRangeType,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct GetCalibratedSensorRangesResponse {
    pub sensor: u8,
    pub ranges: [SensorRangeEntry; MAX_RANGE_ENTRIES],
    pub len: usize,
}

impl CommandResponseData for GetCalibratedSensorRangesResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() < 2 {
            return Err(len_too_short(0x53, 2, data.len()));
        }
        let n = data[1] as usize;
        let need = 2 + n * 5;
        if data.len() < need {
            return Err(len_too_short(0x53, need, data.len()));
        }
        if n > MAX_RANGE_ENTRIES {
            return Err(ParseError::UnknownField {
                descriptor_set: IMU_3DM_DESCRIPTOR_SET,
                descriptor: 0x53,
            });
        }
        let mut ranges = [SensorRangeEntry::default(); MAX_RANGE_ENTRIES];
        for (i, slot) in ranges.iter_mut().enumerate().take(n) {
            let b = 2 + i * 5;
            *slot = SensorRangeEntry {
                setting: data[b],
                range: parse_f32_be(&data[b + 1..b + 5]),
            };
        }
        Ok(Self {
            sensor: data[0],
            ranges,
            len: n,
        })
    }
}

impl CommandField for GetCalibratedSensorRanges {
    type Response = GetCalibratedSensorRangesResponse;

    fn descriptor_set(&self) -> u8 {
        IMU_3DM_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x53
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        if buf.is_empty() {
            return Err(SerializeError::OutOfSpace);
        }
        buf[0] = self.sensor as u8;
        Ok(1)
    }
}
impl Imu3dmCommandField for GetCalibratedSensorRanges {}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum LowPassAntiAliasingFilter {
    Write {
        descriptor_set: u8,
        field_desc: u8,
        enable: bool,
        manual: bool,
        frequency_hz: f32,
    },
    Read {
        descriptor_set: u8,
        field_desc: u8,
    },
    Save {
        descriptor_set: u8,
        field_desc: u8,
    },
    Load {
        descriptor_set: u8,
        field_desc: u8,
    },
    Default {
        descriptor_set: u8,
        field_desc: u8,
    },
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct LowPassAntiAliasingFilterResponse {
    pub descriptor_set: u8,
    pub field_desc: u8,
    pub enable: bool,
    pub manual: bool,
    pub frequency_hz: f32,
}

impl CommandResponseData for LowPassAntiAliasingFilterResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() < 8 {
            return Err(len_too_short(0x54, 8, data.len()));
        }
        Ok(Self {
            descriptor_set: data[0],
            field_desc: data[1],
            enable: data[2] != 0,
            manual: data[3] != 0,
            frequency_hz: parse_f32_be(&data[4..8]),
        })
    }
}

impl CommandField for LowPassAntiAliasingFilter {
    type Response = LowPassAntiAliasingFilterResponse;

    fn descriptor_set(&self) -> u8 {
        IMU_3DM_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x54
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        match self {
            Self::Write {
                descriptor_set,
                field_desc,
                enable,
                manual,
                frequency_hz,
            } => {
                if buf.len() < 9 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Write as u8;
                buf[1] = *descriptor_set;
                buf[2] = *field_desc;
                buf[3] = *enable as u8;
                buf[4] = *manual as u8;
                write_f32_be(*frequency_hz, &mut buf[5..9]);
                Ok(9)
            }
            Self::Read {
                descriptor_set,
                field_desc,
            } => {
                if buf.len() < 3 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Read as u8;
                buf[1] = *descriptor_set;
                buf[2] = *field_desc;
                Ok(3)
            }
            Self::Save {
                descriptor_set,
                field_desc,
            } => {
                if buf.len() < 3 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Save as u8;
                buf[1] = *descriptor_set;
                buf[2] = *field_desc;
                Ok(3)
            }
            Self::Load {
                descriptor_set,
                field_desc,
            } => {
                if buf.len() < 3 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Load as u8;
                buf[1] = *descriptor_set;
                buf[2] = *field_desc;
                Ok(3)
            }
            Self::Default {
                descriptor_set,
                field_desc,
            } => {
                if buf.len() < 3 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = FunctionSelector::Default as u8;
                buf[1] = *descriptor_set;
                buf[2] = *field_desc;
                Ok(3)
            }
        }
    }
}
impl Imu3dmCommandField for LowPassAntiAliasingFilter {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_get_data_base_rate_serialize() {
        let cmd = GetDataBaseRate {
            descriptor_set: 0x80,
        };
        let mut buf = [0u8; 8];
        let cnt = cmd.serialize(&mut buf).unwrap();
        assert_eq!(3, cnt);
        assert_eq!([3, 0x0E, 0x80], buf[..3]);
    }

    #[test]
    fn test_message_format_write_serialize() {
        let entries = [
            DescriptorRate {
                descriptor: 0x04,
                decimation: 10,
            },
            DescriptorRate {
                descriptor: 0x05,
                decimation: 20,
            },
        ];
        let cmd = MessageFormat::Write {
            descriptor_set: 0x80,
            descriptors: &entries,
        };
        let mut buf = [0u8; 32];
        let cnt = cmd.serialize(&mut buf).unwrap();
        assert_eq!(11, cnt);
        assert_eq!(11, buf[0]);
        assert_eq!(0x0F, buf[1]);
        assert_eq!(FunctionSelector::Write as u8, buf[2]);
        assert_eq!(0x80, buf[3]);
        assert_eq!(2, buf[4]);
    }

    #[test]
    fn test_data_stream_control_response_parse() {
        let parsed = DataStreamControlResponse::from_data(&[0x80, 1]).unwrap();
        assert_eq!(0x80, parsed.descriptor_set);
        assert!(parsed.enabled);
    }
}
