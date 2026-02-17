//! System (0x7F) commands.

use super::base::FunctionSelector;
use super::{CommandField, CommandResponseData, SYSTEM_DESCRIPTOR_SET, SerializeError};
use crate::api::parse::ReadBuf as _;
use crate::errors::ParseError;

/// Marker trait for System (0x7F) commands.
pub trait SystemCommandField: CommandField {}

fn len_too_short(descriptor: u8, need: usize, got: usize) -> ParseError {
    ParseError::LenTooShort {
        descriptor_set: SYSTEM_DESCRIPTOR_SET,
        descriptor,
        need,
        got,
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[repr(u8)]
pub enum SystemPort {
    All = 0,
    Main = 1,
    Uart1 = 17,
    Uart2 = 18,
    Uart3 = 19,
    Usb1 = 33,
    Usb2 = 34,
}

impl SystemPort {
    fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(Self::All),
            1 => Some(Self::Main),
            17 => Some(Self::Uart1),
            18 => Some(Self::Uart2),
            19 => Some(Self::Uart3),
            33 => Some(Self::Usb1),
            34 => Some(Self::Usb2),
            _ => None,
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct InterfaceControlResponse {
    pub port: SystemPort,
    pub protocols_incoming: u32,
    pub protocols_outgoing: u32,
}

impl CommandResponseData for InterfaceControlResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() < 9 {
            return Err(len_too_short(0x02, 9, data.len()));
        }
        let mut b = data;
        let port = SystemPort::from_u8(b.read_u8()).ok_or(ParseError::UnknownField {
            descriptor_set: SYSTEM_DESCRIPTOR_SET,
            descriptor: 0x02,
        })?;
        Ok(Self {
            port,
            protocols_incoming: b.read_u32(),
            protocols_outgoing: b.read_u32(),
        })
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct InterfaceControlWrite {
    pub port: SystemPort,
    pub protocols_incoming: u32,
    pub protocols_outgoing: u32,
}

impl CommandField for InterfaceControlWrite {
    type Response = InterfaceControlResponse;

    fn descriptor_set(&self) -> u8 {
        SYSTEM_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x02
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        if buf.len() < 10 {
            return Err(SerializeError::OutOfSpace);
        }
        buf[0] = FunctionSelector::Write as u8;
        buf[1] = self.port as u8;
        buf[2..6].copy_from_slice(&self.protocols_incoming.to_be_bytes());
        buf[6..10].copy_from_slice(&self.protocols_outgoing.to_be_bytes());
        Ok(10)
    }
}
impl SystemCommandField for InterfaceControlWrite {}

macro_rules! impl_system_selector_port_command {
    ($name:ident, $selector:expr) => {
        #[derive(Debug, Copy, Clone, PartialEq, Eq)]
        pub struct $name {
            pub port: SystemPort,
        }

        impl CommandField for $name {
            type Response = ();

            fn descriptor_set(&self) -> u8 {
                SYSTEM_DESCRIPTOR_SET
            }

            fn descriptor(&self) -> u8 {
                0x02
            }

            fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
                if buf.len() < 2 {
                    return Err(SerializeError::OutOfSpace);
                }
                buf[0] = $selector as u8;
                buf[1] = self.port as u8;
                Ok(2)
            }
        }

        impl SystemCommandField for $name {}
    };
}

impl_system_selector_port_command!(InterfaceControlRead, FunctionSelector::Read);
impl_system_selector_port_command!(InterfaceControlSave, FunctionSelector::Save);
impl_system_selector_port_command!(InterfaceControlLoad, FunctionSelector::Load);
impl_system_selector_port_command!(InterfaceControlDefault, FunctionSelector::Default);

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CommModeResponse {
    pub mode: u8,
}

impl CommandResponseData for CommModeResponse {
    fn from_data(data: &[u8]) -> Result<Self, ParseError> {
        if data.is_empty() {
            return Err(len_too_short(0x10, 1, data.len()));
        }
        Ok(Self { mode: data[0] })
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CommModeWrite {
    pub mode: u8,
}

impl CommandField for CommModeWrite {
    type Response = CommModeResponse;

    fn descriptor_set(&self) -> u8 {
        SYSTEM_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x10
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        if buf.len() < 2 {
            return Err(SerializeError::OutOfSpace);
        }
        buf[0] = FunctionSelector::Write as u8;
        buf[1] = self.mode;
        Ok(2)
    }
}
impl SystemCommandField for CommModeWrite {}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CommModeRead;

impl CommandField for CommModeRead {
    type Response = ();

    fn descriptor_set(&self) -> u8 {
        SYSTEM_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x10
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        if buf.is_empty() {
            return Err(SerializeError::OutOfSpace);
        }
        buf[0] = FunctionSelector::Read as u8;
        Ok(1)
    }
}
impl SystemCommandField for CommModeRead {}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CommModeDefault;

impl CommandField for CommModeDefault {
    type Response = ();

    fn descriptor_set(&self) -> u8 {
        SYSTEM_DESCRIPTOR_SET
    }

    fn descriptor(&self) -> u8 {
        0x10
    }

    fn serialize_payload(&self, buf: &mut [u8]) -> Result<u8, SerializeError> {
        if buf.is_empty() {
            return Err(SerializeError::OutOfSpace);
        }
        buf[0] = FunctionSelector::Default as u8;
        Ok(1)
    }
}
impl SystemCommandField for CommModeDefault {}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_response_type<C: CommandField<Response = R>, R>(_cmd: C) {}

    #[test]
    fn test_interface_control_write_serialize() {
        let cmd = InterfaceControlWrite {
            port: SystemPort::Usb1,
            protocols_incoming: 1,
            protocols_outgoing: 0x100,
        };
        let mut buf = [0u8; 16];
        let cnt = cmd.serialize(&mut buf).unwrap();
        assert_eq!(12, cnt);
        assert_eq!(0x02, buf[1]);
        assert_eq!(FunctionSelector::Write as u8, buf[2]);
    }

    #[test]
    fn test_interface_control_response_parse() {
        let parsed = InterfaceControlResponse::from_data(&[33, 0, 0, 0, 1, 0, 0, 1, 0]).unwrap();
        assert_eq!(SystemPort::Usb1, parsed.port);
        assert_eq!(1, parsed.protocols_incoming);
        assert_eq!(0x100, parsed.protocols_outgoing);
    }

    #[test]
    fn test_comm_mode_serialize() {
        let write = CommModeWrite { mode: 2 };
        let read = CommModeRead;
        let default = CommModeDefault;
        let mut buf = [0u8; 8];

        assert_eq!(4, write.serialize(&mut buf).unwrap());
        assert_eq!(FunctionSelector::Write as u8, buf[2]);
        assert_eq!(2, buf[3]);
        assert_eq!(3, read.serialize(&mut buf).unwrap());
        assert_eq!(FunctionSelector::Read as u8, buf[2]);
        assert_eq!(3, default.serialize(&mut buf).unwrap());
        assert_eq!(FunctionSelector::Default as u8, buf[2]);
    }

    #[test]
    fn test_response_type_distinction() {
        assert_response_type::<CommModeWrite, CommModeResponse>(CommModeWrite { mode: 1 });
        assert_response_type::<CommModeRead, ()>(CommModeRead);
    }
}
