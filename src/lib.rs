//#![cfg_attr(not(test), no_std)]

pub mod api;
pub mod checksum;
pub mod errors;
pub mod fields;
pub mod framer;
pub mod interface;
pub mod serialize;
#[cfg(feature = "serialport")]
pub mod serialport;

/// first header sync byte
const SYNC1: u8 = 0x75;
/// second header sync byte
const SYNC2: u8 = 0x65;

/// The size of the CRC section at the end of a MIP packet
const CRC_SIZE: usize = 2;
/// The size of the header at the front of each MIP packet (2 sync, 1 len, 1 descriptor set)
const HEADER_SIZE: usize = 4;
/// The max payload size of a MIP packet
const MAX_PAYLOAD_SIZE: usize = 255;
/// The maximum serialized size of a complete MIP packet, SYNC to CRC
const MAX_PACKET_SIZE: usize = HEADER_SIZE + MAX_PAYLOAD_SIZE + CRC_SIZE;
