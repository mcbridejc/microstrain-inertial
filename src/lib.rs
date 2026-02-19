//! Microstrain IMU driver
//!
//! # Overview
//!
//! A no_std compatible driver for exchanging MIP packets with microstrain IMU devices over a serial
//! interface.
//!
//! This was written and tested with the 3DM-CV7, but should be generally applicable to other
//! devices. But it may be required to extend the message definitions in [`api`] with new messages.
//!
//! # Structure
//!
//! ## Message Framing - the [`framer`] module
//!
//! This module provides a parser for parsing a stream of bytes into frames, including rewind and
//! re-parse on checksum error. The parser uses the framing, length and CRC bytes to find a frame,
//! extract the contents (descriptor set, and payload) and return them.
//!
//! ## Message Definitions - the [`api`] module
//!
//! Message content is defined in the [`api`] module. There are two general types of messages:
//! [`commands`](api::commands) and [`data`](api::data). These are categorized further into a few
//! "descriptor sets". Each packet consists of one of more fields, all of which belong to the same
//! descriptor set.
//!
//! ## Device interaction - the [`interface`] module
//!
//! An interface provides an API for controlling a single device. This primarily means the ability
//! to send a command and receive back the response, and to receive streaming data fields.
//! Currently, there is only one interface implementation:
//! [`AsyncInterface`](interface::AsyncInterface).
//!
//! # Using OS serial ports
//!
//! For convenience, the [`serialport`] module (requires `std`) provides a
//! function to launch OS threads for performing background serial port IO
#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(docsrs, feature(doc_cfg))]
#![deny(missing_docs)]

pub mod api;
pub mod checksum;
pub mod errors;
pub mod framer;
pub mod interface;
pub mod owned_packet;
#[cfg(feature = "serialport")]
#[cfg_attr(docsrs, doc(cfg(feature = "serialport")))]
pub mod serialport;

/// first header sync byte
pub const SYNC1: u8 = 0x75;
/// second header sync byte
pub const SYNC2: u8 = 0x65;

/// The size of the CRC section at the end of a MIP packet
const CRC_SIZE: usize = 2;
/// The size of the header at the front of each MIP packet (2 sync, 1 len, 1 descriptor set)
const HEADER_SIZE: usize = 4;
/// The max payload size of a MIP packet
const MAX_PAYLOAD_SIZE: usize = 255;
/// The maximum serialized size of a complete MIP packet, SYNC to CRC
pub const MAX_PACKET_SIZE: usize = HEADER_SIZE + MAX_PAYLOAD_SIZE + CRC_SIZE;
