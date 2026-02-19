//! Command and data message definitions
//!
//! MIP packets are broken up into sets, called "descriptor sets". There are two types of descriptor
//! sets, "Command" and "Data". Command descriptor sets include packets sent to the IMU for
//! configuraiton and control, as well as packets sent in response to those commands. Data
//! descriptor sets are only sent by the IMU, and contain the streamed data like sensor readings, or
//! state estimator outputs, etc.
//!
//! Each packet, regardless of type, can contain many different fields, each with a different
//! descriptor, so that multiple commands or data fields can be packed into a single packet.
//! Only fields which belong to the same descriptor set can be combined into a packet.

pub mod commands;
pub mod data;
mod fields;
mod parse;
pub mod types;
