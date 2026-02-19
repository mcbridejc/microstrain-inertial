//! Device Control Interfaces

use thiserror::Error;

mod async_interface;
mod data_buffer;

pub use async_interface::*;
pub use data_buffer::*;

use crate::{
    api::commands::SerializeError,
    errors::ParseError,
};

#[derive(Clone, Copy, Debug, Error, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Error type for reading recieved data packets from an interface
pub enum ReadDataError {
    /// data buffer overrun
    #[error("data buffer overrun")]
    Overrun,
}

#[derive(Clone, Copy, Debug, Error, PartialEq, Eq)]
/// Error when sending commands via an interface
pub enum CommandSendError {
    /// Command serialization failed
    #[error("Command serialization failed")]
    SerializeFailed(#[from] SerializeError),
    /// An error occurred parsing the response to the command
    #[error("An error occurred parsing the response to the command")]
    ResponseParseError(#[from] ParseError),
    /// Another command is still pending
    #[error("Another command is still pending")]
    CommandInProgress,
    ///The received response did not match expectations
    #[error("The received response did not match expectations")]
    UnexpectedResponse,
}
