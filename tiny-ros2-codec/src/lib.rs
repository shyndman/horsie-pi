#![no_std]
#![feature(error_in_core)]

#[cfg(test)]
extern crate std;

mod codec;
mod error;
pub mod foundation;
mod message;

pub use codec::RosSerialCodec;
pub use error::*;
pub use message::*;
