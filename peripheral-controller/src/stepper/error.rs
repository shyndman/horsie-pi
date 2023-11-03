use core::{error::Error, fmt::Display};

use anyhow::anyhow;

#[derive(Debug)]
pub enum ErrorKind {
    ConnectionFailed,
    UartGeneral(esp_hal_common::uart::Error),
    UartReadExact(embedded_io::ReadExactError<esp_hal_common::uart::Error>),
    MissingResponse,
    WriteFailed,
}
impl Display for ErrorKind {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{:?}", self)
    }
}
impl Error for ErrorKind {}

impl Into<anyhow::Error> for ErrorKind {
    fn into(self) -> anyhow::Error {
        anyhow!(self)
    }
}
