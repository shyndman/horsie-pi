use core::{
    error::Error,
    fmt::{Debug, Display},
};

#[derive(Debug)]
pub enum ErrorKind {
    MissingSyncNibble,
    IncompatibleVersion,
    LengthCrcMismatch,
    BodyCrcMismatch,
    MessageDecodeFailure,
    UnknownTopicId,
    WrongByteCountDecodeFailure,
}

impl Into<RosSerialError> for ErrorKind {
    fn into(self) -> RosSerialError {
        RosSerialError::new(self)
    }
}

#[derive(Debug)]
pub struct RosSerialError {
    pub kind: ErrorKind,
}
impl RosSerialError {
    pub fn new(kind: ErrorKind) -> Self {
        Self { kind }
    }
}
impl Display for RosSerialError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        self.kind.fmt(f)
    }
}
impl Error for RosSerialError {}
