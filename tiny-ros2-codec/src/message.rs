use bitreader::BitReader;
use bytes::BytesMut;
use num_enum::TryFromPrimitive;

use crate::{ErrorKind, RosSerialError};

const PUBLISHER_ID: u16 = 1;
const SUBSCRIBER_ID: u16 = 2;
const PARAMETER_REQUEST_ID: u16 = 3;
const LOG_ID: u16 = 4;
const TIME_ID: u16 = 5;

#[derive(Clone, Copy, Debug)]
pub enum RosSerialMessage {
    TopicInfo,
    ParameterRequest,
    Log {
        level: RosLogLevel,
    },
    Time {
        /// The seconds component, valid over all int32 values.
        seconds: u32,

        /// The nanoseconds component, valid in the range [0, 10e9).
        nanos: u32,
    },
}

impl RosSerialMessage {
    pub fn topic_id(self) -> u16 {
        match self {
            RosSerialMessage::TopicInfo => todo!(),
            RosSerialMessage::ParameterRequest => PARAMETER_REQUEST_ID,
            RosSerialMessage::Log { .. } => LOG_ID,
            RosSerialMessage::Time { .. } => TIME_ID,
        }
    }

    pub fn try_from_id_bytes(
        topic_id: u16,
        message_bytes: &BytesMut,
    ) -> Result<Self, RosSerialError> {
        match topic_id {
            PUBLISHER_ID | SUBSCRIBER_ID => Ok(RosSerialMessage::TopicInfo),
            PARAMETER_REQUEST_ID => Ok(RosSerialMessage::ParameterRequest),
            LOG_ID => parse_log_msg(message_bytes),
            TIME_ID => parse_time_msg(message_bytes),
            _ => Err(ErrorKind::UnknownTopicId.into()),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, TryFromPrimitive)]
// #[num_enum(error_type(name = CustomError, constructor = CustomError::new))]
#[repr(u8)]
pub enum RosLogLevel {
    Debug = 0b001,
    Info = 0b010,
    Warn = 0b011,
    Error = 0b100,
    Fatal = 0b101,
}

fn parse_log_msg(message_bytes: &BytesMut) -> Result<RosSerialMessage, RosSerialError> {
    if message_bytes.len() == 1 {
        let mut cursor = BitReader::new(&message_bytes);

        match RosLogLevel::try_from(cursor.read_u8(8).unwrap()) {
            Ok(level) => Ok(RosSerialMessage::Log { level: level }),
            Err(_) => Err(ErrorKind::MessageDecodeFailure.into()),
        }
    } else {
        Err(ErrorKind::WrongByteCountDecodeFailure.into())
    }
}

fn parse_time_msg(message_bytes: &BytesMut) -> Result<RosSerialMessage, RosSerialError> {
    if message_bytes.len() == 4 {
        let mut cursor = BitReader::new(&message_bytes);
        Ok(RosSerialMessage::Time {
            seconds: cursor.read_u32(32).unwrap(),
            nanos: cursor.read_u32(32).unwrap(),
        })
    } else {
        Err(ErrorKind::WrongByteCountDecodeFailure.into())
    }
}
