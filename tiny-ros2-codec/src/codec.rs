use bitreader::BitReader;
use bytes::{Buf, BufMut, BytesMut};

use crate::{
    error::{ErrorKind, RosSerialError},
    foundation::{crc, resume_crc, Decoder, Encoder},
    RosSerialMessage,
};

/// Sent at the beginning of a message
const SYNC_NIBBLE: u8 = 0b0101;

/// The version of the protocol we are capable reading and writing
const PROTOCOL_VERSION: u16 = 1;

/// 0..4          5..16
/// SYNC_NIBBLE | PROTOCOL_VERSION
pub const PREAMBLE_LEN: usize = 2;

///
pub const TOPIC_ID_LEN: usize = 2;

/// 0..15      16..23         24..39
/// BODY_LEN | BODY_LEN_CRC | TOPIC_ID
pub const HEADER_LEN: usize = 3 + TOPIC_ID_LEN;

/// Speaks the language used to communicate between the main-brain Raspberry Pi and
/// various MPUs strewn throughout Horsie Pi.
pub struct RosSerialCodec {
    /// The current context of envelope decoding
    decode_state: DecodeState,
}

/// The currently occupied state in the decoding state machine
enum DecodeState {
    Preamble,
    Header,
    Body { body_len: usize, topic_id: u16 },
}

impl RosSerialCodec {
    pub fn new() -> Self {
        RosSerialCodec {
            decode_state: DecodeState::Preamble,
        }
    }
}

impl Decoder for RosSerialCodec {
    type Item = RosSerialMessage;
    type Error = RosSerialError;

    fn decode(&mut self, src: &mut BytesMut) -> Result<Option<Self::Item>, Self::Error> {
        match self.decode_state {
            DecodeState::Preamble => {
                if src.len() < PREAMBLE_LEN {
                    return Ok(None);
                }

                let preamble = src.split_to(PREAMBLE_LEN);
                let mut cursor = BitReader::new(&preamble);

                let sync = cursor.read_u8(4).unwrap();
                if sync != SYNC_NIBBLE {
                    return Err(ErrorKind::MissingSyncNibble.into());
                }

                let version = cursor.read_u16(12).unwrap();
                if version != PROTOCOL_VERSION {
                    return Err(ErrorKind::IncompatibleVersion.into());
                }

                self.decode_state = DecodeState::Header;
                self.decode(src)
            }
            DecodeState::Header => {
                if src.len() < HEADER_LEN {
                    return Ok(None);
                }

                let header = src.split_to(HEADER_LEN);
                let mut cursor = BitReader::new(&header);

                // Parse payload length and check its CRC
                let mut buf: [u8; 2] = [0; 2];
                cursor.read_u8_slice(&mut buf).unwrap();
                if crc(&buf) != cursor.read_u8(8).unwrap() {
                    return Err(ErrorKind::LengthCrcMismatch.into());
                }

                let body_len = u16::from_ne_bytes(buf) as usize;
                // Note: This will be verified alongside the body bytes during
                // `DecodeState::Body`
                let topic_id = cursor.read_u16(16).unwrap();

                // Reserving space for BODY + CRC + (next frame...)
                src.reserve(body_len as usize + 1 + PREAMBLE_LEN);
                self.decode_state = DecodeState::Body { body_len, topic_id };
                self.decode(src)
            }
            DecodeState::Body { body_len, topic_id } => {
                if src.len() < body_len + 1 {
                    return Ok(None);
                }

                let body_bytes = src.split_to(body_len);
                let topic_id_bytes = topic_id.to_be_bytes();
                let crc = resume_crc(crc(&body_bytes), &topic_id_bytes);

                if crc != src.get_u8() {
                    return Err(ErrorKind::BodyCrcMismatch.into());
                }

                self.decode_state = DecodeState::Preamble;
                match RosSerialMessage::try_from_id_bytes(topic_id, &body_bytes) {
                    Ok(msg) => Ok(Some(msg)),
                    Err(_) => Err(ErrorKind::MessageDecodeFailure.into()),
                }
            }
        }
    }
}

impl Encoder<RosSerialMessage> for RosSerialCodec {
    type Error = RosSerialError;

    fn encode(
        &mut self,
        item: RosSerialMessage,
        dst: &mut BytesMut,
    ) -> Result<(), Self::Error> {
        dst.reserve(PREAMBLE_LEN + HEADER_LEN);

        // PREAMBLE
        let preamble: u16 = ((SYNC_NIBBLE as u16) << 12) | PROTOCOL_VERSION as u16;
        dst.put_u16_ne(preamble);

        // HEADER
        // Split off a mutable buffer for the header, because we don't know the length
        // of the body yet.
        const LENGTH_AND_CRC_LEN: usize = 2 + 1; // u16 length + u8 crc
        let mut length_and_crc_dst = dst.split_to(LENGTH_AND_CRC_LEN);

        // Write the topic ID, because we do know that
        let topic_id = item.topic_id();
        let topic_id_bytes = topic_id.to_ne_bytes();
        dst.put_u16_ne(topic_id);

        // BODY
        // Snapshot the length of the body. We'll use it in a sec
        let (body_start, body_end_exclusive) = {
            let mark = dst.len();
            match item {
                RosSerialMessage::TopicInfo => todo!(),
                RosSerialMessage::ParameterRequest => todo!(),
                RosSerialMessage::Log { level } => {
                    dst.reserve(1);
                    dst.put_u8(level as u8);
                }
                RosSerialMessage::Time {
                    seconds: sec,
                    nanos: nanosec,
                } => {
                    dst.reserve(8);
                    dst.put_u32_ne(sec);
                    dst.put_u32_ne(nanosec);
                }
            }
            (mark as usize, dst.len() as usize)
        };

        let body_crc = crc(&dst[body_start..body_end_exclusive]);
        let body_crc = resume_crc(body_crc, &topic_id_bytes);
        dst.put_u8(body_crc);

        // Finally, write out the length bytes and the CRC in the space we left back in
        // header
        let body_len_bytes = ((body_end_exclusive - body_start) as u16).to_ne_bytes();
        length_and_crc_dst.put_slice(&body_len_bytes);
        length_and_crc_dst.put_u8(crc(&body_len_bytes));

        Ok(())
    }
}
