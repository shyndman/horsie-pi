use bytes::BytesMut;

use crate::error::RosSerialError;

pub trait Decoder {
    type Item;
    type Error: From<RosSerialError>;

    fn decode(&mut self, src: &mut BytesMut) -> Result<Option<Self::Item>, Self::Error>;
}

pub trait Encoder<Item> {
    type Error: From<RosSerialError>;

    fn encode(&mut self, item: Item, dst: &mut BytesMut) -> Result<(), Self::Error>;
}

pub fn crc(data: &[u8]) -> u8 {
    let mut crc = 0u8;
    for mut byte in data.iter().cloned() {
        for _ in 0..8 {
            if ((crc >> 7) ^ (byte & 0x01)) != 0 {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = crc << 1;
            }
            byte = byte >> 1;
        }
    }
    crc
}

pub fn resume_crc(last_crc: u8, data: &[u8]) -> u8 {
    let mut crc = last_crc;
    for mut byte in data.iter().cloned() {
        for _ in 0..8 {
            if ((crc >> 7) ^ (byte & 0x01)) != 0 {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = crc << 1;
            }
            byte = byte >> 1;
        }
    }
    crc
}
