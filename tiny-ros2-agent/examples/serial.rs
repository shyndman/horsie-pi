use std::{str, time::Duration};

use futures::TryStreamExt;
use tiny_ros2_codec::{foundation::Decoder, RosSerialCodec, RosSerialMessage};
use tokio_serial::SerialPortBuilderExt;
use tokio_util::codec::Decoder as TokioDecoder;
use tracing::info;
use tracing_subscriber::{fmt, prelude::*, EnvFilter};

const TTY_DEVICE: &str = "/dev/ttyUSB0";
const BAUDRATE: u32 = 9600;

#[tokio::main]
async fn main() -> tokio_serial::Result<()> {
    tracing_subscriber::registry()
        .with(fmt::layer())
        .with(EnvFilter::from_default_env())
        .init();

    info!("Opening {} at {} baud", TTY_DEVICE, BAUDRATE);
    let mut port = tokio_serial::new(TTY_DEVICE, BAUDRATE)
        .timeout(Duration::from_secs(1))
        .open_native_async()?;
    port.set_exclusive(false)
        .expect("Unable to attain exclusive serial port access");
    info!("...done");

    let mut reader = TokioRosSerialCodec::new().framed(port);
    loop {
        match reader.try_next().await {
            Ok(Some(msg)) => {
                println!("{:?}", msg);
            }
            Ok(None) => {
                println!("no message read");
            }
            Err(e) => {
                println!("error: {}", e);
            }
        }
    }
}

struct TokioRosSerialCodec {
    inner_codec: RosSerialCodec,
}

impl TokioRosSerialCodec {
    fn new() -> Self {
        Self {
            inner_codec: RosSerialCodec::new(),
        }
    }
}
impl TokioDecoder for TokioRosSerialCodec {
    type Item = RosSerialMessage;
    type Error = std::io::Error;

    fn decode(
        &mut self,
        src: &mut bytes::BytesMut,
    ) -> Result<Option<Self::Item>, Self::Error> {
        println!("TokioDecoder::decode");
        let res = self.inner_codec.decode(src);
        res.map_err(|ros_err| std::io::Error::new(std::io::ErrorKind::InvalidData, ros_err))
    }
}
