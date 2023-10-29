use core::fmt::Debug;

use embassy_sync::{blocking_mutex::raw::RawMutex, mutex::Mutex};
use embedded_io::ReadExactError;
use embedded_io_async::{Read, Write};
use esp_hal_common::Uart;

/// Error returned by I2C device implementations in this crate.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum UartDeviceError<BUS> {
    /// An operation on the inner I2C bus failed.
    Uart(BUS),
    /// Configuration of the inner I2C bus failed.
    Config,
}

/// UART device on a shared bus.
pub struct UartDevice<'a, M: 'static + RawMutex, P: 'static + esp_hal_common::uart::Instance>
{
    bus: &'a Mutex<M, Uart<'static, P>>,
}

impl<'a, M: RawMutex, P: 'static + esp_hal_common::uart::Instance> UartDevice<'a, M, P> {
    /// Create a new `UartDevice`.
    pub fn new(bus: &'a Mutex<M, Uart<'static, P>>) -> Self {
        Self { bus }
    }

    pub async fn read_exact(
        &mut self,
        buf: &mut [u8],
    ) -> Result<(), ReadExactError<esp32s3_hal::uart::Error>> {
        let mut bus = self.bus.lock().await;
        bus.read_exact(buf).await
    }

    pub async fn write_all(&mut self, buf: &[u8]) -> Result<(), esp32s3_hal::uart::Error> {
        let mut bus = self.bus.lock().await;
        bus.write_all(buf).await
    }

    pub async fn prepare_receive_buffer<const BUF_LEN: usize>(&mut self) {
        let mut bus = self.bus.lock().await;
        bus.reset_rx_fifo_full_interrupt();
        bus.set_rx_fifo_full_threshold(BUF_LEN as u16).unwrap();
    }

    // TODO(shyndman): The TMC2209 UART layer NEEDs this to safely access the serial line.
    // Implement!
    //
    // pub async fn transaction<F: FnMut(&mut Uart<'static, P>)>(
    //     &mut self,
    //     mut critical_section: F,
    // ) {
    //     let mut bus = self.bus.lock().await;
    //     critical_section(bus.borrow_mut());
    // }
}
