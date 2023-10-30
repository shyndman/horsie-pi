use core::cell::RefCell;

use embassy_sync::{blocking_mutex::raw::RawMutex, mutex::Mutex};
use embedded_hal_1::i2c::{I2c, Operation};
use embedded_hal_async::i2c;

/// I2C device on a shared bus.
pub struct DualModeI2cDevice<'a, M: RawMutex, BUS> {
    bus: &'a Mutex<M, RefCell<BUS>>,
}

impl<'a, M: RawMutex, BUS> DualModeI2cDevice<'a, M, BUS> {
    /// Create a new `I2cDevice`.
    pub fn new(bus: &'a Mutex<M, RefCell<BUS>>) -> Self {
        Self { bus }
    }
}

// Asynchronous implementation

impl<'a, M: RawMutex, BUS> i2c::ErrorType for DualModeI2cDevice<'a, M, BUS>
where
    BUS: i2c::ErrorType,
{
    type Error = esp32s3_hal::i2c::Error;
}

impl<M, BUS> i2c::I2c for DualModeI2cDevice<'_, M, BUS>
where
    M: RawMutex + 'static,
    BUS: i2c::I2c + 'static,
{
    async fn read(
        &mut self,
        address: u8,
        read: &mut [u8],
    ) -> Result<(), esp32s3_hal::i2c::Error> {
        self.bus
            .lock()
            .await
            .borrow_mut()
            .read(address, read)
            .await
            .map_err(|_| esp32s3_hal::i2c::Error::ExecIncomplete)?;
        Ok(())
    }

    async fn write(
        &mut self,
        address: u8,
        write: &[u8],
    ) -> Result<(), esp32s3_hal::i2c::Error> {
        self.bus
            .lock()
            .await
            .borrow_mut()
            .write(address, write)
            .await
            .map_err(|_| esp32s3_hal::i2c::Error::ExecIncomplete)?;
        Ok(())
    }

    async fn write_read(
        &mut self,
        address: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), esp32s3_hal::i2c::Error> {
        self.bus
            .lock()
            .await
            .borrow_mut()
            .write_read(address, write, read)
            .await
            .map_err(|_| esp32s3_hal::i2c::Error::ExecIncomplete)?;
        Ok(())
    }

    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal_async::i2c::Operation<'_>],
    ) -> Result<(), esp32s3_hal::i2c::Error> {
        self.bus
            .lock()
            .await
            .borrow_mut()
            .transaction(address, operations)
            .await
            .map_err(|_| esp32s3_hal::i2c::Error::ExecIncomplete)?;
        Ok(())
    }
}

// Synchronous implementation

impl<M, BUS> I2c for DualModeI2cDevice<'_, M, BUS>
where
    M: RawMutex,
    BUS: I2c,
{
    fn read(
        &mut self,
        address: u8,
        buffer: &mut [u8],
    ) -> Result<(), esp32s3_hal::i2c::Error> {
        self.bus
            .try_lock()
            .map_err(|_| esp32s3_hal::i2c::Error::AckCheckFailed)?
            .borrow_mut()
            .read(address, buffer)
            .map_err(|_| esp32s3_hal::i2c::Error::AckCheckFailed)
    }

    fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), esp32s3_hal::i2c::Error> {
        self.bus
            .try_lock()
            .map_err(|_| esp32s3_hal::i2c::Error::AckCheckFailed)?
            .borrow_mut()
            .write(address, bytes)
            .map_err(|_| esp32s3_hal::i2c::Error::AckCheckFailed)
    }

    fn write_read(
        &mut self,
        address: u8,
        wr_buffer: &[u8],
        rd_buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.bus
            .try_lock()
            .map_err(|_| esp32s3_hal::i2c::Error::AckCheckFailed)?
            .borrow_mut()
            .write_read(address, wr_buffer, rd_buffer)
            .map_err(|_| esp32s3_hal::i2c::Error::AckCheckFailed)
    }

    fn transaction<'a>(
        &mut self,
        address: u8,
        operations: &mut [Operation<'a>],
    ) -> Result<(), Self::Error> {
        let _ = address;
        let _ = operations;
        self.bus
            .try_lock()
            .map_err(|_| esp32s3_hal::i2c::Error::AckCheckFailed)?
            .borrow_mut()
            .transaction(address, operations)
            .map_err(|_| esp32s3_hal::i2c::Error::AckCheckFailed)
    }
}
