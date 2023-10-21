//! Asynchronous I2C Driver for the Hynetek HUSB238 current and power sensor

#![no_std]

pub mod error;
pub mod reg;

use core::fmt::Debug;

use embedded_hal_async::i2c::I2c;

use self::{error::Error, reg::*};

const DEFAULT_I2C_ADDRESS: u8 = 0x08;

pub struct Husb238<BUS> {
    address: u8,
    bus: BUS,
}
impl<BUS, E> Husb238<BUS>
where
    BUS: I2c<Error = E>,
    E: Into<Error<E>> + Debug,
{
    pub async fn new(bus: BUS) -> Self {
        Self {
            address: DEFAULT_I2C_ADDRESS,
            bus,
        }
    }

    pub async fn is_attached(&mut self) -> Result<bool, Error<E>> {
        Ok(self.read_register::<StatusRegister>().await?.attached)
    }
    pub async fn get_cc_direction(&mut self) -> Result<ConfigChannelDirection, Error<E>> {
        Ok(self.read_register::<StatusRegister>().await?.cc_dir)
    }
    pub async fn get_pd_response(&mut self) -> Result<PdResponse, Error<E>> {
        Ok(self.read_register::<StatusRegister>().await?.pd_response)
    }
    pub async fn get_5v_contract_voltage(&mut self) -> Result<bool, Error<E>> {
        Ok(self.read_register::<StatusRegister>().await?.is_5v)
    }

    // pub async fn get_5v_contract_current(&mut self) -> Result<Source5vCurrent, Error<E>> {}
    // pub async fn is_voltage_detected(&mut self, pd: SourceVoltage) -> Result<bool, Error<E>> {}
    // pub async fn current_detected(&mut self, pd: SourceVoltage) -> Result<SourceCurrent, Error<E>> {
    // }
    // pub async fn get_pd_src_voltage(&mut self) -> Result<Source5vCurrent, Error<E>> {}
    // pub async fn get_pd_src_current(&mut self) -> Result<SourceCurrent, Error<E>> {}
    // pub async fn get_selected_pd(&mut self) -> Result<SourceVoltage, Error<E>> {}

    pub async fn select_pd(&mut self, pd: SourceVoltage) {}
    pub async fn reset(&mut self) {}
    pub async fn request_pd(&mut self) {}
    pub async fn get_source_capabilities(&mut self) {}

    pub async fn read_register<R: Register>(&mut self) -> Result<R, Error<E>> {
        let mut buf: [u8; REG_SIZE] = [0; REG_SIZE];
        if let Err(e) = self
            .bus
            .write_read(self.address, &[R::address() as u8], &mut buf)
            .await
        {
            return Err(Error::BusError(e));
        }
        Ok(R::from_bytes(buf))
    }

    pub async fn write_register<R: WritableRegister>(
        &mut self,
        register: R,
    ) -> Result<(), Error<E>> {
        let reg_bytes = register.into_bytes();
        let to_write: [u8; 2] = [R::address().into(), reg_bytes[0]];
        self.bus.write(self.address, &to_write).await.unwrap();
        Ok(())
    }

    pub async fn update_register<R: WritableRegister>(
        &mut self,
        mutator: impl FnOnce(R) -> R,
    ) -> Result<(), Error<E>> {
        let reg = self.read_register::<R>().await?;
        self.write_register(mutator(reg)).await?;
        Ok(())
    }
}
