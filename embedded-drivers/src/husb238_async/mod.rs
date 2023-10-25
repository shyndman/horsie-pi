//! Asynchronous I2C Driver for the Hynetek HUSB238 current and power sensor

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
    pub async fn new(bus: BUS) -> Result<Self, Error<E>> {
        let mut device = Self {
            address: DEFAULT_I2C_ADDRESS,
            bus,
        };
        device.get_5v_contract_voltage().await?;
        Ok(device)
    }

    pub async fn is_attached(&mut self) -> Result<bool, Error<E>> {
        Ok(self.read_register::<StatusRegister>().await?.attached)
    }
    pub async fn cc_direction(&mut self) -> Result<ConfigChannelDirection, Error<E>> {
        Ok(self.read_register::<StatusRegister>().await?.cc_dir)
    }
    pub async fn pd_response(&mut self) -> Result<PdResponse, Error<E>> {
        Ok(self.read_register::<StatusRegister>().await?.pd_response)
    }
    pub async fn get_5v_contract_voltage(&mut self) -> Result<bool, Error<E>> {
        Ok(self.read_register::<StatusRegister>().await?.is_5v)
    }

    pub async fn get_5v_contract_current(&mut self) -> Result<Source5vCurrent, Error<E>> {
        Ok(self.read_register::<StatusRegister>().await?.current_5v)
    }

    pub async fn is_voltage_detected(&mut self, pd: SourceVoltage) -> Result<bool, Error<E>> {
        Ok(match pd {
            SourceVoltage::Pd5V => self.read_register::<Pdo5VRegister>().await?.detected,
            SourceVoltage::Pd9V => self.read_register::<Pdo9VRegister>().await?.detected,
            SourceVoltage::Pd12V => self.read_register::<Pdo12VRegister>().await?.detected,
            SourceVoltage::Pd15V => self.read_register::<Pdo15VRegister>().await?.detected,
            SourceVoltage::Pd18V => self.read_register::<Pdo18VRegister>().await?.detected,
            SourceVoltage::Pd20V => self.read_register::<Pdo20VRegister>().await?.detected,
            _ => return Err(Error::InvalidPowerData),
        })
    }

    pub async fn current_detected(
        &mut self,
        pd: SourceVoltage,
    ) -> Result<SourceCurrent, Error<E>> {
        Ok(match pd {
            SourceVoltage::Pd5V => self.read_register::<Pdo5VRegister>().await?.current,
            SourceVoltage::Pd9V => self.read_register::<Pdo9VRegister>().await?.current,
            SourceVoltage::Pd12V => self.read_register::<Pdo12VRegister>().await?.current,
            SourceVoltage::Pd15V => self.read_register::<Pdo15VRegister>().await?.current,
            SourceVoltage::Pd18V => self.read_register::<Pdo18VRegister>().await?.current,
            SourceVoltage::Pd20V => self.read_register::<Pdo20VRegister>().await?.current,
            _ => return Err(Error::InvalidPowerData),
        })
    }

    pub async fn active_power_settings(
        &mut self,
    ) -> Result<(SourceVoltage, SourceCurrent), Error<E>> {
        let reg = self.read_register::<ActivePowerSettingsRegister>().await?;
        Ok((reg.source_voltage, reg.source_current))
    }

    pub async fn selected_pd(&mut self) -> Result<SourceVoltage, Error<E>> {
        Ok(self
            .read_register::<SelectedPowerDataObjectRegister>()
            .await?
            .voltage)
    }

    pub async fn select_pd(&mut self, pd: SourceVoltage) -> Result<(), Error<E>> {
        Ok(self
            .update_register::<SelectedPowerDataObjectRegister>(|mut reg| {
                reg.voltage = pd;
                reg
            })
            .await?)
    }

    pub async fn reset(&mut self) -> Result<(), Error<E>> {
        Ok(self
            .update_register::<GoCommandRegister>(|mut reg| {
                reg.command_function = CommandFunction::HardReset;
                reg
            })
            .await?)
    }

    pub async fn request_pd(&mut self) -> Result<(), Error<E>> {
        Ok(self
            .update_register::<GoCommandRegister>(|mut reg| {
                reg.command_function = CommandFunction::PdoSelectRequest;
                reg
            })
            .await?)
    }

    pub async fn get_source_capabilities(&mut self) -> Result<(), Error<E>> {
        Ok(self
            .update_register::<GoCommandRegister>(|mut reg| {
                reg.command_function = CommandFunction::GetSourceCapabilities;
                reg
            })
            .await?)
    }

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
