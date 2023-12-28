//! Asynchronous I2C Driver for the TI INA260 current and power sensor

pub mod error;
pub mod reg;

use core::fmt::Debug;

use embedded_hal_async::i2c::I2c;

use self::{error::Error, reg::*};

const DEFAULT_I2C_ADDRESS: u8 = 0x40;

pub struct Ina260<BUS> {
    address: u8,
    bus: BUS,
}

impl<BUS, E> Ina260<BUS>
where
    BUS: I2c<Error = E>,
    E: Into<Error<E>> + Debug,
{
    /// Attempts to initialize a new INA260 on the I2C bus with an address of 0x40.
    pub async fn new(bus: BUS) -> Result<Self, Error<E>> {
        Self::new_with_address(bus, DEFAULT_I2C_ADDRESS).await
    }

    pub async fn new_with_address(mut bus: BUS, address: u8) -> Result<Self, Error<E>> {
        let id_reg = Self::_read_register::<DieIdRegister>(address, &mut bus).await?;
        let manufacturer_reg =
            Self::_read_register::<ManufacturerIdRegister>(address, &mut bus).await?;

        if manufacturer_reg.id != 0x5449 || id_reg.device_id != 0x227 {
            return Err(Error::ImposterAtExpectedAddress);
        }

        Ok(Self {
            address: address,
            bus,
        })
    }

    /// Resets the hardware. All registers are set to default values.
    pub async fn reset(&mut self) -> Result<(), Error<E>> {
        self.update_register::<ConfigurationRegister>(|mut reg| {
            reg.reset = true;
            reg
        })
        .await
    }

    /// Requests the current power (in mA) measured by the sensor
    ///
    /// If averaging is enabled, this register reports the averaged value.
    pub async fn read_current(&mut self) -> Result<f32, Error<E>> {
        let reg = self.read_register::<CurrentRegister>().await?;
        let sign = if reg.negative { -1.0 } else { 1.0 };
        Ok(sign * reg.current as f32 * 1.25)
    }

    /// Requests the current power (in mV) measured by the sensor
    ///
    /// If averaging is enabled, this register reports the averaged value.
    pub async fn read_bus_voltage(&mut self) -> Result<f32, Error<E>> {
        let reg = self.read_register::<BusVoltageRegister>().await?;
        let sign = if reg.negative { -1.0 } else { 1.0 };
        Ok(sign * reg.voltage as f32 * 1.25)
    }

    /// Requests the current power (in mW) measured by the sensor
    ///
    /// If averaging is enabled, this register reports the averaged value.
    pub async fn read_power(&mut self) -> Result<u32, Error<E>> {
        let reg = self.read_register::<PowerRegister>().await?;
        Ok(reg.centiwatts as u32 * 10)
    }

    pub async fn measurement_mode(&mut self) -> Result<MeasurementOperatingMode, Error<E>> {
        Ok(self
            .read_register::<ConfigurationRegister>()
            .await?
            .measurements_operating_mode)
    }

    pub async fn set_measurement_mode(
        &mut self,
        mode: MeasurementOperatingMode,
    ) -> Result<(), Error<E>> {
        self.update_register::<ConfigurationRegister>(|mut reg| {
            reg.measurements_operating_mode = mode;
            reg
        })
        .await
    }

    pub async fn is_conversion_ready(&mut self) -> Result<bool, Error<E>> {
        Ok(self
            .read_register::<MaskEnableRegister>()
            .await?
            .conversion_ready_flag)
    }

    pub async fn is_alert_set(&mut self) -> Result<bool, Error<E>> {
        Ok(self
            .read_register::<MaskEnableRegister>()
            .await?
            .alert_fn_flag)
    }

    pub async fn alert_limit(&mut self) -> Result<f32, Error<E>> {
        Ok(self
            .read_register::<AlertLimitRegister>()
            .await?
            .alert_limit as f32 *
            1.25)
    }
    pub async fn set_alert_limit(&mut self, limit: f32) -> Result<(), Error<E>> {
        self.update_register::<AlertLimitRegister>(|mut reg| {
            // TODO(shyndman): Indicate out of range values
            reg.alert_limit = (limit / 1.25) as u16;
            reg
        })
        .await
    }

    pub async fn alert_latch(&mut self) -> Result<AlertLatch, Error<E>> {
        Ok(self
            .read_register::<MaskEnableRegister>()
            .await?
            .alert_latch_enable)
    }
    pub async fn set_alert_latch(&mut self, latch: AlertLatch) -> Result<(), Error<E>> {
        self.update_register::<MaskEnableRegister>(|mut reg| {
            reg.alert_latch_enable = latch;
            reg
        })
        .await
    }

    pub async fn alert_polarity(&mut self) -> Result<AlertPolarity, Error<E>> {
        Ok(self
            .read_register::<MaskEnableRegister>()
            .await?
            .alert_polarity)
    }
    pub async fn set_alert_polarity(
        &mut self,
        polarity: AlertPolarity,
    ) -> Result<(), Error<E>> {
        self.update_register::<MaskEnableRegister>(|mut reg| {
            reg.alert_polarity = polarity;
            reg
        })
        .await
    }

    pub async fn alert_type(&mut self) -> Result<AlertType, Error<E>> {
        Ok(self.read_register::<MaskEnableRegister>().await?.alert_type)
    }
    pub async fn set_alert_type(&mut self, alert_type: AlertType) -> Result<(), Error<E>> {
        self.update_register::<MaskEnableRegister>(|mut reg| {
            reg.alert_type = alert_type;
            reg
        })
        .await
    }

    pub async fn current_conversion_time(&mut self) -> Result<ConversionTime, Error<E>> {
        Ok(self
            .read_register::<ConfigurationRegister>()
            .await?
            .shunt_current_conversion_time)
    }
    pub async fn set_current_conversion_time(
        &mut self,
        time: ConversionTime,
    ) -> Result<(), Error<E>> {
        self.update_register::<ConfigurationRegister>(|mut reg| {
            reg.shunt_current_conversion_time = time;
            reg
        })
        .await
    }

    pub async fn voltage_conversion_time(&mut self) -> Result<ConversionTime, Error<E>> {
        Ok(self
            .read_register::<ConfigurationRegister>()
            .await?
            .bus_voltage_conversion_time)
    }
    pub async fn set_voltage_conversion_time(
        &mut self,
        time: ConversionTime,
    ) -> Result<(), Error<E>> {
        self.update_register::<ConfigurationRegister>(|mut reg| {
            reg.bus_voltage_conversion_time = time;
            reg
        })
        .await
    }

    pub async fn averaging_window_size(&mut self) -> Result<AveragingWindowSize, Error<E>> {
        Ok(self
            .read_register::<ConfigurationRegister>()
            .await?
            .averaging_window_size)
    }
    pub async fn set_averaging_window_size(
        &mut self,
        count: AveragingWindowSize,
    ) -> Result<(), Error<E>> {
        self.update_register::<ConfigurationRegister>(|mut reg| {
            reg.averaging_window_size = count;
            reg
        })
        .await
    }

    pub async fn read_register<R: Register>(&mut self) -> Result<R, Error<E>> {
        Self::_read_register(self.address, &mut self.bus).await
    }

    pub async fn write_register<R: WritableRegister>(
        &mut self,
        register: R,
    ) -> Result<(), Error<E>> {
        let reg_bytes = register.into_bytes();
        let to_write: [u8; 3] = [R::address().into(), reg_bytes[0], reg_bytes[1]];
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

    async fn _read_register<R: Register>(address: u8, bus: &mut BUS) -> Result<R, Error<E>> {
        let mut buf: [u8; REG_SIZE] = [0; REG_SIZE];
        if let Err(e) = bus
            .write_read(address, &[R::address() as u8], &mut buf)
            .await
        {
            return Err(Error::BusError(e));
        }

        Ok(R::from_bytes(buf))
    }
}
