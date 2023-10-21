use bondrewd::*;

pub(super) const REG_SIZE: usize = 2;

pub trait Register: bondrewd::Bitfields<REG_SIZE> {
    fn address() -> RegisterAddress;
}

pub trait WritableRegister: Register + Default {}

#[repr(u8)]
pub enum RegisterAddress {
    Config = 0x00,
    Current = 0x01,
    BusVoltage = 0x02,
    Power = 0x03,
    MaskEnable = 0x06,
    AlertLimit = 0x07,
    ManufacturerId = 0xFE,
    DieId = 0xFF,
}
impl Into<u8> for RegisterAddress {
    fn into(self) -> u8 {
        self as u8
    }
}

/// All-register reset, shunt voltage and bus voltage ADC conversion times and averaging,
/// operating mode. Address: 0x00
#[derive(Bitfields, Clone, PartialEq)]
#[bondrewd(default_endianness = "be", enforce_bytes = 2)]
#[cfg_attr(feature = "defmt", derive(::defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub struct ConfigurationRegister {
    pub reset: bool,
    #[bondrewd(bit_length = 3, reserve)]
    reserve: u8,
    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    pub averaging_window_size: AveragingWindowSize,
    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    pub bus_voltage_conversion_time: ConversionTime,
    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    pub shunt_current_conversion_time: ConversionTime,
    #[bondrewd(struct_size = 1, bit_length = 3)]
    pub measurements_operating_mode: MeasurementOperatingMode,
}
impl Register for ConfigurationRegister {
    fn address() -> RegisterAddress {
        RegisterAddress::Config
    }
}
impl WritableRegister for ConfigurationRegister {}
impl Default for ConfigurationRegister {
    fn default() -> Self {
        Self::from_bytes([0b01100001, 0b00100111])
    }
}

/// Contains the value of the current flowing through the shunt resistor
/// Address: 0x01
#[derive(Bitfields, PartialEq)]
#[bondrewd(default_endianness = "be", enforce_bytes = 2, read_only)]
#[cfg_attr(feature = "defmt", derive(::defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub struct CurrentRegister {
    pub negative: bool,
    #[bondrewd(bit_length = 15)]
    pub current: u16,
}
impl Register for CurrentRegister {
    fn address() -> RegisterAddress {
        RegisterAddress::Current
    }
}

/// Bus voltage measurement data
/// Address: 0x02
#[derive(Bitfields, PartialEq)]
#[bondrewd(default_endianness = "be", enforce_bytes = 2, read_only)]
#[cfg_attr(feature = "defmt", derive(::defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub struct BusVoltageRegister {
    #[bondrewd(bit_length = 1, reserve)]
    pub negative: bool,
    #[bondrewd(bit_length = 15)]
    pub voltage: u16,
}
impl Register for BusVoltageRegister {
    fn address() -> RegisterAddress {
        RegisterAddress::BusVoltage
    }
}

/// Contains the value of the calculated power being delivered to the load
/// Address: 0x03
#[derive(Bitfields, PartialEq)]
#[bondrewd(default_endianness = "be", enforce_bytes = 2, read_only)]
#[cfg_attr(feature = "defmt", derive(::defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub struct PowerRegister {
    pub centiwatts: u16,
}
impl Register for PowerRegister {
    fn address() -> RegisterAddress {
        RegisterAddress::Power
    }
}

/// Alert configuration and Conversion Ready flag
/// Address: 0x06
#[derive(Bitfields, PartialEq)]
#[bondrewd(default_endianness = "be", enforce_bytes = 2)]
#[cfg_attr(feature = "defmt", derive(::defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub struct MaskEnableRegister {
    /// Configures the trigger conditions of the alert
    #[bondrewd(struct_size = 1, bit_length = 6)]
    pub alert_type: AlertType,

    #[bondrewd(bit_length = 5, reserve)]
    reserve: u8,

    /// While only one Alert Function can be monitored at the ALERT pin at a time, the
    /// Conversion Ready can also be enabled to assert the ALERT pin. Reading the Alert
    /// Function Flag following an alert allows the user to determine if the Alert
    /// Function was the source of the Alert.
    ///
    /// When the Alert Latch Enable bit is set to Latch mode, the Alert Function Flag bit
    /// clears only when the Mask/Enable Register is read. When the Alert Latch Enable bit
    /// is set to Transparent mode, the Alert Function Flag bit is cleared following the
    /// next conversion that does not result in an Alert condition.
    #[bondrewd(read_only)]
    pub alert_fn_flag: bool,
    /// Although the device can be read at any time, and the data from the last conversion
    /// is available, the Conversion Ready Flag bit is provided to help coordinate
    /// one-shot or triggered conversions. The Conversion Ready Flag bit is set after all
    /// conversions, averaging, and multiplications are complete. Conversion Ready Flag
    /// bit clears under the following conditions:
    ///
    /// 1.) Writing to the Configuration Register (except for Power-Down selection)
    /// 2.) Reading the Mask/Enable Register
    #[bondrewd(read_only)]
    pub conversion_ready_flag: bool,
    /// This bit is set to '1' if an arithmetic operation resulted in an overflow error.
    /// It indicates that power data may have exceeded the maximum reportable value of
    /// 419.43 W.
    #[bondrewd(read_only)]
    pub math_overflow_flag: bool,
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    pub alert_polarity: AlertPolarity,
    /// When the Alert Latch Enable bit is set to Transparent mode, the ALERT pin and Flag
    /// bit resets to the idle states when the fault has been cleared. When the Alert
    /// Latch Enable bit is set to Latch mode, the ALERT pin and Alert Flag bit remains
    /// active following a fault until the Mask/Enable Register has been read.
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    pub alert_latch_enable: AlertLatch,
}
impl Register for MaskEnableRegister {
    fn address() -> RegisterAddress {
        RegisterAddress::MaskEnable
    }
}
impl WritableRegister for MaskEnableRegister {}
impl Default for MaskEnableRegister {
    fn default() -> Self {
        Self::from_bytes([0b00000000, 0b00000000])
    }
}

/// Contains the limit value to compare to the selected Alert function
/// Address: 0x05
#[derive(Bitfields, PartialEq)]
#[bondrewd(default_endianness = "be", enforce_bytes = 2)]
#[cfg_attr(feature = "defmt", derive(::defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub struct AlertLimitRegister {
    pub alert_limit: u16,
}
impl Register for AlertLimitRegister {
    fn address() -> RegisterAddress {
        RegisterAddress::AlertLimit
    }
}
impl WritableRegister for AlertLimitRegister {}
impl Default for AlertLimitRegister {
    fn default() -> Self {
        Self::from_bytes([0b00000000, 0b00000000])
    }
}

/// Contains unique manufacturer identification number
/// Address: 0xFE
#[derive(Bitfields, PartialEq)]
#[bondrewd(default_endianness = "be", enforce_bytes = 2, read_only)]
#[cfg_attr(feature = "defmt", derive(::defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub struct ManufacturerIdRegister {
    pub id: u16,
}
impl Register for ManufacturerIdRegister {
    fn address() -> RegisterAddress {
        RegisterAddress::ManufacturerId
    }
}

/// Contains unique die identification number
/// Address: 0xFF
#[derive(Bitfields, PartialEq)]
#[bondrewd(default_endianness = "be", enforce_bytes = 2, read_only)]
#[cfg_attr(feature = "defmt", derive(::defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub struct DieIdRegister {
    #[bondrewd(bit_length = 12)]
    pub device_id: u16,
    #[bondrewd(bit_length = 4)]
    pub revision: u8,
}
impl Register for DieIdRegister {
    fn address() -> RegisterAddress {
        RegisterAddress::DieId
    }
}

#[derive(Eq, PartialEq, Clone, Bitfields)]
#[bondrewd(default_endianness = "be", enforce_bits = 3)]
#[cfg_attr(feature = "defmt", derive(::defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub struct MeasurementOperatingMode {
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    measurement_type: MeasurementType,
    shunt_is_measurement_type: bool,
    bus_voltage_is_measurement_type: bool,
}

#[derive(Eq, PartialEq, Clone, BitfieldEnum)]
#[bondrewd_enum(u8)]
#[cfg_attr(feature = "defmt", derive(::defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub enum MeasurementType {
    Triggered = 0x0,
    Continuous = 0x1,
}

#[derive(Eq, PartialEq, Clone, BitfieldEnum)]
#[bondrewd_enum(u8)]
#[cfg_attr(feature = "defmt", derive(::defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub enum ConversionTime {
    Duration140us,
    Duration204us,
    Duration332us,
    Duration558us,
    Duration1110us,
    Duration2116us,
    Duration4156us,
    Duration8244us,
    Invalid,
}

#[derive(Eq, PartialEq, Clone, BitfieldEnum)]
#[bondrewd_enum(u8)]
#[cfg_attr(feature = "defmt", derive(::defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub enum AveragingWindowSize {
    Count1,
    Count4,
    Count16,
    Count64,
    Count128,
    Count256,
    Count512,
    Count1024,
    Invalid,
}

#[derive(Eq, PartialEq, Clone, Bitfields)]
#[bondrewd(default_endianness = "be", enforce_bits = 6)]
#[cfg_attr(feature = "defmt", derive(::defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub struct AlertType {
    /// Setting this bit high configures the ALERT pin to be asserted if the current
    /// measurement following a conversion exceeds the value programmed in the Alert Limit
    /// Register
    pub over_current_limit: bool,
    /// Setting this bit high configures the ALERT pin to be asserted if the current
    /// measurement following a conversion drops below the value programmed in the Alert
    /// Limit Register
    pub under_current_limit: bool,
    /// Setting this bit high configures the ALERT pin to be asserted if the bus voltage
    /// measurement following a conversion exceeds the value programmed in the Alert Limit
    /// Register
    pub bus_voltage_over_voltage: bool,
    /// Setting this bit high configures the ALERT pin to be asserted if the bus
    /// voltage measurement following a conversion drops below the value programmed in
    /// the Alert Limit Register
    pub bus_voltage_under_voltage: bool,
    /// Setting this bit high configures the ALERT pin to be asserted if the Power
    /// calculation made following a bus voltage measurement exceeds the value programmed
    /// in the Alert Limit Register
    pub power_over_limit: bool,
    /// Setting this bit high configures the ALERT pin to be asserted when the Conversion
    /// Ready Flag, Bit 3, is asserted indicating that the device is ready for the next
    /// conversion
    pub alert_conversion_ready: bool,
}

#[derive(Eq, PartialEq, Clone, BitfieldEnum)]
#[bondrewd_enum(u8)]
#[cfg_attr(feature = "defmt", derive(::defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub enum AlertPolarity {
    /// Active high open-collector (Default)
    Normal = 0x0,
    /// Active low open-collector
    Inverted = 0x1,
    Invalid,
}

#[derive(Eq, PartialEq, Clone, BitfieldEnum)]
#[bondrewd_enum(u8)]
#[cfg_attr(feature = "defmt", derive(::defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub enum AlertLatch {
    /// Alert will reset when fault is cleared
    Transparent = 0x0,
    /// Alert will latch until Mask/Enable register is read
    Enabled = 0x1,
    Invalid,
}

#[cfg(test)]
mod test {
    use bondrewd::Bitfields;

    use super::{
        AveragingWindowSize, ConfigurationRegister, ConversionTime, MeasurementOperatingMode,
        MeasurementType,
    };

    #[test]
    fn test_byte_and_bit_read_order() {
        let reg = ConfigurationRegister::from_bytes([0b01100001, 0b00110111]);
        assert_eq!(reg.reset, false);
        assert_eq!(reg.averaging_window_size, AveragingWindowSize::Count1);
        assert_eq!(
            reg.bus_voltage_conversion_time,
            ConversionTime::Duration1110us
        );
        assert_eq!(
            reg.shunt_current_conversion_time,
            ConversionTime::Duration4156us
        );
        assert_eq!(
            reg.measurements_operating_mode,
            MeasurementOperatingMode {
                measurement_type: MeasurementType::Continuous,
                bus_voltage_is_measurement_type: true,
                shunt_is_measurement_type: true,
            }
        );
    }
}
