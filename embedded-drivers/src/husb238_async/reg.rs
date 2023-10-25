use bondrewd::*;
use paste::paste;

pub(super) const REG_SIZE: usize = 1;

pub trait Register: bondrewd::Bitfields<REG_SIZE> {
    fn address() -> RegisterAddress;
}

pub trait WritableRegister: Register + Default {}

#[repr(u8)]
pub enum RegisterAddress {
    ActivePowerSettings = 0x00,
    Status = 0x01,
    PowerDataObject5V = 0x02,
    PowerDataObject9V = 0x03,
    PowerDataObject12V = 0x04,
    PowerDataObject15V = 0x05,
    PowerDataObject18V = 0x06,
    PowerDataObject20V = 0x07,
    SelectedPowerDataObject = 0x08,
    GoCommand = 0x09,
}
impl Into<u8> for RegisterAddress {
    fn into(self) -> u8 {
        self as u8
    }
}

/// All-register reset, shunt voltage and bus voltage ADC conversion times and averaging,
/// operating mode. Address: 0x00
#[derive(Bitfields, Clone, PartialEq)]
#[bondrewd(default_endianness = "be", enforce_bytes = 1)]
#[cfg_attr(feature = "defmt", derive(::defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub struct ActivePowerSettingsRegister {
    #[bondrewd(enum_primitive = "u8", bit_length = 4)]
    pub source_voltage: SourceVoltage,
    #[bondrewd(enum_primitive = "u8", bit_length = 4)]
    pub source_current: SourceCurrent,
}
impl Register for ActivePowerSettingsRegister {
    fn address() -> RegisterAddress {
        RegisterAddress::ActivePowerSettings
    }
}

/// Contains the value of the current flowing through the shunt resistor
/// Address: 0x01
#[derive(Bitfields, PartialEq)]
#[bondrewd(default_endianness = "be", enforce_bytes = 1, read_only)]
#[cfg_attr(feature = "defmt", derive(::defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub struct StatusRegister {
    #[bondrewd(bit_length = 1, enum_primitive = "u8")]
    pub cc_dir: ConfigChannelDirection,
    #[bondrewd(bit_length = 1)]
    pub attached: bool,
    #[bondrewd(bit_length = 3, enum_primitive = "u8")]
    pub pd_response: PdResponse,
    #[bondrewd(bit_length = 1)]
    pub is_5v: bool,
    #[bondrewd(bit_length = 2, enum_primitive = "u8")]
    pub current_5v: Source5vCurrent,
}
impl Register for StatusRegister {
    fn address() -> RegisterAddress {
        RegisterAddress::Status
    }
}

macro_rules! pdo_register {
    ($v: literal) => {
        paste!(
            /// Information about the voltage's availability in the source
            #[derive(Bitfields, PartialEq)]
            #[bondrewd(default_endianness = "be", enforce_bytes = 1, read_only)]
            #[cfg_attr(feature = "defmt", derive(::defmt::Format))]
            #[cfg_attr(test, derive(Debug))]
            pub struct [<Pdo $v VRegister>] {
                #[bondrewd(bit_length = 1)]
                pub detected: bool,
                #[bondrewd(bit_length = 3, reserve)]
                reserved: u8,
                #[bondrewd(enum_primitive = "u8", bit_length = 4)]
                pub current: SourceCurrent,
            }
            impl Register for [<Pdo $v VRegister>] {
                fn address() -> RegisterAddress {
                    RegisterAddress:: [<PowerDataObject $v V>]
                }
            }
        );
    };
}

pdo_register!(5);
pdo_register!(9);
pdo_register!(12);
pdo_register!(15);
pdo_register!(18);
pdo_register!(20);

/// Contains the value of the calculated power being delivered to the load
/// Address: 0x03
#[derive(Bitfields, PartialEq)]
#[bondrewd(default_endianness = "be", enforce_bytes = 1)]
#[cfg_attr(feature = "defmt", derive(::defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub struct SelectedPowerDataObjectRegister {
    #[bondrewd(enum_primitive = "u8", bit_length = 4)]
    pub voltage: SourceVoltage,
    #[bondrewd(bit_length = 4, reserve)]
    reserved: u8,
}
impl Register for SelectedPowerDataObjectRegister {
    fn address() -> RegisterAddress {
        RegisterAddress::SelectedPowerDataObject
    }
}
impl WritableRegister for SelectedPowerDataObjectRegister {}
impl Default for SelectedPowerDataObjectRegister {
    fn default() -> Self {
        Self::from_bytes([0b00000000])
    }
}

/// Alert configuration and Conversion Ready flag
/// Address: 0x06
#[derive(Bitfields, PartialEq)]
#[bondrewd(default_endianness = "be", enforce_bytes = 1)]
#[cfg_attr(feature = "defmt", derive(::defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub struct GoCommandRegister {
    #[bondrewd(bit_length = 3, reserve)]
    reserved: u8,
    #[bondrewd(enum_primitive = "u8", bit_length = 5)]
    pub command_function: CommandFunction,
}
impl Register for GoCommandRegister {
    fn address() -> RegisterAddress {
        RegisterAddress::GoCommand
    }
}
impl WritableRegister for GoCommandRegister {}
impl Default for GoCommandRegister {
    fn default() -> Self {
        Self::from_bytes([0b00000000])
    }
}

#[derive(Eq, PartialEq, Clone, BitfieldEnum)]
#[bondrewd_enum(u8, bit_length = 3)]
#[cfg_attr(feature = "defmt", derive(::defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub enum PdResponse {
    NoResponse = 0b000,
    Success = 0b001,
    InvalidCommandOrArgument = 0b011,
    CommandNotSupported = 0b100,
    TransactionFailed = 0b101,
    Invalid,
}

#[derive(Eq, PartialEq, Clone, BitfieldEnum)]
#[bondrewd_enum(u8, bit_length = 1)]
#[cfg_attr(feature = "defmt", derive(::defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub enum ConfigChannelDirection {
    Cc1Attached = 0,
    Cc2Attached = 1,
}

#[derive(Eq, PartialEq, Clone, BitfieldEnum)]
#[bondrewd_enum(u8)]
#[cfg_attr(feature = "defmt", derive(::defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub enum CommandFunction {
    PdoSelectRequest = 0b00001,
    GetSourceCapabilities = 0b00100,
    HardReset = 0b10000,
    Invalid,
}

#[derive(Eq, PartialEq, Clone, BitfieldEnum)]
#[bondrewd_enum(u8)]
#[cfg_attr(feature = "defmt", derive(::defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub enum SourceVoltage {
    Unattached = 0x0,
    Pd5V = 0b0001,
    Pd9V = 0b0010,
    Pd12V = 0b0011,
    Pd15V = 0b0100,
    Pd18V = 0b0101,
    Pd20V = 0b0110,
    Invalid,
}

#[derive(Eq, PartialEq, Clone, BitfieldEnum)]
#[bondrewd_enum(u8)]
#[cfg_attr(feature = "defmt", derive(::defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub enum SourceCurrent {
    Pd0_5A = 0b0000,
    Pd0_7A = 0b0001,
    Pd1A = 0b0010,
    Pd1_25A = 0b0011,
    Pd1_5A = 0b0100,
    Pd1_75A = 0b0101,
    Pd2A = 0b0110,
    Pd2_25A = 0b0111,
    Pd2_5A = 0b1000,
    Pd2_75A = 0b1001,
    Pd3A = 0b1010,
    Pd3_25A = 0b1011,
    Pd3_5A = 0b1100,
    Pd4A = 0b1101,
    Pd4_5A = 0b1110,
    Pd5A = 0b1111,
    Invalid,
}

#[derive(Eq, PartialEq, Clone, BitfieldEnum)]
#[bondrewd_enum(u8, bit_length = 2)]
#[cfg_attr(feature = "defmt", derive(::defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub enum Source5vCurrent {
    PdDefaultCurrent = 0b00,
    Pd1_5A = 0b01,
    Pd2_4A = 0b10,
    Pd3A = 0b11,
}

#[cfg(test)]
mod test {}
