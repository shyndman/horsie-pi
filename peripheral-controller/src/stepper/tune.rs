use embassy_sync::blocking_mutex::raw::RawMutex;
use micromath::F32Ext;

use super::{
    motor_constants::{MotorConstants, TMC2209_VSENSE_OHMS},
    uart::Tmc2209UartConnection,
};

pub enum TuningProfile {
    Silent,
    Performance,
}

const IHOLD_DELAY: u8 = 12;

const SEMIN: u16 = 8;
const SEMAX: u16 = 4;
const SEUP: u16 = 3;
const SEDN: u16 = 0;
const SEIMIN: bool = false; // If we drop to 1/4 current, high accels don't work right

pub async fn tune_driver<M: RawMutex, P: esp_hal_common::uart::Instance>(
    driver: &mut Tmc2209UartConnection<M, P>,
    motor_constants: MotorConstants,
) {
    // Disable automatic power down, because its pin is shared
    let mut gconf = tmc2209::reg::GCONF::default();
    gconf.set_pdn_disable(true);
    driver.write_register(gconf).await.unwrap();

    let mut ihold_irun = tmc2209::reg::IHOLD_IRUN::default();
    let (vsense, irun) = tmc2209::rms_current_to_vsense_cs(
        TMC2209_VSENSE_OHMS,
        motor_constants.run_current_milliamps(),
    );
    let ihold = (irun - 8).max(0);
    defmt::info!("Setting IRUN={}, IHOLD={}", irun, ihold);
    ihold_irun.set_irun(irun);
    ihold_irun.set_ihold(ihold);
    ihold_irun.set_ihold_delay(IHOLD_DELAY);
    driver.write_register(ihold_irun).await.unwrap();

    let mut tpowerdown = tmc2209::reg::TPOWERDOWN::default();
    tpowerdown.0 = 20;
    driver.write_register(tpowerdown).await.unwrap();

    let mut chopconf = tmc2209::reg::CHOPCONF::default();
    let (hstrt, hend) = motor_constants.hysteresis(None, None, None, None);
    chopconf.set_hstrt(hstrt);
    chopconf.set_hstrt(hend);
    chopconf.set_mres(0b00); // 256 steps
    chopconf.set_intpol(true); // Interpolation of microsteps to 256
    chopconf.set_vsense(vsense);
    chopconf.set_tbl(1);
    chopconf.set_toff(((0.85e-5 * tmc2209::INTERNAL_CLOCK_HZ - 12.0) / 32.0).ceil() as u32);
    chopconf.set_dedge(true);
    driver.write_register(chopconf).await.unwrap();

    // Stallguard
    let mut tpwnthrs = tmc2209::reg::TPWMTHRS::default();
    tpwnthrs.set(0xfffff); // This keeps the stepper in SpreadCycle mode
    driver.write_register(tpwnthrs).await.unwrap();

    // Coolstep
    let mut tcoolthrs = tmc2209::reg::TCOOLTHRS::default();
    tcoolthrs.set(293);
    driver.write_register(tcoolthrs).await.unwrap();

    let mut sgthrs = tmc2209::reg::SGTHRS::default();
    sgthrs.0 = 10;
    driver.write_register(sgthrs).await.unwrap();

    let mut coolconf = tmc2209::reg::COOLCONF::default();
    coolconf.set_semin(SEMIN);
    coolconf.set_semax(SEMAX);
    coolconf.set_seup(SEUP);
    coolconf.set_sedn(SEDN);
    coolconf.set_seimin(SEIMIN);
    driver.write_register(coolconf).await.unwrap();

    gconf.set_en_spread_cycle(true); // SpreadCycle
    gconf.set_i_scale_analog(false);
    gconf.set_multistep_filt(true);
    gconf.set_mstep_reg_select(true); // Set microsteps via registers
    driver.write_register(gconf).await.unwrap();
}
