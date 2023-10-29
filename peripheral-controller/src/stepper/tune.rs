use defmt::unwrap;
use esp32s3_hal::{uart, Uart};

use super::{
    motor_constants::{MotorConstants, TMC2209_VSENSE_OHMS},
    uart::Tmc2209UartConnection,
};

pub async fn tune_driver<'d, P: uart::Instance>(
    driver: &mut Tmc2209UartConnection,
    motor_constants: MotorConstants,
    uart: &mut Uart<'static, P>,
) {
    // Disable automatic power down, because its pin is shared
    let mut gconf = tmc2209::reg::GCONF::default();
    gconf.set_pdn_disable(true);
    unwrap!(driver.write_register(uart, gconf).await);

    let mut ihold_irun = tmc2209::reg::IHOLD_IRUN::default();
    let (vsense, irun) = tmc2209::rms_current_to_vsense_cs(
        TMC2209_VSENSE_OHMS,
        motor_constants.run_current_milliamps(),
    );
    ihold_irun.set_irun(irun);
    ihold_irun.set_ihold((irun - 8).max(0));
    unwrap!(driver.write_register(uart, ihold_irun).await);

    let mut chopconf = tmc2209::reg::CHOPCONF::default();
    let (hstrt, hend) = motor_constants.hysteresis(None, Some(12.0), None, None);
    chopconf.set_hstrt(hstrt);
    chopconf.set_hstrt(hend);
    chopconf.set_mres(0b01); // 128 steps
    chopconf.set_intpol(true); // Interpolation of microsteps to 256
    chopconf.set_vsense(vsense);
    unwrap!(driver.write_register(uart, chopconf).await);

    // StealthChop configuration
    let mut pwmconf = tmc2209::reg::PWMCONF::default();
    pwmconf.set_pwm_autoscale(true);
    pwmconf.set_pwm_autograd(true);
    pwmconf.set_pwm_grad(motor_constants.pwm_gradient(None, None) as u8);
    pwmconf.set_pwm_ofs(motor_constants.pwm_output_frequency(None) as u8);
    pwmconf.set_pwm_reg(15);
    pwmconf.set_pwm_lim(4);
    unwrap!(driver.write_register(uart, pwmconf).await);

    // Stallguard
    let mut tpwnthrs = tmc2209::reg::TPWMTHRS::default();
    tpwnthrs.set(0); // Disable switching between StealthChop and SpreadCycle
    unwrap!(driver.write_register(uart, tpwnthrs).await);

    // Coolstep
    let mut tcoolthrs = tmc2209::reg::TCOOLTHRS::default();
    tcoolthrs.set(56);
    unwrap!(driver.write_register(uart, tcoolthrs).await);

    let mut sgthrs = tmc2209::reg::SGTHRS::default();
    sgthrs.0 = 1;
    unwrap!(driver.write_register(uart, sgthrs).await);

    gconf.set_en_spread_cycle(false); // StealthChop only
    gconf.set_mstep_reg_select(true); // Set microsteps via registers
    gconf.set_pdn_disable(false); // Re-enable PDN
    unwrap!(driver.write_register(uart, gconf).await);
}
