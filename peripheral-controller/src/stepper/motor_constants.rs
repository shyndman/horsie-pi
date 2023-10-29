use core::f32::consts::PI;

use micromath::F32Ext;

// use micromath::F32Ext;

pub const TMC2209_VSENSE_OHMS: f32 = 0.11;

/// NEMA 8: S20STH30-0604A
pub const NEMA8_S20STH30_0604A_CONSTANTS: MotorConstants = MotorConstants {
    max_current_amps: 0.6,
    run_current_amps: 0.4,
    holding_torque_nm: 0.01765197,
    phase_inductance_henries: 0.0017,
    phase_resistance_ohms: 6.5,
    steps_per_rotation: 200,
    rated_volts: 3.9,
};

/// NEMA 11: 11HS18-0674S
pub const NEMA11_11HS18_0674S_CONSTANTS: MotorConstants = MotorConstants {
    max_current_amps: 0.67,
    run_current_amps: 0.5,
    holding_torque_nm: 0.1,
    phase_inductance_henries: 0.006,
    phase_resistance_ohms: 6.8,
    steps_per_rotation: 200,
    rated_volts: 4.6,
};

pub struct MotorConstants {
    pub run_current_amps: f32,
    pub max_current_amps: f32,
    pub phase_resistance_ohms: f32,
    pub phase_inductance_henries: f32,
    pub holding_torque_nm: f32,
    pub steps_per_rotation: u32,
    pub rated_volts: f32,
}

impl MotorConstants {
    pub fn run_current_milliamps(&self) -> f32 {
        self.run_current_amps * 1000.0
    }

    /// Volts per radian/second
    pub fn back_emf(&self) -> f32 {
        self.holding_torque_nm / (2.0 * self.max_current_amps)
    }

    pub fn pwm_gradient(&self, fclk: Option<f32>, steps: Option<u32>) -> f32 {
        let fclk = fclk.unwrap_or(tmc2209::INTERNAL_CLOCK_HZ);
        let steps = steps.unwrap_or(self.steps_per_rotation) as f32;
        self.back_emf() * 2.0 * PI * fclk * 1.46 / (self.rated_volts * 256.0 * steps)
    }

    pub fn pwm_output_frequency(&self, current: Option<f32>) -> f32 {
        let current = current.unwrap_or(self.run_current_amps);
        (374.0 * self.phase_resistance_ohms * current / self.rated_volts).ceil()
    }

    pub fn hysteresis(
        &self,
        fclk: Option<f32>,
        current: Option<f32>,
        tbl: Option<u8>,
        toff: Option<u8>,
    ) -> (u32, u32) {
        let fclk = fclk.unwrap_or(tmc2209::INTERNAL_CLOCK_HZ);
        let current = current.unwrap_or(self.max_current_amps);
        let tbl = tbl.unwrap_or(1u8);
        let toff = toff.unwrap_or(3u8);

        let tblank = 16.0 * (1.5_f32.powi(tbl as i32)) / fclk;
        let tsd = (12.0 + 32.0 * toff as f32) / fclk;
        let dcoilblank = self.rated_volts * tblank / self.phase_inductance_henries;
        let dcoilsd =
            self.phase_resistance_ohms * current * 2.0 * tsd / self.phase_inductance_henries;
        let hysteresis =
            (0.5 + ((dcoilblank + dcoilsd) * 2.0 * 248.0 * 32.0 / current) / 32.0 - 8.0)
                .max(-2.0)
                .ceil() as u32;
        let htotal = hysteresis.min(14);
        let hstrt = htotal.min(8).max(1);
        let hend = (htotal - hstrt).min(12);

        (hstrt - 1, hend + 3)
    }
}
