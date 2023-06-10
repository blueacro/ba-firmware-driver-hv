#![cfg_attr(not(test), no_std)]

use num_traits::float::Float;

const SECONDS_IN_DAY: u32 = (24 * 60 * 60) as u32;

pub fn pwm_from_time(seconds_from_midnight: u32) -> f32 {
    pwm_from_time_shift(seconds_from_midnight, 0)
}

pub fn pwm_from_time_shift(seconds_from_midnight: u32, seconds_shift: i32) -> f32 {
    let shift = seconds_shift as f32 / SECONDS_IN_DAY as f32;
    let ratio =
        ((((seconds_from_midnight as f32) / (SECONDS_IN_DAY as f32)) - shift) * 2f32) - 1f32;
    let angle_of_sun = ratio * core::f32::consts::PI;
    angle_of_sun.cos().clamp(0f32, 1f32)
}

#[cfg(test)]
pub mod test {
    const SUNRISE: u32 = 6 * 60 * 60;
    const SUNSET: u32 = (6 + 12) * 60 * 60;

    use approx::assert_relative_eq;

    use super::*;
    extern crate approx;
    extern crate std;

    #[test]
    fn test_time_boundary() {
        assert!(pwm_from_time(0) == 0f32);
        assert!(pwm_from_time(SECONDS_IN_DAY - 1) == 0f32);
    }

    #[test]
    fn test_time_boundary_shift() {
        assert!(pwm_from_time_shift(0, 2 * 60 * 60) == 0f32);
        assert!(pwm_from_time_shift(SECONDS_IN_DAY - 1, 2 * 60 * 60) == 0f32);
    }

    #[test]
    fn test_midday() {
        assert_relative_eq!(pwm_from_time(12 * 60 * 60), 1f32, epsilon = 0.01f32);
    }

    #[test]
    fn test_midday_shift() {
        assert_relative_eq!(
            pwm_from_time_shift(14 * 60 * 60, 2 * 60 * 60),
            1f32,
            epsilon = 0.01f32
        );
    }

    #[test]
    fn test_sunrise_sunset() {
        assert_relative_eq!(pwm_from_time(SUNRISE + 1), 0.0f32, epsilon = 0.01f32);
        assert_relative_eq!(pwm_from_time(SUNSET - 1), 0.0f32, epsilon = 0.01f32);
    }

    #[test]
    fn test_sunrise_sunset_shift() {
        const TWO_HOURS: u32 = 2 * 60 * 60;
        assert_relative_eq!(pwm_from_time_shift(SUNRISE + 1 + TWO_HOURS, TWO_HOURS as i32), 0.0f32, epsilon = 0.01f32);
        assert_relative_eq!(pwm_from_time_shift(SUNSET - 1 + TWO_HOURS, TWO_HOURS as i32), 0.0f32, epsilon = 0.01f32);
    }

    #[test]
    fn test_three_after() {
        assert_relative_eq!(
            pwm_from_time(SUNRISE + (60 * 60 * 3)),
            0.7f32,
            epsilon = 0.01f32
        );
        assert_relative_eq!(
            pwm_from_time(SUNSET - (60 * 60 * 3)),
            0.7f32,
            epsilon = 0.01f32
        );
    }

    #[test]
    fn test_three_after_shift() {
        const TWO_HOURS: u32 = 2 * 60 * 60;

        assert_relative_eq!(
            pwm_from_time_shift(SUNRISE + TWO_HOURS + (60 * 60 * 3), TWO_HOURS as i32),
            0.7f32,
            epsilon = 0.01f32
        );
        assert_relative_eq!(
            pwm_from_time_shift(SUNSET + TWO_HOURS - (60 * 60 * 3), TWO_HOURS as i32),
            0.7f32,
            epsilon = 0.01f32
        );
    }
}
