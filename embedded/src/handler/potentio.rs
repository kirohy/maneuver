use embedded_hal;
use stm32f4xx_hal as hal;

use hal::{
    adc::{
        config::{SampleTime, Sequence},
        Adc,
    },
    delay::Delay,
    gpio::Analog,
    prelude::{_embedded_hal_adc_OneShot, _embedded_hal_blocking_delay_DelayMs},
    stm32::ADC3,
};

use core::f32::consts::PI;

const POTENTIO_GAIN: f32 = 1.0 / 15.0;
const DEG_TO_RAD: f32 = PI / 180.0;

pub struct Potentiometer<T> {
    adc: Adc<ADC3>,
    pin: T,
    init: f32,
}

impl<T> Potentiometer<T>
where
    T: embedded_hal::adc::Channel<ADC3, ID = u8>,
{
    pub fn new(adc: Adc<ADC3>, pin: T) -> Self {
        Potentiometer {
            adc,
            pin,
            init: 0.0,
        }
    }

    pub fn initialize(&mut self, delay: &mut Delay, delay_ms: u32, count: u32) -> Result<(), ()> {
        let mut init_tmp = 0.0_f32;
        for _ in 0..count {
            init_tmp += self.adc.convert(&self.pin, SampleTime::Cycles_480) as f32;
            delay.delay_ms(delay_ms);
        }
        init_tmp /= count as f32;
        self.init = init_tmp;
        Ok(())
    }

    pub fn read_rad(&mut self) -> Result<f32, ()> {
        let result = self.adc.convert(&self.pin, SampleTime::Cycles_480) as f32;
        Ok((result - self.init) * POTENTIO_GAIN * DEG_TO_RAD)
    }
}
