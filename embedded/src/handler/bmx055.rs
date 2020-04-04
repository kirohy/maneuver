use super::madgwick;

use stm32f4xx_hal as hal;

use hal::prelude::{
    _embedded_hal_blocking_delay_DelayMs, _embedded_hal_blocking_i2c_Write,
    _embedded_hal_blocking_i2c_WriteRead,
};
use hal::{delay::Delay, i2c::*};

const SETTINGS_ACC: [u8; 6] = [0x0F, 0x03, 0x10, 0x08, 0x11, 0x00];
const SETTINGS_GYR: [u8; 6] = [0x0F, 0x04, 0x10, 0x07, 0x11, 0x00];
// const SETTINGS_MAG: [u8; 10] = [0x4B, 0x01, 0x4C, 0x00, 0x4E, 0x84, 0x51, 0x04, 0x52, 0x0F];

const ADDR_ACC: u8 = 0x19;
const ADDR_GYR: u8 = 0x69;
// const ADDR_MAG: u8 = 0x13;

pub struct IMU<T, U> {
    dev: I2c<T, U>,
    x_acc: f32,
    y_acc: f32,
    z_acc: f32,
    x_gyr: f32,
    y_gyr: f32,
    z_gyr: f32,
    x_gyr_init: f32,
    y_gyr_init: f32,
    z_gyr_init: f32,
    pub imu_data: madgwick::Estimated,
}

impl<T, U> IMU<T, U>
where
    U: Pins<T>,
    I2c<T, U>: _embedded_hal_blocking_i2c_WriteRead + _embedded_hal_blocking_i2c_Write,
{
    pub fn new(i2c: I2c<T, U>, gain: f32, freq: f32) -> Self {
        IMU {
            dev: i2c,
            x_acc: 0.0,
            y_acc: 0.0,
            z_acc: 0.0,
            x_gyr: 0.0,
            y_gyr: 0.0,
            z_gyr: 0.0,
            x_gyr_init: 0.0,
            y_gyr_init: 0.0,
            z_gyr_init: 0.0,
            imu_data: madgwick::Estimated::new(gain, freq),
        }
    }

    pub fn initialize(&mut self, delay: &mut Delay, delay_ms: u32, count: u32) {
        loop {
            match self.dev.write(ADDR_ACC, &SETTINGS_ACC) {
                Ok(_) => break,
                Err(_) => continue,
            }
        }
        delay.delay_ms(delay_ms);

        loop {
            match self.dev.write(ADDR_GYR, &SETTINGS_GYR) {
                Ok(_) => break,
                Err(_) => continue,
            }
        }
        delay.delay_ms(delay_ms);

        let mut offset_x = 0.0_f32;
        let mut offset_y = 0.0_f32;
        let mut offset_z = 0.0_f32;

        for _ in 0..count {
            self.measure_gyr();
            delay.delay_ms(delay_ms);
            offset_x += self.x_gyr;
            offset_y += self.y_gyr;
            offset_z += self.z_gyr;
        }
        offset_x /= count as f32;
        offset_y /= count as f32;
        offset_z /= count as f32;
        self.x_gyr_init = offset_x;
        self.y_gyr_init = offset_y;
        self.z_gyr_init = offset_z;
    }

    fn measure_acc(&mut self) {
        let addr = [0x02];
        let mut data = [0u8; 6];
        let coefficient = 0.0096105; // 9.80665(m/s^2) * 0.00098(G/LSB)
        if let Ok(_) = self.dev.write_read(ADDR_ACC, &addr, &mut data) {
            self.x_acc = ((data[1] as f32 * 256.0) + (data[0] & 0xF0) as f32) / 16.0;
            if self.x_acc > 2047.0 {
                self.x_acc -= 4096.0
            }

            self.y_acc = ((data[3] as f32 * 256.0) + (data[2] & 0xF0) as f32) / 16.0;
            if self.y_acc > 2047.0 {
                self.y_acc -= 4096.0
            }

            self.z_acc = ((data[5] as f32 * 256.0) + (data[4] & 0xF0) as f32) / 16.0;
            if self.z_acc > 2047.0 {
                self.z_acc -= 4096.0
            }

            self.x_acc *= coefficient;
            self.y_acc *= coefficient;
            self.z_acc *= coefficient;
        }
    }

    fn measure_gyr(&mut self) {
        let addr = [0x02];
        let mut data = [0u8; 6];
        let coefficient = 0.003815; // 32767 <-> 2000degree/sec
        if let Ok(_) = self.dev.write_read(ADDR_GYR, &addr, &mut data) {
            self.x_gyr = (data[1] as f32 * 256.0) + data[0] as f32;
            if self.x_gyr > 32767.0 {
                self.x_gyr -= 65536.0
            }

            self.y_gyr = (data[3] as f32 * 256.0) + data[2] as f32;
            if self.y_gyr > 32767.0 {
                self.y_gyr -= 65536.0
            }

            self.z_gyr = (data[5] as f32 * 256.0) + data[4] as f32;
            if self.z_gyr > 32767.0 {
                self.z_gyr -= 65536.0
            }

            self.x_gyr *= coefficient;
            self.y_gyr *= coefficient;
            self.z_gyr *= coefficient;
        }
    }

    fn compensate_gyr(&mut self) {
        self.x_gyr -= self.x_gyr_init;
        self.y_gyr -= self.y_gyr_init;
        self.z_gyr -= self.z_gyr_init;
    }

    pub fn update(&mut self) {
        self.measure_acc();
        self.measure_gyr();
        self.compensate_gyr();
        self.imu_data.update_imu(
            self.x_acc, self.y_acc, self.z_acc, self.x_gyr, self.y_gyr, self.z_gyr,
        );
    }
}
