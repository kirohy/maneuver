use stm32f4xx_hal as hal;

use hal::prelude::_embedded_hal_serial_Write;
use hal::{block, serial::Tx};

use super::madgwick;

pub fn transmit_base<T>(tx: &mut Tx<T>, data: &[u8])
where
    Tx<T>: _embedded_hal_serial_Write<u8>,
    <Tx<T> as _embedded_hal_serial_Write<u8>>::Error: core::fmt::Debug,
{
    for iter in data {
        block!(tx.write(*iter)).unwrap();
    }
}

pub fn transmit_float<T>(tx: &mut Tx<T>, data: f32, header: &[u8])
where
    Tx<T>: _embedded_hal_serial_Write<u8>,
    <Tx<T> as _embedded_hal_serial_Write<u8>>::Error: core::fmt::Debug,
{
    let bytes = data.to_le_bytes();

    transmit_base(tx, header);
    transmit_base(tx, &bytes);
}

pub fn transmit_quaternion<T>(tx: &mut Tx<T>, data: madgwick::Estimated, header: &[u8])
where
    Tx<T>: _embedded_hal_serial_Write<u8>,
    <Tx<T> as _embedded_hal_serial_Write<u8>>::Error: core::fmt::Debug,
{
    let mut bytes = [0_u8; 16];
    for i in 0..4 {
        let tmp = data.get_q(i as i32).unwrap().to_le_bytes();
        for j in 0..4 {
            bytes[i * 4 + j] = tmp[j];
        }
    }

    transmit_base(tx, header);
    transmit_base(tx, &bytes);
}
