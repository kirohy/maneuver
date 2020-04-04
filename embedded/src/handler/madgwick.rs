use libm::{asinf, atan2f};

fn inv_sqrt(x: f32) -> f32 {
    let half_x = 0.5_f32 * x;
    #[repr(C)]
    union Val {
        f: f32,
        i: u32,
    }
    let mut v = Val { f: x };
    unsafe {
        v.i = 0x5f3759df - (v.i >> 1);
        v.f = v.f * (1.5f32 - half_x * v.f * v.f);
        v.f = v.f * (1.5f32 - half_x * v.f * v.f);
        v.f
    }
}

fn normalize3d(x: &mut f32, y: &mut f32, z: &mut f32) {
    let norm = inv_sqrt(*x * *x + *y * *y + *z * *z);
    *x *= norm;
    *y *= norm;
    *z *= norm;
}

fn normalize4d(q0: &mut f32, q1: &mut f32, q2: &mut f32, q3: &mut f32) {
    let norm = inv_sqrt(*q0 * *q0 + *q1 * *q1 + *q2 * *q2 + *q3 * *q3);
    *q0 *= norm;
    *q1 *= norm;
    *q2 *= norm;
    *q3 *= norm;
}

fn rotate_and_scalevector(
    q0: f32,
    q1: f32,
    q2: f32,
    q3: f32,
    _2dx: f32,
    _2dy: f32,
    _2dz: f32,
    rx: &mut f32,
    ry: &mut f32,
    rz: &mut f32,
) {
    *rx = _2dx * (0.5_f32 - q2 * q2 - q3 * q3)
        + _2dy * (q0 * q3 + q1 * q2)
        + _2dz * (q1 * q3 - q0 * q2);
    *ry = _2dx * (q1 * q2 - q0 * q3)
        + _2dy * (0.5_f32 - q1 * q1 - q3 * q3)
        + _2dz * (q0 * q1 + q2 * q3);
    *rz = _2dx * (q0 * q2 + q1 * q3)
        + _2dy * (q2 * q3 - q0 * q1)
        + _2dz * (0.5_f32 - q1 * q1 - q2 * q2);
}

fn add_gradient_descent_step(
    q0: f32,
    q1: f32,
    q2: f32,
    q3: f32,
    _2dx: f32,
    _2dy: f32,
    _2dz: f32,
    mx: f32,
    my: f32,
    mz: f32,
    s0: &mut f32,
    s1: &mut f32,
    s2: &mut f32,
    s3: &mut f32,
) {
    let mut f0: f32 = 0.0;
    let mut f1: f32 = 0.0;
    let mut f2: f32 = 0.0;

    rotate_and_scalevector(q0, q1, q2, q3, _2dx, _2dy, _2dz, &mut f0, &mut f1, &mut f2);

    f0 -= mx;
    f1 -= my;
    f2 -= mz;

    *s0 +=
        (_2dy * q3 - _2dz * q2) * f0 + (-_2dx * q3 + _2dz * q1) * f1 + (_2dx * q2 - _2dy * q1) * f2;
    *s1 += (_2dy * q2 + _2dz * q3) * f0
        + (_2dx * q2 - 2.0_f32 * _2dy * q1 + _2dz * q0) * f1
        + (_2dx * q3 - _2dy * q0 - 2.0_f32 * _2dz * q1) * f2;
    *s2 += (-2.0_f32 * _2dx * q2 + _2dy * q1 - _2dz * q0) * f0
        + (_2dx * q1 + _2dz * q3) * f1
        + (_2dx * q0 + _2dy * q3 - 2.0_f32 * _2dz * q2) * f2;
    *s3 += (-2.0_f32 * _2dx * q3 + _2dy * q0 + _2dz * q1) * f0
        + (-_2dx * q0 - 2.0_f32 * _2dy * q3 + _2dz * q2) * f1
        + (_2dx * q1 + _2dy * q2) * f2;
}

fn orientation_change_from_gyro(
    q0: f32,
    q1: f32,
    q2: f32,
    q3: f32,
    gx: f32,
    gy: f32,
    gz: f32,
    q_dot1: &mut f32,
    q_dot2: &mut f32,
    q_dot3: &mut f32,
    q_dot4: &mut f32,
) {
    *q_dot1 = 0.5_f32 * (-q1 * gx - q2 * gy - q3 * gz);
    *q_dot2 = 0.5_f32 * (q0 * gx + q2 * gz - q3 * gy);
    *q_dot3 = 0.5_f32 * (q0 * gy - q1 * gz + q3 * gx);
    *q_dot4 = 0.5_f32 * (q0 * gz + q1 * gy - q2 * gx);
}

#[derive(Clone, Copy)]
pub struct Estimated {
    gain: f32,
    q0: f32,
    q1: f32,
    q2: f32,
    q3: f32,
    dt: f32,
}

impl Default for Estimated {
    fn default() -> Self {
        Self {
            gain: 0.1,
            q0: 1.0,
            q1: 0.0,
            q2: 0.0,
            q3: 0.0,
            dt: 1.0 / 512.0,
        }
    }
}

impl Estimated {
    pub fn new(gain: f32, freq: f32) -> Self {
        Self {
            gain,
            dt: 1.0_f32 / freq,
            ..Default::default()
        }
    }

    #[allow(dead_code)]
    fn get_angles_rad(&self) -> (f32, f32, f32) {
        let roll = atan2f(
            2.0 * (self.q0 * self.q1 + self.q2 * self.q3),
            self.q0 * self.q0 - self.q1 * self.q1 - self.q2 * self.q2 + self.q3 * self.q3,
        );
        let pitch = asinf(2.0_f32 * (self.q0 * self.q2 - self.q1 * self.q3));
        let yaw = atan2f(
            2.0 * (self.q0 * self.q3 + self.q1 * self.q2),
            self.q0 * self.q0 + self.q1 * self.q1 - self.q2 * self.q2 - self.q3 * self.q3,
        );
        (roll, pitch, yaw)
    }

    pub fn get_q0(&self) -> f32 {
        self.q0
    }

    pub fn get_q1(&self) -> f32 {
        self.q1
    }

    pub fn get_q2(&self) -> f32 {
        self.q2
    }

    pub fn get_q3(&self) -> f32 {
        self.q3
    }

    pub fn get_q(&self, n: i32) -> Result<f32, ()> {
        match n {
            0 => Ok(self.get_q0()),
            1 => Ok(self.get_q1()),
            2 => Ok(self.get_q2()),
            3 => Ok(self.get_q3()),
            _ => Err(()),
        }
    }

    pub fn update_imu(&mut self, a_x: f32, a_y: f32, a_z: f32, g_x: f32, g_y: f32, g_z: f32) {
        let mut ax = a_x;
        let mut ay = a_y;
        let mut az = a_z;
        let mut _2bxy = 0.0_f32;
        let mut _2bz = 0.0_f32;
        let mut s0 = 0.0_f32;
        let mut s1 = 0.0_f32;
        let mut s2 = 0.0_f32;
        let mut s3 = 0.0_f32;

        // convert to rad/sec
        let gx = g_x * 0.0174533_f32;
        let gy = g_y * 0.0174533_f32;
        let gz = g_z * 0.0174533_f32;

        let mut q_dot1 = 0.0_f32;
        let mut q_dot2 = 0.0_f32;
        let mut q_dot3 = 0.0_f32;
        let mut q_dot4 = 0.0_f32;

        orientation_change_from_gyro(
            self.q0,
            self.q1,
            self.q2,
            self.q3,
            gx,
            gy,
            gz,
            &mut q_dot1,
            &mut q_dot2,
            &mut q_dot3,
            &mut q_dot4,
        );

        if !((ax == 0.0_f32) && (ax == 0.0_f32) && (ax == 0.0_f32)) {
            normalize3d(&mut ax, &mut ay, &mut az);

            add_gradient_descent_step(
                self.q0, self.q1, self.q2, self.q3, 0.0_f32, 0.0_f32, 2.0_f32, ax, ay, az, &mut s0,
                &mut s1, &mut s2, &mut s3,
            );

            normalize4d(&mut s0, &mut s1, &mut s2, &mut s3);

            q_dot1 -= self.gain * s0;
            q_dot2 -= self.gain * s1;
            q_dot3 -= self.gain * s2;
            q_dot4 -= self.gain * s3;
        }

        self.q0 += q_dot1 * self.dt;
        self.q1 += q_dot2 * self.dt;
        self.q2 += q_dot3 * self.dt;
        self.q3 += q_dot4 * self.dt;

        normalize4d(&mut self.q0, &mut self.q1, &mut self.q2, &mut self.q3);
    }
}
