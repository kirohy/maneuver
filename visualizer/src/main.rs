use std::{
    io::Read,
    net::TcpListener,
    sync::{mpsc, Arc, Mutex},
    thread,
    time::{Duration, Instant},
};

use kiss3d::{camera::ArcBall, light::Light, window::Window};

use na::{
    geometry::{Quaternion, UnitQuaternion},
    Point3,
};
use nalgebra as na;

use visualizer::graphics::*;

const HEADER: [u8; 2] = [0xE0, 0xE0];
const BUF_SIZE: usize = 22;

fn main() {
    let listener = TcpListener::bind("0.0.0.0:55555").unwrap();
    let mut data_raw = [0u8; BUF_SIZE];

    let (tx, rx) = mpsc::channel();

    let data = Arc::new(Mutex::new(Vec::new()));

    match listener.accept() {
        Ok((mut stream, addr)) => {
            println!("new client {}", addr);
            {
                let data = data.clone();
                let _ = thread::spawn(move || {
                    let mut buf = Vec::new();
                    loop {
                        match stream.read(&mut data_raw) {
                            Ok(len) => {
                                if len == BUF_SIZE {
                                    let mut buf_now = data_raw.to_vec();
                                    buf.append(&mut buf_now);
                                    break;
                                }
                            }
                            Err(_) => (),
                        }
                        thread::sleep(Duration::from_micros(10000));
                    }
                    thread::sleep(Duration::from_micros(10000));
                    let start = Instant::now();
                    loop {
                        match stream.read(&mut data_raw) {
                            Ok(len) => {
                                let data = data.clone();
                                if len == BUF_SIZE {
                                    let mut buf_now = data_raw.to_vec();
                                    for iter in buf_now.iter() {
                                        buf.push(*iter);
                                    }
                                    let mut state = false;
                                    let mut head = 0_usize;
                                    for (n, iter) in buf.iter().enumerate() {
                                        if !state {
                                            if *iter == HEADER[0] {
                                                state = true;
                                            }
                                        } else {
                                            if *iter == HEADER[1] {
                                                head = n - 1;
                                                break;
                                            } else {
                                                state = false;
                                            }
                                        }
                                    }
                                    buf.rotate_left(head + 2);
                                    buf.truncate(BUF_SIZE);
                                    let mut data = data.lock().unwrap();
                                    data.clear();
                                    data.append(&mut buf);
                                    buf.append(&mut buf_now);
                                    print!("TIME: {:>010} | ", start.elapsed().as_nanos());
                                    for i in 0..BUF_SIZE {
                                        print!("0x{:>02X} ", data[i]);
                                    }
                                    print!("\n");
                                    tx.send(()).unwrap();
                                } else if len == 0 {
                                    eprintln!("Could not read anything");
                                } else {
                                    eprintln!("Could not read full size");
                                }
                            }
                            Err(_) => eprintln!("Could not read from {}", addr),
                        }
                        thread::sleep(Duration::from_micros(9500));
                    }
                });
            }
        }
        Err(e) => println!("Could not get client : {}", e),
    }

    let mut window = Window::new("Arm Simulator");
    let eye = Point3::new(10.0, 10.0, 10.0);
    let at: Point3<f32> = Point3::origin();

    window.set_background_color(0.7, 0.7, 0.7);
    window.set_light(Light::StickToCamera);

    let mut camera = ArcBall::new(eye, at);

    let mut xyz_axis_world = window.add_group();
    put_xyz_axis(&mut xyz_axis_world, 1.0);

    let arm = window.add_group();
    let mut arm_sim = Arm::new(arm, 0.5, 4.0, 4.0);

    let data = data.clone();
    while window.render_with_camera(&mut camera) {
        if let Ok(_) = rx.recv() {
            let data = data.lock().unwrap();
            let q0 = f32::from_le_bytes([data[0], data[1], data[2], data[3]]);
            let q1 = f32::from_le_bytes([data[4], data[5], data[6], data[7]]);
            let q2 = f32::from_le_bytes([data[8], data[9], data[10], data[11]]);
            let q3 = f32::from_le_bytes([data[12], data[13], data[14], data[15]]);
            let angle = f32::from_le_bytes([data[16], data[17], data[18], data[19]]);
            let rotate_q = UnitQuaternion::from_quaternion(Quaternion::new(q0, q1, q2, q3));
            arm_sim.set_upper_posture(rotate_q);
            arm_sim.set_theta_lower(angle);
        }
    }
}
