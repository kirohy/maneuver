use nix::sys::termios::*;

use std::{os::unix::io::AsRawFd, path::Path};
use tokio::{
    fs::*,
    net::TcpStream,
    prelude::*,
    time::{self, Duration, Instant},
};

const DEVICE: &str = "/dev/ttyACM0";
const BUF_SIZE: usize = 22;
const TCP_ADDR: &str = "127.0.0.1:55555";

#[tokio::main]
async fn main() {
    let device_path = Path::new(DEVICE);

    let mut fd = loop {
        match File::open(device_path).await {
            Ok(f) => break f,
            Err(_) => eprintln!("Could not open {}.", DEVICE),
        }
        time::delay_for(Duration::from_millis(10)).await;
    };
    let fd_raw = fd.as_raw_fd();

    let mut tty = tcgetattr(fd_raw).unwrap();
    tty.control_flags.insert(ControlFlags::CS8);
    tty.control_flags.insert(ControlFlags::CREAD);
    cfsetospeed(&mut tty, BaudRate::B115200).unwrap();
    cfsetispeed(&mut tty, BaudRate::B115200).unwrap();
    tcsetattr(fd_raw, SetArg::TCSANOW, &tty).unwrap();
    tcflush(fd_raw, FlushArg::TCIOFLUSH).unwrap();

    println!("Successfully opened {}", DEVICE);

    let mut stream = TcpStream::connect(TCP_ADDR).await.unwrap();
    println!("Successfully connected to {}", TCP_ADDR);

    let start = Instant::now();
    let mut interval = time::interval(Duration::from_micros(9980));

    let mut data_raw = [0u8; BUF_SIZE];

    loop {
        print!("TIME: {:>010} | ", start.elapsed().as_nanos());
        match fd.read(&mut data_raw).await {
            Ok(len) => {
                if len > 0 {
                    match stream.write(&data_raw[0..len]).await {
                        Ok(_) => {
                            for i in 0..len {
                                print!("0x{:>02X} ", data_raw[i]);
                            }
                            print!("\n");
                        }
                        Err(_) => print!("Could not write on {}.\n", TCP_ADDR),
                    }
                    stream.flush().await.unwrap();
                } else {
                    print!("Could not read from {}.\n", DEVICE);
                }
            }
            Err(_) => print!("Could not read from {}.\n", DEVICE),
        }
        interval.tick().await;
    }
}
