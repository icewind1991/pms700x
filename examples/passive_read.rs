use linux_embedded_hal::Serial;
use nb::block;
use pms700x::Pms700X;
use std::env::args;
use std::path::PathBuf;
use std::thread::sleep;
use std::time::Duration;

fn main() {
    let port = args().skip(1).next().expect("No serial provided");
    let serial = Serial::open(&PathBuf::from(port)).unwrap();

    let mut pms = Pms700X::new(serial).into_passive().unwrap();
    sleep(Duration::from_millis(50));
    dbg!(block!(pms.read()).unwrap());
}
