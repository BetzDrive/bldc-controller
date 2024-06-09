mod bindings;
use bindings::*;

fn main() {
    println!("Hello, world!");
    println!("{}", motor_pwm_clock_freq);
}
