use std::{env, error};

mod joystick;
mod serial;
mod flightgear;

fn main() -> Result<(), Box<dyn error::Error>> {
    let args: Vec<_> = env::args().collect();
    let port = if args.len() > 1 {
        args[1].clone()
    } else {
        "/dev/ttyUSB1".to_owned()
    };

    println!("Connecting to serial port at {}", port);
    let mut serial = serial::SerialConnection::new(&port.into(), 115200)?;

    let joystick = joystick::Joystick::new()?;

    println!(
        "Created joystick with device path {}",
        joystick.device_path()?.to_string_lossy()
    );

    joystick.synchronise()?;

    loop {
        // === NEW: read lighting from FlightGear ===
        let panel_light = flightgear::get_property_float("/controls/lighting/main-panel-norm")
            .unwrap_or(0.0);

        let gear_warning = flightgear::get_property_bool("/ECAM/warnings/landing-gear-warning-light")
            .unwrap_or(false)
            || (flightgear::get_property_int("/controls/switches/annun-test")
            .unwrap_or(0) == 1);

        // Create a tiny packet: two floats separated by comma
        let out = format!("X{:.2},{:.0}Y\n", panel_light, if gear_warning {1} else {0});

        // Send to Arduino
        serial.write(out.as_bytes())?;
        println!("{}", out);
        //println!(".");
        
        // ===========================================
    
        serial.flush_input()?; // clear everything before polling FG
        let button_state = serial.read_button_state()?;

        for (i, &pressed) in button_state.pressed.iter().enumerate() {
            joystick.button_press(button_map(i), pressed)?;
        }

        for (i, &value) in button_state.joysticks.iter().enumerate() {
            println!("a {}", value);
            joystick.move_axis(axis_map(i), value as i32 - 512)?;
        }

        joystick.synchronise()?;
    }
}

fn button_map(i: usize) -> joystick::Button {
    use joystick::Button::*;
    match i {
        0 => LeftNorth,
        1 => LeftSouth,
        2 => LeftEast,
        3 => LeftWest,
        4 => LeftSpecial,
        5 => RightNorth,
        6 => RightSouth,
        7 => RightEast,
        8 => RightWest,
        9 => RightSpecial,
        10 => L1,
        11 => R1,
        12 => L2,
        13 => R2,
        _ => unreachable!(),
    }
}

fn axis_map(i: usize) -> joystick::Axis {
    use joystick::Axis::*;
    match i {
        0 => X,
        1 => Y,
        2 => RX,
        3 => RY,
        _ => unreachable!(),
    }
}
