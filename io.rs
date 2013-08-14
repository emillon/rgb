pub fn read_port(port: u8) -> u8 {
    match port {
        0x00 => { // Joystick
            0x0F // None pressed
        }
        _ => fail!("Input on port %02X", port as uint)
    }
}
