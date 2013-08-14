pub fn u16_lo(n: u16) -> u8 {
    (n & 0xFF) as u8
}

pub fn u16_hi(n: u16) -> u8 {
    ((n & 0xFF00) >> 8) as u8
}

pub fn u16_make(hi: u8, lo: u8) -> u16 {
    (hi as u16 << 8) + lo as u16
}

pub fn u16_set_hi(w: u16, b: u8) -> u16 {
    u16_make(b, u16_lo(w))
}

pub fn u16_set_lo(w: u16, b: u8) -> u16 {
    u16_make(u16_hi(w), b)
}

