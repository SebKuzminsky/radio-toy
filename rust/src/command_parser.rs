const IN_BUF_SIZE: usize = 64;

// pub enum Command {
//     Ping,
// }

pub struct InputBuffer {
    buf: [u8; IN_BUF_SIZE],
    index: usize,
}

impl InputBuffer {
    pub fn new() -> Self {
        InputBuffer {
            buf: [0; IN_BUF_SIZE],
            index: 0,
        }
    }

    pub fn handle_char(&mut self, b: u8) {
        if b == b'\n' {
            self.handle_line();
            self.index = 0;
        } else {
            log::info!("{}", b as char);
            let i = self.index;
            self.buf[i] = b;
            self.index += 1;
            if self.index >= IN_BUF_SIZE {
                log::info!("input buffer overflow, resetting: {:?}", self.buf);
                self.index = 0;
            }
        }
    }

    fn handle_line(&self) {
        let s = match core::str::from_utf8(&self.buf[0..self.index]) {
            Ok(s) => s,
            Err(_) => return,
        };

        log::info!("got line: '{}'", s);
        let mut tokens = s.split_whitespace();
        let cmd = match tokens.next() {
            Some(cmd) => cmd,
            None => return,
        };
        match cmd {
            cmd if "bootloader".eq_ignore_ascii_case(cmd) => {
                log::info!("resetting to bootloader");
                embassy_rp::rom_data::reset_to_usb_boot(0, 0);
            }
            cmd if "ping".eq_ignore_ascii_case(cmd) => {
                log::info!("pong");
            }
            _ => {
                log::info!("unknown command '{}'", cmd);
            }
        }
    }
}
