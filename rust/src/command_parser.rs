const BUF_SIZE: usize = 64;

#[derive(Debug)]
pub enum Command {
    Ping,
    TxPreambleBytes(u8),
}

pub struct Parser {
    buf: [u8; BUF_SIZE],
    index: usize,
}

impl Parser {
    pub fn new() -> Self {
        Self {
            buf: [0; BUF_SIZE],
            index: 0,
        }
    }

    pub fn ingest(&mut self, b: u8) -> Option<Command> {
        match b {
            b'\n' | b'\r' => {
                let r = self.handle_line();
                self.index = 0;
                r
            }

            b => {
                log::info!("{}", b as char);
                let i = self.index;
                self.buf[i] = b;
                self.index += 1;
                if self.index >= BUF_SIZE {
                    log::info!("input buffer overflow, resetting: {:?}", self.buf);
                    self.index = 0;
                }
                None
            }
        }
    }

    fn handle_line(&self) -> Option<Command> {
        let s = match core::str::from_utf8(&self.buf[0..self.index]) {
            Ok(s) => s,
            Err(_) => return None,
        };

        log::info!("got line: '{}'", s);
        let mut tokens = s.split_whitespace();
        let cmd = match tokens.next() {
            Some(cmd) => cmd,
            None => return None,
        };

        match cmd {
            cmd if "bootloader".eq_ignore_ascii_case(cmd) => {
                log::info!("resetting to bootloader");
                embassy_rp::rom_data::reset_to_usb_boot(0, 0);
                unreachable!();
            }

            cmd if "ping".eq_ignore_ascii_case(cmd) => {
                log::info!("pong");
                Some(Command::Ping)
            }

            cmd if "tx-preamble-bytes".eq_ignore_ascii_case(cmd) => match tokens.next() {
                Some(t) => match t.parse::<u8>() {
                    Ok(num_bytes) => Some(Command::TxPreambleBytes(num_bytes)),

                    Err(e) => {
                        log::error!("failed to parse tx-preamble-bytes from {}: {}", t, e);
                        None
                    }
                },
                None => None,
            },

            _ => {
                log::info!("unknown command '{}'", cmd);
                None
            }
        }
    }
}
