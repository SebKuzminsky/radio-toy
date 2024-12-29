use crate::command_parser;

use embedded_io_async::Write;

#[embassy_executor::task]
pub async fn net_ui_task(
    stack: embassy_net::Stack<'static>,
    sender: &'static embassy_sync::channel::Channel<
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        command_parser::Command,
        100,
    >,
) -> ! {
    loop {
        let mut tcp_rx_buffer: [u8; 1024] = [0; 1024];
        let mut tcp_tx_buffer: [u8; 1024] = [0; 1024];

        let mut socket =
            embassy_net::tcp::TcpSocket::new(stack, &mut tcp_rx_buffer, &mut tcp_tx_buffer);
        // socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));

        if socket.accept(1234).await == Ok(()) {
            log::info!(
                "handling TCP connection from {:?}",
                socket.remote_endpoint()
            );

            let mut parser = command_parser::Parser::new();

            let mut buf: [u8; 1024] = [0; 1024];
            loop {
                match socket.read(&mut buf).await {
                    Ok(0) => {
                        log::warn!("read EOF");
                        break;
                    }

                    Ok(n) => {
                        log::info!(
                            "read from tcp: {}",
                            core::str::from_utf8(&buf[..n]).unwrap()
                        );

                        for b in &buf[..n] {
                            match parser.ingest(*b) {
                                Some(command) => {
                                    sender.send(command).await;
                                }
                                None => {}
                            }
                        }

                        match socket.write_all(&buf[..n]).await {
                            Ok(_) => {}
                            Err(e) => {
                                log::warn!("write error: {:?}", e);
                                break;
                            }
                        };
                    }

                    Err(e) => {
                        log::warn!("read error: {:?}", e);
                        break;
                    }
                };
            }
            socket.close();
        }
    }
}
