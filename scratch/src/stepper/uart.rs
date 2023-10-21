use core::fmt::Debug;

use defmt::{error, info, trace, unwrap, Debug2Format};
use embassy_rp::uart::BufferedUart;
use embassy_time::{Duration, Timer};
use embedded_io_async::*;
use tmc2209::reg;

pub const UART_BAUD_RATE: u32 = 230400;

const TIMEOUT: Duration = Duration::from_micros(120);

/// Represents the connection to a TMC2209's UART interface
pub struct Tmc2209UartConnection {
    uart_address: u8,
    change_count: u32,
}

impl Tmc2209UartConnection {
    pub async fn connect<'d, T: embassy_rp::uart::Instance>(
        uart: &mut BufferedUart<'d, T>,
        uart_address: u8,
    ) -> Tmc2209UartConnection {
        info!("Connecting to TMC2209 @UART{}", uart_address);

        let change_count =
            read_register_internal(uart, uart_address, tmc2209::reg::Address::IFCNT)
                .await
                .unwrap()
                .reg::<reg::IFCNT>()
                .unwrap()
                .0;

        Tmc2209UartConnection {
            uart_address,
            change_count,
        }
    }

    pub async fn read_register<
        'd,
        R: tmc2209::reg::Register + From<u32>,
        T: embassy_rp::uart::Instance,
    >(
        &self,
        uart: &mut BufferedUart<'d, T>,
    ) -> Result<R, ()> {
        let state = read_register_internal(uart, self.uart_address, R::ADDRESS)
            .await
            .unwrap();

        let val: u32 = state.into();
        Ok(R::from(val))
    }

    pub async fn read_register_state<'d, T: embassy_rp::uart::Instance>(
        &self,
        uart: &mut BufferedUart<'d, T>,
        register_address: tmc2209::reg::Address,
    ) -> Result<tmc2209::reg::State, ()> {
        read_register_internal(uart, self.uart_address, register_address).await
    }

    pub async fn write_register<'d, T: embassy_rp::uart::Instance, R>(
        &mut self,
        uart: &mut BufferedUart<'d, T>,
        register: R,
    ) -> Result<u32, ()>
    where
        R: tmc2209::reg::WritableRegister + Debug,
    {
        info!("Write register {:?}", defmt::Debug2Format(&register));

        let req = tmc2209::WriteRequest::new(self.uart_address, register);
        trace!("write request: {}", req.bytes());
        if let Err(e) = uart.write_all(req.bytes()).await {
            error!("{:?}", e);
            return Err(());
        }
        unwrap!(uart.flush().await);

        // Clear the echo
        trace!("...reading echo");
        let mut buffer: [u8; tmc2209::WriteRequest::LEN_BYTES] =
            [0; tmc2209::WriteRequest::LEN_BYTES];
        match uart.read_exact(&mut buffer).await {
            Ok(_) => trace!("echo received\n{}", buffer),
            Err(e) => {
                error!("{}", e);
                panic!();
            }
        }
        Timer::after(TIMEOUT).await;

        // Check the change count register to make sure the write was successful
        let new_change_count = self
            .read_register_state(uart, tmc2209::reg::Address::IFCNT)
            .await?
            .reg::<tmc2209::reg::IFCNT>()
            .unwrap()
            .0;
        let prev_change_count = self.change_count;
        self.change_count = new_change_count;

        if (prev_change_count + 1) % 256 == new_change_count {
            Ok(new_change_count)
        } else {
            error!(
                "change count mismatch: {} + 1 != {}",
                prev_change_count, new_change_count
            );
            Err(())
        }
    }
}

async fn read_register_internal<'d, T: embassy_rp::uart::Instance>(
    uart: &mut BufferedUart<'d, T>,
    uart_address: u8,
    register_address: tmc2209::reg::Address,
) -> Result<tmc2209::reg::State, ()> {
    trace!(
        "Writing read request to {:?}@{}",
        defmt::Debug2Format(&register_address),
        uart_address
    );
    let req = tmc2209::ReadRequest::from_addr(uart_address, register_address);
    if let Err(e) = uart.write_all(req.bytes()).await {
        error!("{:?}", e);
        return Err(());
    }

    trace!("...reading echo");
    let mut buffer: [u8; tmc2209::ReadRequest::LEN_BYTES] =
        [0; tmc2209::ReadRequest::LEN_BYTES];

    match uart.read_exact(&mut buffer).await {
        Ok(_) => trace!("echo received\n{}", buffer),
        Err(e) => {
            error!("{}", e);
            panic!();
        }
    }

    let mut buffer: [u8; tmc2209::ReadResponse::LEN_BYTES] =
        [0; tmc2209::ReadResponse::LEN_BYTES];
    let mut reader = tmc2209::Reader::default();

    trace!("...reading response");
    Timer::after(TIMEOUT).await;

    match uart.read_exact(&mut buffer).await {
        Ok(_) => {
            trace!("response bytes {}", buffer);
            match reader.read_response(&buffer) {
                (_, Some(res)) => {
                    let state = res.reg_state().unwrap();
                    trace!("response {:?}", Debug2Format(&state));
                    Ok(state)
                }
                (bytes, _) => {
                    error!("no response: {} bytes read", bytes);
                    error!("{}", &buffer);
                    Err(())
                }
            }
        }
        Err(e) => {
            error!("{:?}", e);
            Err(())
        }
    }
}
