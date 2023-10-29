use core::fmt::Debug;

use defmt::{error, info, trace, unwrap, Debug2Format};
use embassy_time::{Duration, Timer};
use embedded_io_async::{Read, Write};
use esp32s3_hal::{uart, Uart};
use tmc2209::reg;

pub const UART_BAUD_RATE: u32 = 230400;

const TIMEOUT: Duration = Duration::from_micros(120);

/// Represents the connection to a TMC2209's UART interface
pub struct Tmc2209UartConnection {
    uart_address: u8,
    change_count: u32,
}

impl Tmc2209UartConnection {
    pub async fn connect<P: uart::Instance>(
        uart: &mut Uart<'static, P>,
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
        P: uart::Instance,
    >(
        &self,
        uart: &mut Uart<'static, P>,
    ) -> Result<R, ()> {
        let state = read_register_internal(uart, self.uart_address, R::ADDRESS)
            .await
            .unwrap();

        let val: u32 = state.into();
        Ok(R::from(val))
    }

    pub async fn read_register_state<'d, P: uart::Instance>(
        &self,
        uart: &mut Uart<'static, P>,
        register_address: tmc2209::reg::Address,
    ) -> Result<tmc2209::reg::State, ()> {
        read_register_internal(uart, self.uart_address, register_address).await
    }

    pub async fn write_register<'d, P: uart::Instance, R>(
        &mut self,
        uart: &mut Uart<'static, P>,
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

async fn read_register_internal<'d, P: uart::Instance>(
    uart: &mut Uart<'static, P>,
    uart_address: u8,
    register_address: tmc2209::reg::Address,
) -> Result<tmc2209::reg::State, ()> {
    defmt::debug!(
        "Writing read request to {:?}@{}",
        defmt::Debug2Format(&register_address),
        uart_address
    );

    // Size the read buffer to hold the echo'd request and the response
    uart.set_rx_fifo_full_threshold(
        (tmc2209::ReadRequest::LEN_BYTES + tmc2209::ReadResponse::LEN_BYTES) as u16,
    )
    .unwrap();

    let req = tmc2209::ReadRequest::from_addr(uart_address, register_address);
    defmt::debug!("bytes: {}", req.bytes(),);

    if let Err(e) = uart.write_bytes(req.bytes()) {
        error!("{:?}", e);
        return Err(());
    }

    defmt::debug!(
        "...reading echo and response, rx interrupt: {}",
        uart.rx_fifo_full_interrupt_set()
    );
    let mut buffer: [u8; tmc2209::ReadRequest::LEN_BYTES + tmc2209::ReadResponse::LEN_BYTES] =
        [0; tmc2209::ReadRequest::LEN_BYTES + tmc2209::ReadResponse::LEN_BYTES];

    match uart.read_exact(&mut buffer).await {
        Ok(_) => {
            defmt::debug!("bytes read\n {}", buffer);
        }
        Err(e) => {
            error!("{}", e);
            panic!();
        }
    }

    let mut reader = tmc2209::Reader::default();

    defmt::debug!("...reading response");

    match reader.read_response(&buffer[tmc2209::ReadRequest::LEN_BYTES..12]) {
        (_, Some(res)) => {
            let state = res.reg_state().unwrap();
            defmt::debug!("response {:?}", Debug2Format(&state));
            Ok(state)
        }
        (bytes, _) => {
            error!("no response: {} bytes read", bytes);
            error!("{}", &buffer);
            Err(())
        }
    }
}
