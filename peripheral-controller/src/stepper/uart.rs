use core::fmt::Debug;

use defmt::Debug2Format;
use embassy_sync::blocking_mutex::raw::RawMutex;
use tmc2209::reg;

use crate::uart::bus::UartDevice;

pub const UART_BAUD_RATE: u32 = 230400;

/// Represents the connection to a TMC2209's UART interface
pub struct Tmc2209UartConnection<
    M: RawMutex + 'static,
    P: 'static + esp_hal_common::uart::Instance,
> {
    uart_device: UartDevice<'static, M, P>,
    uart_address: u8,
    change_count: u32,
}

impl<M: RawMutex, P: 'static + esp_hal_common::uart::Instance> Tmc2209UartConnection<M, P> {
    pub async fn connect(
        mut uart_device: UartDevice<'static, M, P>,
        uart_address: u8,
    ) -> Self {
        defmt::info!("Connecting to TMC2209 @UART{}", uart_address);

        let change_count = read_register_internal(
            &mut uart_device,
            uart_address,
            tmc2209::reg::Address::IFCNT,
        )
        .await
        .unwrap()
        .reg::<reg::IFCNT>()
        .unwrap()
        .0;

        Tmc2209UartConnection {
            uart_device,
            uart_address,
            change_count,
        }
    }

    pub async fn read_register<R: tmc2209::reg::Register + From<u32>>(
        &mut self,
    ) -> Result<R, ()> {
        let state =
            read_register_internal(&mut self.uart_device, self.uart_address, R::ADDRESS)
                .await
                .unwrap();

        let val: u32 = state.into();
        Ok(R::from(val))
    }

    pub async fn read_register_state(
        &mut self,
        register_address: tmc2209::reg::Address,
    ) -> Result<tmc2209::reg::State, ()> {
        read_register_internal(&mut self.uart_device, self.uart_address, register_address)
            .await
    }

    pub async fn write_register<R>(&mut self, register: R) -> Result<u32, ()>
    where
        R: tmc2209::reg::WritableRegister + Debug,
    {
        defmt::info!("Write register {:?}", defmt::Debug2Format(&register));

        let req = tmc2209::WriteRequest::new(self.uart_address, register);
        defmt::trace!("write request: {}", req.bytes());
        if let Err(e) = self.uart_device.write_all(req.bytes()).await {
            defmt::error!("{:?}", Debug2Format(&e));
            return Err(());
        }

        // Clear the echo
        defmt::trace!("...reading echo");
        let mut buffer: [u8; tmc2209::WriteRequest::LEN_BYTES] =
            [0; tmc2209::WriteRequest::LEN_BYTES];
        match self.uart_device.read_exact(&mut buffer).await {
            Ok(_) => defmt::trace!("echo received\n{}", buffer),
            Err(e) => {
                defmt::error!("{}", Debug2Format(&e));
                panic!();
            }
        }

        // Check the change count register to make sure the write was successful
        let new_change_count = self
            .read_register_state(tmc2209::reg::Address::IFCNT)
            .await?
            .reg::<tmc2209::reg::IFCNT>()
            .unwrap()
            .0;
        let prev_change_count = self.change_count;
        self.change_count = new_change_count;

        if (prev_change_count + 1) % 256 == new_change_count {
            Ok(new_change_count)
        } else {
            defmt::error!(
                "change count mismatch: {} + 1 != {}",
                prev_change_count,
                new_change_count
            );
            Err(())
        }
    }
}

async fn read_register_internal<M: RawMutex, P: esp_hal_common::uart::Instance>(
    uart_device: &mut UartDevice<'static, M, P>,
    uart_address: u8,
    register_address: tmc2209::reg::Address,
) -> Result<tmc2209::reg::State, ()> {
    defmt::debug!(
        "Writing read request to {:?}@{}",
        defmt::Debug2Format(&register_address),
        uart_address
    );

    // Size the read buffer to hold the echo'd request and the response
    const READ_BUFFER_LENGTH: usize =
        tmc2209::ReadRequest::LEN_BYTES + tmc2209::ReadResponse::LEN_BYTES;

    // Set read buffer size, and reset interrupt
    uart_device
        .prepare_receive_buffer::<READ_BUFFER_LENGTH>()
        .await;

    let req = tmc2209::ReadRequest::from_addr(uart_address, register_address);
    defmt::trace!("read request bytes: {}", req.bytes());
    if let Err(e) = uart_device.write_all(req.bytes()).await {
        defmt::error!("{:?}", Debug2Format(&e));
        return Err(());
    }

    let mut buffer: [u8; tmc2209::ReadRequest::LEN_BYTES + tmc2209::ReadResponse::LEN_BYTES] =
        [0; tmc2209::ReadRequest::LEN_BYTES + tmc2209::ReadResponse::LEN_BYTES];
    match uart_device.read_exact(&mut buffer).await {
        Ok(_) => {
            defmt::trace!("bytes read\n {}", buffer);
        }
        Err(e) => {
            defmt::error!("{}", Debug2Format(&e));
            panic!();
        }
    }

    let mut reader = tmc2209::Reader::default();
    defmt::trace!("...reading response");
    match reader.read_response(&buffer[tmc2209::ReadRequest::LEN_BYTES..12]) {
        (_, Some(res)) => {
            let state = res.reg_state().unwrap();
            defmt::debug!("response {:?}", Debug2Format(&state));
            Ok(state)
        }
        (bytes, _) => {
            defmt::error!("no response: {} bytes read", bytes);
            defmt::error!("{}", &buffer);
            Err(())
        }
    }
}
