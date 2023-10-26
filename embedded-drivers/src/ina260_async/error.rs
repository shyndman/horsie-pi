use core::fmt::Debug;

#[cfg_attr(feature = "defmt", derive(::defmt::Format))]
#[derive(Debug)]
pub enum Error<E>
where
    E: Debug,
{
    ImposterAtExpectedAddress,
    BusError(E),
}

impl<E: Debug> From<E> for Error<E> {
    fn from(error: E) -> Self {
        Error::BusError(error)
    }
}
