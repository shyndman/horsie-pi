use core::fmt::Debug;

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
