mod fire;
mod meteor;
pub use fire::*;
pub use meteor::*;

pub trait EffectFrameProvider {
    fn advance(&mut self);
}
