use thiserror::Error;

#[derive(Error, Debug)]
pub enum ParaxisError {
    #[error("Sizes cannot be negative")]
    NegativeSize,
    #[error("Position is empty")]
    EmptyPosition,
    #[error("Position out of bounds")]
    OutOfBounds,
    #[error("Uninitialised node")]
    UnintNode,
}
