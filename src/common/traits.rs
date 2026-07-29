use crate::common::{errors::ParaxisError, structs::Node};

pub trait Grid<T: Clone, P> {
    fn new(size: P) -> Result<Self, ParaxisError>
    where
        Self: Sized;
    fn insert(&mut self, data: T, position: &P) -> Result<(), ParaxisError>;
    fn remove(&mut self, position: &P) -> Result<Node<T, P>, ParaxisError>;
    fn get(&self, position: &P) -> Result<&Node<T, P>, ParaxisError>;
    fn get_mut(&mut self, position: &P) -> Result<&mut Node<T, P>, ParaxisError>;
    fn in_grid_bounds(&self, position: &P) -> bool;
}
