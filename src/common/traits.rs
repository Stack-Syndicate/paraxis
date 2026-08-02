use crate::common::{errors::ParaxisError, structs::Node};

/*
* P -> Position
* D -> Data
* */

pub trait Grid<P, D> {
    fn new(size: &P) -> Result<Self, ParaxisError>
    where
        Self: Sized;
    fn insert(&mut self, data: D, position: &P) -> Result<(), ParaxisError>;
    fn remove(&mut self, position: &P) -> Result<Node<P, D>, ParaxisError>;
    fn get(&self, position: &P) -> Result<&Node<P, D>, ParaxisError>;
    fn get_mut(&mut self, position: &P) -> Result<&mut Node<P, D>, ParaxisError>;
    fn in_grid_bounds(&self, position: &P) -> bool;
}

pub trait Tree<P, D> {
    fn new(raw_data: Vec<(P, D)>) -> Self;
    fn add(&mut self, position: &P, data: D);
    fn rebalance(&mut self);
    fn nearest_neighbour(&self, position: &P) -> Result<&Node<P, D>, ParaxisError>;
    fn nearest_neighbour_mut(&mut self, position: &P) -> Result<&mut Node<P, D>, ParaxisError>;
}
