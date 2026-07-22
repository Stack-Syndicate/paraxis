use crate::common::structs::Node;

pub trait Grid<T: Clone, P> {
    fn new(size: P) -> impl Grid<T, P>;
    fn insert(&mut self, data: T, position: &P);
    fn remove(&mut self, position: &P) -> Node<T, P>;
    fn get(&self, position: &P) -> Option<&Node<T, P>>;
    fn get_mut(&mut self, position: &P) -> Option<&mut Node<T, P>>;
    fn in_grid_bounds(&self, position: &P) -> bool;
}
