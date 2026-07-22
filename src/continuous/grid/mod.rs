use std::collections::HashMap;

use crate::common::{structs::Node, traits::Grid};

pub struct SparseGrid<T, const N: usize> {
    data: HashMap<[i32; N], Node<T, [i32; N]>>,
    size: [i32; N],
}
impl<T: Clone, const N: usize> Grid<T, [i32; N]> for SparseGrid<T, N> {
    fn new(size: [i32; N]) -> impl Grid<T, [i32; N]> {
        let data = HashMap::new();
        Self { data, size }
    }
    fn insert(&mut self, data: T, position: &[i32; N]) {
        if !self.in_grid_bounds(position) {
            panic!("Insertion error: position out of range.")
        }
        self.data
            .entry(*position)
            .or_insert_with(|| Node {
                position: *position,
                inner: None,
            })
            .inner = Some(data)
    }
    fn remove(&mut self, position: &[i32; N]) -> Node<T, [i32; N]> {
        if !self.in_grid_bounds(position) {
            panic!("Removal error: position out of range.")
        }
        self.data.remove(position).unwrap()
    }
    fn get(&self, position: &[i32; N]) -> Option<&Node<T, [i32; N]>> {
        if !self.in_grid_bounds(position) {
            panic!("Get error: position out of range.")
        }
        self.data.get(position)
    }
    fn get_mut(&mut self, position: &[i32; N]) -> Option<&mut Node<T, [i32; N]>> {
        if !self.in_grid_bounds(position) {
            panic!("Get error: position out of range.")
        }
        self.data.get_mut(position)
    }
    fn in_grid_bounds(&self, position: &[i32; N]) -> bool {
        0 <= position[0]
            && position[0] < self.size[0]
            && 0 <= position[1]
            && position[1] < self.size[1]
    }
}
