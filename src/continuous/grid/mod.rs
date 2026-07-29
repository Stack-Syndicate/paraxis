use std::collections::HashMap;

use crate::common::{errors::ParaxisError, structs::Node, traits::Grid};

pub struct SparseGrid<T, const N: usize> {
    data: HashMap<[i32; N], Node<T, [i32; N]>>,
    size: [i32; N],
}
impl<T: Clone, const N: usize> Grid<T, [i32; N]> for SparseGrid<T, N> {
    fn new(size: [i32; N]) -> Result<impl Grid<T, [i32; N]>, ParaxisError> {
        if size.iter().any(|s| *s < 0) {
            return Err(ParaxisError::NegativeSize);
        }
        let data = HashMap::new();
        Ok(Self { data, size })
    }
    fn insert(&mut self, data: T, position: &[i32; N]) -> Result<(), ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        self.data
            .entry(*position)
            .or_insert_with(|| Node {
                position: *position,
                inner: None,
            })
            .inner = Some(data);
        Ok(())
    }
    fn remove(&mut self, position: &[i32; N]) -> Result<Node<T, [i32; N]>, ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        let value_opt = self.data.remove(position);
        match value_opt {
            Some(value) => Ok(value),
            None => Err(ParaxisError::EmptyPosition),
        }
    }
    fn get(&self, position: &[i32; N]) -> Result<&Node<T, [i32; N]>, ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        let value_opt = self.data.get(position);
        match value_opt.is_none() {
            true => Err(ParaxisError::EmptyPosition),
            false => Ok(value_opt.unwrap()),
        }
    }
    fn get_mut(&mut self, position: &[i32; N]) -> Result<&mut Node<T, [i32; N]>, ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        let value_opt = self.data.get_mut(position);
        match value_opt.is_none() {
            true => Err(ParaxisError::EmptyPosition),
            false => Ok(value_opt.unwrap()),
        }
    }
    fn in_grid_bounds(&self, position: &[i32; N]) -> bool {
        0 <= position[0]
            && position[0] < self.size[0]
            && 0 <= position[1]
            && position[1] < self.size[1]
    }
}
