use std::collections::HashMap;

use crate::common::{errors::ParaxisError, structs::Node, traits::Grid};

pub struct SparseGrid<T, const N: usize> {
    data: HashMap<Vec<u8>, Node<T, [f32; N]>>,
    size: [f32; N],
}
impl<T: Clone, const N: usize> Grid<T, [f32; N]> for SparseGrid<T, N> {
    fn new(size: [f32; N]) -> Result<Self, ParaxisError> {
        if size.iter().any(|s| *s < 0.0) {
            return Err(ParaxisError::NegativeSize);
        }
        let data = HashMap::new();
        Ok(Self { data, size })
    }
    fn insert(&mut self, data: T, position: &[f32; N]) -> Result<(), ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        let position_bytes = bytemuck::cast_slice(position);
        self.data
            .entry(position_bytes.to_vec())
            .or_insert_with(|| Node {
                position: *position,
                inner: None,
            })
            .inner = Some(data);
        Ok(())
    }
    fn remove(&mut self, position: &[f32; N]) -> Result<Node<T, [f32; N]>, ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        let position_bytes = bytemuck::cast_slice(position);
        let node = self.data.get_mut(position_bytes).unwrap();
        if node.inner.is_some() {
            let node_clone = node.clone();
            node.inner = None;
            Ok(node_clone)
        } else {
            Err(ParaxisError::EmptyPosition)
        }
    }
    fn get(&self, position: &[f32; N]) -> Result<&Node<T, [f32; N]>, ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        let position_bytes = bytemuck::cast_slice(position);
        let value_opt = self.data.get(position_bytes);
        match value_opt.is_none() {
            true => Err(ParaxisError::EmptyPosition),
            false => Ok(value_opt.unwrap()),
        }
    }
    fn get_mut(&mut self, position: &[f32; N]) -> Result<&mut Node<T, [f32; N]>, ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        let position_bytes = bytemuck::cast_slice(position);
        let value_opt = self.data.get_mut(position_bytes);
        match value_opt.is_none() {
            true => Err(ParaxisError::EmptyPosition),
            false => Ok(value_opt.unwrap()),
        }
    }
    fn in_grid_bounds(&self, position: &[f32; N]) -> bool {
        position
            .iter()
            .zip(self.size.iter())
            .all(|(&p, &s)| (0.0..s).contains(&p))
    }
}
