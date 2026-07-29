use crate::common::{errors::ParaxisError, structs::Node, traits::Grid, utils::grid_id};
use itertools::Itertools;
use std::collections::HashMap;

pub struct DenseGrid<T, const N: usize> {
    data: Vec<Node<T, [i32; N]>>,
    size: [i32; N],
}
impl<T: Clone, const N: usize> Grid<T, [i32; N]> for DenseGrid<T, N> {
    fn new(size: [i32; N]) -> Result<Self, ParaxisError> {
        if size.iter().any(|s| *s < 0) {
            return Err(ParaxisError::NegativeSize);
        }
        let mut data = Vec::new();
        let iter = size.iter().map(|len| 0..*len).multi_cartesian_product();
        for indices in iter {
            data.push(Node::<T, [i32; N]> {
                position: TryInto::<[i32; N]>::try_into(indices.as_slice()).unwrap(),
                inner: None,
            });
        }
        Ok(Self { data, size })
    }
    fn insert(&mut self, data: T, position: &[i32; N]) -> Result<(), ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        let node_opt = self.data.get_mut(grid_id(self.size, *position));
        if let Some(node) = node_opt {
            node.inner = Some(data);
            Ok(())
        } else {
            Err(ParaxisError::UnintNode)
        }
    }
    fn remove(&mut self, position: &[i32; N]) -> Result<Node<T, [i32; N]>, ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        let node_opt = self.data.get_mut(grid_id(self.size, *position));
        match node_opt {
            None => Err(ParaxisError::UnintNode),
            Some(node) => {
                let node_clone = node.clone();
                node.inner = None;
                Ok(node_clone)
            }
        }
    }
    fn get(&self, position: &[i32; N]) -> Result<&Node<T, [i32; N]>, ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        let node_opt = self.data.get(grid_id(self.size, *position));
        match node_opt {
            None => Err(ParaxisError::UnintNode),
            Some(node) => Ok(node),
        }
    }
    fn get_mut(&mut self, position: &[i32; N]) -> Result<&mut Node<T, [i32; N]>, ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        let node_opt = self.data.get_mut(grid_id(self.size, *position));
        match node_opt {
            None => Err(ParaxisError::UnintNode),
            Some(node) => Ok(node),
        }
    }
    fn in_grid_bounds(&self, position: &[i32; N]) -> bool {
        self.size
            .iter()
            .zip(position.iter())
            .all(|(&s, &p)| (0..s).contains(&p))
    }
}

pub struct SparseGrid<T, const N: usize> {
    data: HashMap<[i32; N], Node<T, [i32; N]>>,
    size: [i32; N],
}
impl<T: Clone, const N: usize> Grid<T, [i32; N]> for SparseGrid<T, N> {
    fn new(size: [i32; N]) -> Result<Self, ParaxisError> {
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
        self.data.insert(
            *position,
            Node {
                inner: Some(data),
                position: *position,
            },
        );
        Ok(())
    }
    fn remove(&mut self, position: &[i32; N]) -> Result<Node<T, [i32; N]>, ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        };
        let node_opt = self.data.get_mut(position);
        if let Some(node) = node_opt {
            let node_clone = node.clone();
            node.inner = None;
            Ok(node_clone)
        } else {
            Err(ParaxisError::UnintNode)
        }
    }
    fn get(&self, position: &[i32; N]) -> Result<&Node<T, [i32; N]>, ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        let node_opt = self.data.get(position);
        match node_opt {
            None => Err(ParaxisError::UnintNode),
            Some(node) => Ok(node),
        }
    }
    fn get_mut(&mut self, position: &[i32; N]) -> Result<&mut Node<T, [i32; N]>, ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        let node_opt = self.data.get_mut(position);
        match node_opt {
            None => Err(ParaxisError::UnintNode),
            Some(node) => Ok(node),
        }
    }
    fn in_grid_bounds(&self, position: &[i32; N]) -> bool {
        self.size
            .iter()
            .zip(position.iter())
            .all(|(&s, &p)| (0..s).contains(&p))
    }
}
