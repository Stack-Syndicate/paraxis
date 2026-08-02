use crate::common::{errors::ParaxisError, structs::Node, traits::Grid, utils::grid_id};
use itertools::Itertools;
use std::collections::HashMap;

pub struct ContinuousGrid<P, D> {
    data: HashMap<Vec<u8>, Node<P, D>>,
    size: P,
}
impl<D: Clone, const N: usize> Grid<[f32; N], D> for ContinuousGrid<[f32; N], D> {
    fn new(size: &[f32; N]) -> Result<Self, ParaxisError> {
        if size.iter().any(|s| *s < 0.0) {
            return Err(ParaxisError::NegativeSize);
        }
        let data = HashMap::new();
        Ok(Self { data, size: *size })
    }
    fn insert(&mut self, data: D, position: &[f32; N]) -> Result<(), ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        let position_bytes = bytemuck::cast_slice(position);
        self.data
            .entry(position_bytes.to_vec())
            .or_insert_with(|| Node::new(position.clone(), Some(data)));
        Ok(())
    }
    fn remove(&mut self, position: &[f32; N]) -> Result<Node<[f32; N], D>, ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        let position_bytes = bytemuck::cast_slice(position);
        let node_opt = self.data.get_mut(position_bytes);
        match node_opt {
            Some(node) => {
                let node_clone = node.clone();
                node.write().inner = None;
                Ok(node_clone)
            }
            None => Err(ParaxisError::UnintNode),
        }
    }
    fn get(&self, position: &[f32; N]) -> Result<&Node<[f32; N], D>, ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        let position_bytes = bytemuck::cast_slice(position);
        let node_opt = self.data.get(position_bytes);
        match node_opt {
            Some(node) => Ok(node),
            None => Err(ParaxisError::UnintNode),
        }
    }
    fn get_mut(&mut self, position: &[f32; N]) -> Result<&mut Node<[f32; N], D>, ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        let position_bytes = bytemuck::cast_slice(position);
        let node_opt = self.data.get_mut(position_bytes);
        match node_opt {
            Some(node) => Ok(node),
            None => Err(ParaxisError::UnintNode),
        }
    }
    fn in_grid_bounds(&self, position: &[f32; N]) -> bool {
        position
            .iter()
            .zip(self.size.iter())
            .all(|(&p, &s)| (0.0..s).contains(&p))
    }
}

pub struct DenseGrid<P, D> {
    data: Vec<Node<P, D>>,
    size: P,
}
impl<D: Clone, const N: usize> Grid<[i32; N], D> for DenseGrid<[i32; N], D> {
    fn new(size: &[i32; N]) -> Result<Self, ParaxisError> {
        if size.iter().any(|s| *s < 0) {
            return Err(ParaxisError::NegativeSize);
        }
        let mut data = Vec::new();
        let iter = size.iter().map(|len| 0..*len).multi_cartesian_product();
        for indices in iter {
            data.push(Node::new(
                TryInto::<[i32; N]>::try_into(indices.as_slice())
                    .unwrap()
                    .clone(),
                None,
            ));
        }
        Ok(Self { data, size: *size })
    }
    fn insert(&mut self, data: D, position: &[i32; N]) -> Result<(), ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        let node_opt = self.data.get_mut(grid_id(self.size, *position));
        if let Some(node) = node_opt {
            node.write().inner = Some(data);
            Ok(())
        } else {
            Err(ParaxisError::UnintNode)
        }
    }
    fn remove(&mut self, position: &[i32; N]) -> Result<Node<[i32; N], D>, ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        let node_opt = self.data.get_mut(grid_id(self.size, *position));
        match node_opt {
            None => Err(ParaxisError::UnintNode),
            Some(node) => {
                let node_clone = node.clone();
                node.write().inner = None;
                Ok(node_clone)
            }
        }
    }
    fn get(&self, position: &[i32; N]) -> Result<&Node<[i32; N], D>, ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        let node_opt = self.data.get(grid_id(self.size, *position));
        match node_opt {
            None => Err(ParaxisError::UnintNode),
            Some(node) => Ok(node),
        }
    }
    fn get_mut(&mut self, position: &[i32; N]) -> Result<&mut Node<[i32; N], D>, ParaxisError> {
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

pub struct SparseGrid<P, D> {
    data: HashMap<P, Node<P, D>>,
    size: P,
}
impl<D: Clone, const N: usize> Grid<[i32; N], D> for SparseGrid<[i32; N], D> {
    fn new(size: &[i32; N]) -> Result<Self, ParaxisError> {
        if size.iter().any(|s| *s < 0) {
            return Err(ParaxisError::NegativeSize);
        }
        let data = HashMap::new();
        Ok(Self { data, size: *size })
    }
    fn insert(&mut self, data: D, position: &[i32; N]) -> Result<(), ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        self.data
            .insert(*position, Node::new(*position, Some(data)));
        Ok(())
    }
    fn remove(&mut self, position: &[i32; N]) -> Result<Node<[i32; N], D>, ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        };
        let node_opt = self.data.get_mut(position);
        if let Some(node) = node_opt {
            let node_clone = node.clone();
            node.write().inner = None;
            Ok(node_clone)
        } else {
            Err(ParaxisError::UnintNode)
        }
    }
    fn get(&self, position: &[i32; N]) -> Result<&Node<[i32; N], D>, ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        let node_opt = self.data.get(position);
        match node_opt {
            None => Err(ParaxisError::UnintNode),
            Some(node) => Ok(node),
        }
    }
    fn get_mut(&mut self, position: &[i32; N]) -> Result<&mut Node<[i32; N], D>, ParaxisError> {
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
