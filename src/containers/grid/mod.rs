use crate::common::{
    errors::ParaxisError,
    structs::Node,
    traits::Grid,
    utils::{grid_id, position_key},
};
use itertools::Itertools;
use num_traits::{Float, PrimInt, ToBytes};
use std::{collections::HashMap, fmt::Debug, hash::Hash, iter::successors};

pub struct ContinuousGrid<P, D> {
    data: HashMap<u64, Node<P, D>>,
    size: P,
}
impl<T: Float + ToBytes, D: Clone, const N: usize> Grid<[T; N], D> for ContinuousGrid<[T; N], D> {
    fn new(size: &[T; N]) -> Result<Self, ParaxisError> {
        if size.iter().any(|s| *s < T::zero()) {
            return Err(ParaxisError::NegativeSize);
        }
        let data = HashMap::new();
        Ok(Self { data, size: *size })
    }
    fn insert(&mut self, data: D, position: &[T; N]) -> Result<(), ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        let key = position_key(position);
        self.data
            .entry(key)
            .or_insert_with(|| Node::new(*position, Some(data)));
        Ok(())
    }
    fn remove(&mut self, position: &[T; N]) -> Result<Node<[T; N], D>, ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        let key = position_key(position);
        let node_opt = self.data.get_mut(&key);
        match node_opt {
            Some(node) => {
                let node_clone = node.clone();
                node.write().inner = None;
                Ok(node_clone)
            }
            None => Err(ParaxisError::UnintNode),
        }
    }
    fn get(&self, position: &[T; N]) -> Result<&Node<[T; N], D>, ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        let key = position_key(position);
        let node_opt = self.data.get(&key);
        match node_opt {
            Some(node) => Ok(node),
            None => Err(ParaxisError::UnintNode),
        }
    }
    fn get_mut(&mut self, position: &[T; N]) -> Result<&mut Node<[T; N], D>, ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        let key = position_key(position);
        let node_opt = self.data.get_mut(&key);
        match node_opt {
            Some(node) => Ok(node),
            None => Err(ParaxisError::UnintNode),
        }
    }
    fn in_grid_bounds(&self, position: &[T; N]) -> bool {
        position
            .iter()
            .zip(self.size.iter())
            .all(|(&p, &s)| (T::zero()..s).contains(&p))
    }
}

pub struct DenseGrid<P, D> {
    data: Vec<Node<P, D>>,
    size: P,
}
impl<T: PrimInt + Debug, D: Clone, const N: usize> Grid<[T; N], D> for DenseGrid<[T; N], D> {
    fn new(size: &[T; N]) -> Result<Self, ParaxisError> {
        if size.iter().any(|&s| s < T::zero()) {
            return Err(ParaxisError::NegativeSize);
        }
        let iter = size
            .iter()
            .map(|&len| {
                successors(Some(T::zero()), move |&x| {
                    (x + T::one() < len).then_some(x + T::one())
                })
            })
            .multi_cartesian_product();
        let data = iter
            .map(|indices| Node::new(TryInto::<[T; N]>::try_into(indices).unwrap(), None))
            .collect();
        Ok(Self { data, size: *size })
    }
    fn insert(&mut self, data: D, position: &[T; N]) -> Result<(), ParaxisError> {
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
    fn remove(&mut self, position: &[T; N]) -> Result<Node<[T; N], D>, ParaxisError> {
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
    fn get(&self, position: &[T; N]) -> Result<&Node<[T; N], D>, ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        let node_opt = self.data.get(grid_id(self.size, *position));
        match node_opt {
            None => Err(ParaxisError::UnintNode),
            Some(node) => Ok(node),
        }
    }
    fn get_mut(&mut self, position: &[T; N]) -> Result<&mut Node<[T; N], D>, ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        let node_opt = self.data.get_mut(grid_id(self.size, *position));
        match node_opt {
            None => Err(ParaxisError::UnintNode),
            Some(node) => Ok(node),
        }
    }
    fn in_grid_bounds(&self, position: &[T; N]) -> bool {
        self.size
            .iter()
            .zip(position.iter())
            .all(|(&s, &p)| (T::zero()..s).contains(&p))
    }
}

pub struct SparseGrid<P, D> {
    data: HashMap<P, Node<P, D>>,
    size: P,
}
impl<T: PrimInt + Hash, D: Clone, const N: usize> Grid<[T; N], D> for SparseGrid<[T; N], D> {
    fn new(size: &[T; N]) -> Result<Self, ParaxisError> {
        if size.iter().any(|s| *s < T::zero()) {
            return Err(ParaxisError::NegativeSize);
        }
        let data = HashMap::new();
        Ok(Self { data, size: *size })
    }
    fn insert(&mut self, data: D, position: &[T; N]) -> Result<(), ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        self.data
            .insert(*position, Node::new(*position, Some(data)));
        Ok(())
    }
    fn remove(&mut self, position: &[T; N]) -> Result<Node<[T; N], D>, ParaxisError> {
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
    fn get(&self, position: &[T; N]) -> Result<&Node<[T; N], D>, ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        let node_opt = self.data.get(position);
        match node_opt {
            None => Err(ParaxisError::UnintNode),
            Some(node) => Ok(node),
        }
    }
    fn get_mut(&mut self, position: &[T; N]) -> Result<&mut Node<[T; N], D>, ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        let node_opt = self.data.get_mut(position);
        match node_opt {
            None => Err(ParaxisError::UnintNode),
            Some(node) => Ok(node),
        }
    }
    fn in_grid_bounds(&self, position: &[T; N]) -> bool {
        self.size
            .iter()
            .zip(position.iter())
            .all(|(&s, &p)| (T::zero()..s).contains(&p))
    }
}
