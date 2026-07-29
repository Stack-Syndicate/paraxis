use std::collections::HashMap;

use crate::common::{errors::ParaxisError, structs::Node, traits::Grid};

pub struct Grid2D<T: Clone> {
    data: Vec<Node<T, [i32; 2]>>,
    size: [i32; 2],
}
impl<T: Clone> Grid<T, [i32; 2]> for Grid2D<T> {
    fn new(size: [i32; 2]) -> Result<impl Grid<T, [i32; 2]>, ParaxisError> {
        if size.iter().any(|s| *s < 0) {
            return Err(ParaxisError::NegativeSize);
        }
        let mut data = Vec::with_capacity((size[0] * size[1]) as usize);
        for y in 0..size[1] {
            for x in 0..size[0] {
                data.push(Node::<T, [i32; 2]> {
                    position: [x, y],
                    inner: None,
                });
            }
        }
        Ok(Self { data, size })
    }
    fn insert(&mut self, data: T, position: &[i32; 2]) -> Result<(), ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        let grid_id = position[0] + position[1] * self.size[0];
        self.data[grid_id as usize].inner = Some(data);
        Ok(())
    }
    fn remove(&mut self, position: &[i32; 2]) -> Result<Node<T, [i32; 2]>, ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::NegativeSize);
        }
        let grid_id = position[0] + position[1] * self.size[0];
        let old_value_opt = self.data.get_mut(grid_id as usize);
        match old_value_opt.is_none() {
            true => Err(ParaxisError::EmptyPosition),
            false => {
                let old_value = old_value_opt.unwrap();
                old_value.inner = None;
                Ok(old_value.clone())
            }
        }
    }
    fn get(&self, position: &[i32; 2]) -> Result<&Node<T, [i32; 2]>, ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        let grid_id = position[0] + position[1] * self.size[0];
        let value_opt = self.data.get(grid_id as usize);
        match value_opt.is_none() {
            true => Err(ParaxisError::EmptyPosition),
            false => Ok(value_opt.unwrap()),
        }
    }
    fn get_mut(&mut self, position: &[i32; 2]) -> Result<&mut Node<T, [i32; 2]>, ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        let grid_id = position[0] + position[1] * self.size[0];
        let value_opt = self.data.get_mut(grid_id as usize);
        match value_opt.is_none() {
            true => Err(ParaxisError::EmptyPosition),
            false => Ok(value_opt.unwrap()),
        }
    }
    fn in_grid_bounds(&self, position: &[i32; 2]) -> bool {
        0 <= position[0]
            && position[0] < self.size[0]
            && 0 <= position[1]
            && position[1] < self.size[1]
    }
}

pub struct Grid3D<T: Clone> {
    data: Vec<Node<T, [i32; 3]>>,
    size: [i32; 3],
}
impl<T: Clone> Grid<T, [i32; 3]> for Grid3D<T> {
    fn new(size: [i32; 3]) -> Result<impl Grid<T, [i32; 3]>, ParaxisError> {
        if size.iter().any(|s| *s < 0) {
            return Err(ParaxisError::NegativeSize);
        }
        let mut data = Vec::with_capacity((size[0] * size[1] * size[2]) as usize);
        for z in 0..size[2] {
            for y in 0..size[1] {
                for x in 0..size[0] {
                    data.push(Node::<T, [i32; 3]> {
                        position: [x, y, z],
                        inner: None,
                    });
                }
            }
        }
        Ok(Self { data, size })
    }
    fn insert(&mut self, data: T, position: &[i32; 3]) -> Result<(), ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        let grid_id = position[0] + position[1] * self.size[0] + position[2] * self.size[1];
        self.data[grid_id as usize].inner = Some(data);
        Ok(())
    }
    fn remove(&mut self, position: &[i32; 3]) -> Result<Node<T, [i32; 3]>, ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        let grid_id = position[0] + position[1] * self.size[0] + position[2] * self.size[1];
        let old_value_opt = self.data.get_mut(grid_id as usize);
        match old_value_opt.is_none() {
            true => Err(ParaxisError::EmptyPosition),
            false => {
                let old_value = old_value_opt.unwrap();
                old_value.inner = None;
                Ok(old_value.clone())
            }
        }
    }
    fn get(&self, position: &[i32; 3]) -> Result<&Node<T, [i32; 3]>, ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        let grid_id = position[0] + position[1] * self.size[0] + position[2] * self.size[1];
        let value_opt = self.data.get(grid_id as usize);
        match value_opt.is_none() {
            true => Err(ParaxisError::EmptyPosition),
            false => Ok(value_opt.unwrap()),
        }
    }
    fn get_mut(&mut self, position: &[i32; 3]) -> Result<&mut Node<T, [i32; 3]>, ParaxisError> {
        if !self.in_grid_bounds(position) {
            return Err(ParaxisError::OutOfBounds);
        }
        let grid_id = position[0] + position[1] * self.size[0] + position[2] * self.size[1];
        let value_opt = self.data.get_mut(grid_id as usize);
        match value_opt.is_none() {
            true => Err(ParaxisError::EmptyPosition),
            false => Ok(value_opt.unwrap()),
        }
    }
    fn in_grid_bounds(&self, position: &[i32; 3]) -> bool {
        0 <= position[0]
            && position[0] < self.size[0]
            && 0 <= position[1]
            && position[1] < self.size[1]
            && 0 <= position[2]
            && position[2] < self.size[2]
    }
}

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
        };
        let remove_opt = self.data.remove(position);
        match remove_opt.is_none() {
            true => Err(ParaxisError::EmptyPosition),
            false => Ok(remove_opt.unwrap()),
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
