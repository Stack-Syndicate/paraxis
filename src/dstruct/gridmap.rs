use std::collections::{
    HashMap,
    hash_map::{Entry, Iter, IterMut},
};

use crate::maths::la::vector::Vector;

#[derive(Debug, Clone)]
pub struct GridMap<T: Eq, const N: usize> {
    inner: HashMap<Vector<i32, N>, T>,
}
impl<'a, T: Eq, const N: usize> GridMap<T, N> {
    pub fn new() -> Self {
        Self {
            inner: HashMap::new(),
        }
    }
    pub fn entry(&mut self, position: Vector<i32, N>) -> Entry<'_, Vector<i32, N>, T> {
        self.inner.entry(position)
    }
    pub fn insert(&mut self, position: Vector<i32, N>, element: T) {
        self.inner.insert(position, element);
    }
    pub fn remove(&mut self, position: Vector<i32, N>) {
        self.inner.remove(&position);
    }
    pub fn get(&self, position: &Vector<i32, N>) -> Option<&T> {
        self.inner.get(position)
    }
    pub fn get_mut(&mut self, position: &Vector<i32, N>) -> Option<&mut T> {
        self.inner.get_mut(position)
    }
    pub fn get_neighbors(&self, pos: Vector<i32, N>) -> Vec<(Vector<i32, N>, Option<&T>)> {
        let mut neighbours = Vec::with_capacity(2 * N);
        for axis in 0..N {
            for dir in [-1, 1] {
                let offset = Vector::unit(axis) * dir;
                let neighbour_position = pos + offset;
                let neighbour = self.get(&neighbour_position);
                neighbours.push((neighbour_position, neighbour));
            }
        }
        neighbours
    }
    pub fn for_each_neighbor_mut<F>(&mut self, position: Vector<i32, N>, mut f: F)
    where
        F: FnMut(&Vector<i32, N>, &mut T),
    {
        for axis in 0..N {
            for dir in [-1, 1] {
                let neighbor_position = position + (Vector::unit(axis) * dir);
                if let Some(mut value) = self.inner.get_mut(&neighbor_position) {
                    f(&neighbor_position, &mut value);
                }
            }
        }
    }
    pub fn for_each_in_window<F>(
        &self,
        min_corner: Vector<i32, N>,
        size: i32,
        shape: Option<&[usize; N]>,
        mut f: F,
    ) where
        F: FnMut(Vector<i32, N>, Option<&T>),
    {
        let mut offset = [0i32; N];
        let total_cells = size.pow(N as u32);
        for _ in 0..total_cells {
            let mut current_position = min_corner + Vector::from(offset);
            if let Some(bounds) = shape {
                let mut wrapped_coords = [0i32; N];
                for axis in 0..N {
                    let limit = bounds[axis] as i32;
                    wrapped_coords[axis] = current_position.inner[axis].rem_euclid(limit);
                }
                current_position = Vector::from(wrapped_coords);
            }
            f(current_position, self.get(&current_position));
            for axis in 0..N {
                offset[axis] += 1;
                if offset[axis] < size {
                    break;
                } else {
                    offset[axis] = 0;
                }
            }
        }
    }
    pub fn iter(&self) -> Iter<'_, Vector<i32, N>, T> {
        self.inner.iter()
    }
    pub fn iter_mut(&mut self) -> IterMut<'_, Vector<i32, N>, T> {
        self.inner.iter_mut()
    }
}
