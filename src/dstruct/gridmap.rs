use dashmap::{
    DashMap, Entry,
    iter::{Iter, IterMut},
    mapref::one::{Ref, RefMut},
};

use crate::maths::la::vector::Vector;

pub struct GridMap<T: Eq, const N: usize> {
    inner: DashMap<Vector<i32, N>, T>,
}
impl<'a, T: Eq, const N: usize> GridMap<T, N> {
    pub fn new() -> Self {
        Self {
            inner: DashMap::new(),
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
    pub fn get(&self, position: &Vector<i32, N>) -> Option<Ref<'_, Vector<i32, N>, T>> {
        self.inner.get(position)
    }
    pub fn get_mut(&mut self, position: &Vector<i32, N>) -> Option<RefMut<'_, Vector<i32, N>, T>> {
        self.inner.get_mut(position)
    }
    pub fn get_neighbors(
        &self,
        pos: Vector<i32, N>,
    ) -> Vec<(Vector<i32, N>, Option<Ref<'_, Vector<i32, N>, T>>)> {
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
    pub fn iter(&self) -> Iter<'_, Vector<i32, N>, T> {
        self.inner.iter()
    }
    pub fn iter_mut(&mut self) -> IterMut<'_, Vector<i32, N>, T> {
        self.inner.iter_mut()
    }
}
