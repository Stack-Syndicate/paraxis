use nalgebra::{SVector, Scalar};
use std::{collections::HashMap, hash::Hash};

pub struct Grid<D: Scalar + Hash + Eq, T: Clone, const N: usize> {
    data: HashMap<SVector<D, N>, T>,
}
impl<D: Scalar + Hash + Eq, T: Clone, const N: usize> Grid<D, T, N> {
    pub fn new() -> Self {
        Self {
            data: HashMap::new(),
        }
    }
    pub fn from_slice(slice: &[(SVector<D, N>, T)]) -> Self {
        let data = slice.iter().cloned().collect();
        Self { data }
    }
    pub fn from_vec(vec: Vec<(SVector<D, N>, T)>) -> Self {
        let data = vec.into_iter().collect();
        Self { data }
    }
    pub fn insert(&mut self, position: SVector<D, N>, data: T) {
        self.data.insert(position, data);
    }
    pub fn remove(&mut self, position: &SVector<D, N>) {
        self.data.remove(position);
    }
}

impl<D: Scalar + Hash + Eq, T: Clone, const N: usize> Default for Grid<D, T, N> {
    fn default() -> Self {
        Self::new()
    }
}
