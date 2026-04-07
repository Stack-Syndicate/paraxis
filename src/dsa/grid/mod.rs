use crate::maths::vec::Vector;
use num_traits::Num;
use std::{collections::HashMap, hash::Hash};

pub struct Grid<D: Num + Eq + Hash + Clone, T: Clone, const N: usize> {
    data: HashMap<Vector<D, N>, T>,
}
impl<D: Num + Eq + Hash + Clone, T: Clone, const N: usize> Grid<D, T, N> {
    pub fn new() -> Self {
        Self {
            data: HashMap::new(),
        }
    }
    pub fn from_slice(slice: &[(Vector<D, N>, T)]) -> Self {
        let data = slice.to_vec().into_iter().collect();
        Self { data }
    }
    pub fn from_vec(vec: Vec<(Vector<D, N>, T)>) -> Self {
        let data = vec.into_iter().collect();
        Self { data }
    }
    pub fn insert(&mut self, position: Vector<D, N>, data: T) {
        self.data.insert(position, data);
    }
    pub fn remove(&mut self, position: &Vector<D, N>) {
        self.data.remove(position);
    }
}
