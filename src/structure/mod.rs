use crate::mathematics::ivector::IVector;
use std::collections::HashMap;

pub struct Grid<T: Copy, const N: usize> {
    inner: HashMap<IVector<N>, T>,
    shape: IVector<N>,
}

impl<T: Copy, const N: usize> Grid<T, N> {
    pub fn new(shape: IVector<N>) -> Self {
        Self {
            inner: HashMap::new(),
            shape,
        }
    }
    pub fn neighbours(&self, position: &IVector<N>) -> Vec<(IVector<N>, T)> {
        let mut result = Vec::new();
        for d in 0..N {
            for &delta in &[-1, 1] {
                let mut neighbor = *position;
                neighbor[d] = neighbor[d] as i32 + delta;
                if let Some(&value) = self.inner.get(&neighbor) {
                    result.push((neighbor, value));
                }
            }
        }
        result
    }
    pub fn insert(&mut self, item: T, position: IVector<N>) {
        self.inner.insert(position, item);
    }
    pub fn remove(&mut self, position: IVector<N>) {
        self.inner.remove(&position);
    }
    pub fn get(&self, position: IVector<N>) -> Option<&T> {
        self.inner.get(&position)
    }
}

