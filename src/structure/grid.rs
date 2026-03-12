use std::cmp::Ordering;

use crate::mathematics::ivector::IVector;
use indexmap::IndexMap;

#[derive(Clone, Debug)]
pub struct Grid<T, const N: usize> {
    inner: IndexMap<IVector<N>, T>,
    shape: IVector<N>,
}
impl<T: Clone, const N: usize> Grid<T, N> {
    pub fn new(shape: IVector<N>) -> Self {
        Self {
            inner: IndexMap::new(),
            shape,
        }
    }
    pub fn neighbours(&self, position: &IVector<N>) -> Vec<(IVector<N>, T)> {
        let mut result = Vec::new();
        for d in 0..N {
            for &delta in &[-1, 1] {
                let mut neighbor = *position;
                neighbor[d] = neighbor[d] as i32 + delta;
                if let Some(value) = self.inner.get(&neighbor) {
                    let value = value.clone();
                    result.push((neighbor, value));
                }
            }
        }
        result
    }
    pub fn insert(&mut self, item: T, position: IVector<N>) {
        self.inner.insert(position, item);
    }
    pub fn swap_remove(&mut self, position: IVector<N>) {
        self.inner.swap_remove(&position);
    }
    pub fn shift_remove(&mut self, position: IVector<N>) {
        self.inner.shift_remove(&position);
    }
    pub fn sort_by<F: FnMut(&IVector<N>, &T, &IVector<N>, &T) -> Ordering>(&mut self, cmp: F) {
        self.inner.sort_by(cmp);
    }
    pub fn get(&self, position: IVector<N>) -> Option<&T> {
        self.inner.get(&position)
    }
    pub fn shape(&self) -> IVector<N> {
        self.shape
    }
    pub fn positions(&self) -> Vec<IVector<N>> {
        self.inner.keys().cloned().collect()
    }
    pub fn elements(&self) -> Vec<T> {
        self.inner.values().cloned().collect()
    }
}
impl<T: Clone> Grid<T, 2> {
    fn get_row(&self, y: i32) -> Vec<T> {
        (0..self.shape[0])
            .filter_map(|x| self.inner.get(&IVector::new([x, y])).cloned())
            .collect()
    }
    fn get_col(&self, x: i32) -> Vec<T> {
        (0..self.shape[1])
            .filter_map(|y| self.inner.get(&IVector::new([x, y])).cloned())
            .collect()
    }
    pub fn top(&self) -> Vec<T> {
        self.get_row(self.shape[1] - 1)
    }
    pub fn bottom(&self) -> Vec<T> {
        self.get_row(0)
    }
    pub fn left(&self) -> Vec<T> {
        self.get_col(0)
    }
    pub fn right(&self) -> Vec<T> {
        self.get_col(self.shape[0] - 1)
    }
}
impl<T: Clone> Grid<T, 3> {
    fn get_plane(&self, axis: usize, index: i32) -> Vec<T> {
        self.inner
            .iter()
            .filter_map(|(pos, val)| {
                if pos[axis] == index {
                    Some(val.clone())
                } else {
                    None
                }
            })
            .collect()
    }

    pub fn top(&self) -> Vec<T> {
        self.get_plane(1, self.shape[1] - 1)
    }
    pub fn bottom(&self) -> Vec<T> {
        self.get_plane(1, 0)
    }
    pub fn left(&self) -> Vec<T> {
        self.get_plane(0, 0)
    }
    pub fn right(&self) -> Vec<T> {
        self.get_plane(0, self.shape[0] - 1)
    }
    pub fn front(&self) -> Vec<T> {
        self.get_plane(2, 0)
    }
    pub fn back(&self) -> Vec<T> {
        self.get_plane(2, self.shape[2] - 1)
    }
}
