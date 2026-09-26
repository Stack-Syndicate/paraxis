use crate::containers::array::Array;
use std::ops::{Index, IndexMut};

impl<D: Copy> Index<usize> for Array<D> {
    type Output = D;
    fn index(&self, index: usize) -> &Self::Output {
        &self.data[index]
    }
}
impl<D: Copy> IndexMut<usize> for Array<D> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.data[index]
    }
}
impl<D: Copy, const N: usize> Index<&[usize; N]> for Array<D> {
    type Output = D;
    fn index(&self, indices: &[usize; N]) -> &Self::Output {
        &self.data[self.offset(indices)]
    }
}
impl<D: Copy, const N: usize> IndexMut<&[usize; N]> for Array<D> {
    fn index_mut(&mut self, indices: &[usize; N]) -> &mut Self::Output {
        let offset = self.offset(indices);
        &mut self.data[offset]
    }
}
