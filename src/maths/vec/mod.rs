use num_traits::Num;

pub mod float;
pub mod ops;
pub mod signed;

#[derive(Hash, PartialEq, Eq, Clone, Copy)]
pub struct Vector<T: Num, const N: usize> {
    pub inner: [T; N],
}
impl<T: Num + Copy, const N: usize> Vector<T, N> {
    pub fn new(slice: [T; N]) -> Self {
        Self { inner: slice }
    }
}
