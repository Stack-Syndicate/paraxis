use num_traits::{PrimInt, Signed};

use crate::maths::vec::Vector;

impl<T: PrimInt + Signed, const N: usize> Vector<T, N> {
    pub fn dist_manhattan(&self, other: Self) -> T {
        self.inner
            .iter()
            .zip(other.inner.iter())
            .fold(T::zero(), |acc, x| acc + (*x.1 - *x.0).abs())
    }
}
