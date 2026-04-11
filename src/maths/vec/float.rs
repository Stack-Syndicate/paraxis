use num_traits::Float;

use crate::maths::vec::Vector;

impl<T: Float, const N: usize> Vector<T, N> {
    #[inline(always)]
    pub fn dist_euclidean(&self, other: Self) -> T {
        self.inner
            .iter()
            .zip(other.inner.iter())
            .fold(T::zero(), |acc, x| acc + (*x.1 - *x.0) * (*x.1 - *x.0))
            .sqrt()
    }
    #[inline(always)]
    pub fn dist_euclidean_squared(&self, other: Self) -> T {
        self.inner
            .iter()
            .zip(other.inner.iter())
            .fold(T::zero(), |acc, x| acc + (*x.1 - *x.0) * (*x.1 - *x.0))
    }
    #[inline(always)]
    pub fn normalize(&self) -> Self {
        return *self / (self.dot(*self).sqrt());
    }
}
