use std::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Sub, SubAssign};

use num_traits::Num;

use crate::maths::vec::Vector;

impl<T: Num + Copy, const N: usize> Add<T> for Vector<T, N> {
    type Output = Self;
    fn add(self, rhs: T) -> Self::Output {
        let mut inner = self.inner;
        for val in inner.iter_mut() {
            *val = *val + rhs;
        }
        Self { inner }
    }
}

impl<T: Num + Copy, const N: usize> AddAssign<T> for Vector<T, N> {
    fn add_assign(&mut self, rhs: T) {
        for val in self.inner.iter_mut() {
            *val = *val + rhs;
        }
    }
}

impl<T: Num + Copy, const N: usize> Sub<T> for Vector<T, N> {
    type Output = Self;
    fn sub(self, rhs: T) -> Self::Output {
        let mut inner = self.inner;
        for val in inner.iter_mut() {
            *val = *val - rhs;
        }
        Self { inner }
    }
}

impl<T: Num + Copy, const N: usize> SubAssign<T> for Vector<T, N> {
    fn sub_assign(&mut self, rhs: T) {
        for val in self.inner.iter_mut() {
            *val = *val + rhs;
        }
    }
}

impl<T: Num + Copy, const N: usize> Mul<T> for Vector<T, N> {
    type Output = Self;
    fn mul(self, rhs: T) -> Self::Output {
        let mut inner = self.inner;
        for val in inner.iter_mut() {
            *val = *val * rhs;
        }
        Self { inner }
    }
}

impl<T: Num + Copy, const N: usize> MulAssign<T> for Vector<T, N> {
    fn mul_assign(&mut self, rhs: T) {
        for val in self.inner.iter_mut() {
            *val = *val + rhs;
        }
    }
}

impl<T: Num + Copy, const N: usize> Div<T> for Vector<T, N> {
    type Output = Self;
    fn div(self, rhs: T) -> Self::Output {
        let mut inner = self.inner;
        for val in inner.iter_mut() {
            *val = *val / rhs;
        }
        Self { inner }
    }
}

impl<T: Num + Copy, const N: usize> DivAssign<T> for Vector<T, N> {
    fn div_assign(&mut self, rhs: T) {
        for val in self.inner.iter_mut() {
            *val = *val + rhs;
        }
    }
}
