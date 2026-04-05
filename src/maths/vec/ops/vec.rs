use std::ops::{Add, AddAssign, BitAnd, BitOr, Div, DivAssign, Mul, MulAssign, Sub, SubAssign};

use num_traits::Num;

use crate::maths::vec::Vector;

impl<T: Num + Copy, const N: usize> BitOr for Vector<T, N> {
    type Output = T;
    fn bitor(self, rhs: Self) -> Self::Output {
        self.inner
            .iter()
            .zip(rhs.inner.iter())
            .map(|(&s, &r)| s * r)
            .fold(T::zero(), |acc, x| acc + x)
    }
}

impl<T: Num + Copy> BitAnd for Vector<T, 3> {
    type Output = Self;
    fn bitand(self, rhs: Self) -> Self::Output {
        let [sx, sy, sz] = self.inner;
        let [rx, ry, rz] = rhs.inner;
        return Self {
            inner: [sy * rz - sz * ry, sz * rx - sx * rz, sx * ry - sy * rx],
        };
    }
}

impl<T: Num + Copy, const N: usize> Add<Self> for Vector<T, N> {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        let mut inner = self.inner;
        for (val, oval) in inner.iter_mut().zip(rhs.inner.iter()) {
            *val = *val + *oval;
        }
        Self { inner }
    }
}

impl<T: Num + Copy, const N: usize> AddAssign<Self> for Vector<T, N> {
    fn add_assign(&mut self, rhs: Self) {
        for (val, oval) in self.inner.iter_mut().zip(rhs.inner.iter()) {
            *val = *val + *oval;
        }
    }
}

impl<T: Num + Copy, const N: usize> Sub<Self> for Vector<T, N> {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self::Output {
        let mut inner = self.inner;
        for (val, oval) in inner.iter_mut().zip(rhs.inner.iter()) {
            *val = *val - *oval;
        }
        Self { inner }
    }
}

impl<T: Num + Copy, const N: usize> SubAssign<Self> for Vector<T, N> {
    fn sub_assign(&mut self, rhs: Self) {
        for (val, oval) in self.inner.iter_mut().zip(rhs.inner.iter()) {
            *val = *val - *oval;
        }
    }
}

impl<T: Num + Copy, const N: usize> Mul<Self> for Vector<T, N> {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self::Output {
        let mut inner = self.inner;
        for (val, oval) in inner.iter_mut().zip(rhs.inner.iter()) {
            *val = *val * *oval;
        }
        Self { inner }
    }
}

impl<T: Num + Copy, const N: usize> MulAssign<Self> for Vector<T, N> {
    fn mul_assign(&mut self, rhs: Self) {
        for (val, oval) in self.inner.iter_mut().zip(rhs.inner.iter()) {
            *val = *val * *oval;
        }
    }
}

impl<T: Num + Copy, const N: usize> Div<Self> for Vector<T, N> {
    type Output = Self;
    fn div(self, rhs: Self) -> Self::Output {
        let mut inner = self.inner;
        for (val, oval) in inner.iter_mut().zip(rhs.inner.iter()) {
            *val = *val / *oval;
        }
        Self { inner }
    }
}

impl<T: Num + Copy, const N: usize> DivAssign<Self> for Vector<T, N> {
    fn div_assign(&mut self, rhs: Self) {
        for (val, oval) in self.inner.iter_mut().zip(rhs.inner.iter()) {
            *val = *val * *oval;
        }
    }
}
