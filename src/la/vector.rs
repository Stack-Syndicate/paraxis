use std::{
    ops::{Add, AddAssign, BitAnd, BitOr, Div, DivAssign, Mul, MulAssign, Sub, SubAssign},
    simd::{SimdElement, prelude::*},
};

use crate::la::Scalar;

#[derive(Clone, Copy)]
pub struct Vector<T: SimdElement, const N: usize> {
    pub inner: Simd<T, N>,
}

impl<T: SimdElement, const N: usize> Add for Vector<T, N>
where
    Simd<T, N>: Add<Output = Simd<T, N>>,
{
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            inner: self.inner + rhs.inner,
        }
    }
}
impl<T: SimdElement, const N: usize> AddAssign for Vector<T, N>
where
    Simd<T, N>: Add<Output = Simd<T, N>>,
{
    fn add_assign(&mut self, rhs: Self) {
        self.inner.add_assign(rhs.inner);
    }
}
impl<T: SimdElement, const N: usize> Sub for Vector<T, N>
where
    Simd<T, N>: Sub<Output = Simd<T, N>>,
{
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            inner: self.inner - rhs.inner,
        }
    }
}
impl<T: SimdElement, const N: usize> SubAssign for Vector<T, N>
where
    Simd<T, N>: Sub<Output = Simd<T, N>>,
{
    fn sub_assign(&mut self, rhs: Self) {
        self.inner.sub_assign(rhs.inner);
    }
}

impl<T: SimdElement, const N: usize> Mul for Vector<T, N>
where
    Simd<T, N>: Mul<Output = Simd<T, N>>,
{
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        Self {
            inner: self.inner * rhs.inner,
        }
    }
}
impl<T: SimdElement, const N: usize> MulAssign for Vector<T, N>
where
    Simd<T, N>: Mul<Output = Simd<T, N>>,
{
    fn mul_assign(&mut self, rhs: Self) {
        self.inner.mul_assign(rhs.inner);
    }
}
impl<T: SimdElement, const N: usize> Div for Vector<T, N>
where
    Simd<T, N>: Div<Output = Simd<T, N>>,
{
    type Output = Self;

    fn div(self, rhs: Self) -> Self::Output {
        Self {
            inner: self.inner / rhs.inner,
        }
    }
}
impl<T: SimdElement, const N: usize> DivAssign for Vector<T, N>
where
    Simd<T, N>: Div<Output = Simd<T, N>>,
{
    fn div_assign(&mut self, rhs: Self) {
        self.inner.div_assign(rhs.inner);
    }
}

impl<T, const N: usize> BitOr for Vector<T, N>
where
    T: SimdElement + std::iter::Sum,
    Simd<T, N>: Mul<Output = Simd<T, N>>,
{
    type Output = T;
    fn bitor(self, rhs: Self) -> Self::Output {
        (self.inner * rhs.inner).as_array().iter().copied().sum()
    }
}
impl<T> BitAnd for Vector<T, 3>
where
    T: SimdElement,
    Simd<T, 3>: Mul<Output = Simd<T, 3>> + Sub<Output = Simd<T, 3>>,
{
    type Output = Self;

    fn bitand(self, rhs: Self) -> Self::Output {
        let a = self.inner;
        let b = rhs.inner;
        let a_shifted_l = simd_swizzle!(a, [1, 2, 0]);
        let b_shifted_r = simd_swizzle!(b, [2, 0, 1]);
        let a_shifted_r = simd_swizzle!(a, [2, 0, 1]);
        let b_shifted_l = simd_swizzle!(b, [1, 2, 0]);
        let res = (a_shifted_l * b_shifted_r) - (a_shifted_r * b_shifted_l);
        Vector { inner: res }
    }
}

impl<T: SimdElement, const N: usize> Add<T> for Vector<T, N>
where
    Simd<T, N>: Add<Output = Simd<T, N>>,
{
    type Output = Self;

    fn add(self, rhs: T) -> Self::Output {
        Self {
            inner: self.inner + Simd::splat(rhs),
        }
    }
}
impl<T: SimdElement, const N: usize> AddAssign<T> for Vector<T, N>
where
    Simd<T, N>: Add<Output = Simd<T, N>>,
{
    fn add_assign(&mut self, rhs: T) {
        self.inner.add_assign(Simd::splat(rhs));
    }
}
impl<T: SimdElement, const N: usize> Sub<T> for Vector<T, N>
where
    Simd<T, N>: Sub<Output = Simd<T, N>>,
{
    type Output = Self;

    fn sub(self, rhs: T) -> Self::Output {
        Self {
            inner: self.inner - Simd::splat(rhs),
        }
    }
}
impl<T: SimdElement, const N: usize> SubAssign<T> for Vector<T, N>
where
    Simd<T, N>: Sub<Output = Simd<T, N>>,
{
    fn sub_assign(&mut self, rhs: T) {
        self.inner.sub_assign(Simd::splat(rhs));
    }
}

impl<T: SimdElement, const N: usize> Mul<T> for Vector<T, N>
where
    Simd<T, N>: Mul<Output = Simd<T, N>>,
{
    type Output = Self;

    fn mul(self, rhs: T) -> Self::Output {
        Self {
            inner: self.inner * Simd::splat(rhs),
        }
    }
}
impl<T: SimdElement, const N: usize> MulAssign<T> for Vector<T, N>
where
    Simd<T, N>: Mul<Output = Simd<T, N>>,
{
    fn mul_assign(&mut self, rhs: T) {
        self.inner.mul_assign(Simd::splat(rhs));
    }
}
impl<T: SimdElement, const N: usize> Div<T> for Vector<T, N>
where
    Simd<T, N>: Div<Output = Simd<T, N>>,
{
    type Output = Self;

    fn div(self, rhs: T) -> Self::Output {
        Self {
            inner: self.inner / Simd::splat(rhs),
        }
    }
}
impl<T: SimdElement, const N: usize> DivAssign<T> for Vector<T, N>
where
    Simd<T, N>: Div<Output = Simd<T, N>>,
{
    fn div_assign(&mut self, rhs: T) {
        self.inner.div_assign(Simd::splat(rhs));
    }
}

impl<T: Scalar, const N: usize> Vector<T, N>
where
    T: SimdElement + Default + PartialEq + Div<Output = T>,
    Simd<T, N>: SimdFloat<Scalar = T>
        + Mul<Output = Simd<T, N>>
        + Sub<Output = Simd<T, N>>
        + Add<Output = Simd<T, N>>,
    Self: Div<T, Output = Self> + BitOr<Output = T> + Mul<T, Output = Self> + Mul<Output = Self>,
{
    pub fn length(self) -> T {
        let dot = (self.inner * self.inner).reduce_sum();
        dot.sqrt()
    }
    pub fn normalize(self) -> Self {
        let len = self.length();
        if len != T::ZERO { self / len } else { self }
    }
    pub fn distance(self, other: Self) -> T {
        (self - other).length()
    }
    pub fn distance_squared(self, other: Self) -> T {
        let diff = self - other;
        diff | diff
    }
    pub fn project(self, b: Self) -> Self {
        let b_len_sq = b | b;
        b * ((self | b) / b_len_sq)
    }
    pub fn clamp_max(&mut self, min: &[T; N], max: &[T; N]) {
        self.inner = self
            .inner
            .simd_clamp(Simd::from_slice(min), Simd::from_slice(max));
    }
    pub fn lerp(self, target: Self, t: T) -> Self {
        let t_v = Vector {
            inner: Simd::splat(t),
        };
        self + (target - self) * t_v
    }
}

impl<T: SimdElement + Default + Scalar, const N: usize> Vector<T, N> {
    pub fn from_slice(slice: &[T; N]) -> Self {
        Self {
            inner: Simd::from_slice(slice),
        }
    }
    pub fn unit(axis: usize) -> Self {
        let mut ui = Simd::from_slice(&[T::ZERO; N]);
        ui[axis] = T::ONE;
        Self { inner: ui }
    }
}
impl<T: SimdElement + Default + Scalar, const N: usize> From<[T; N]> for Vector<T, N> {
    fn from(arr: [T; N]) -> Self {
        Self::from_slice(&arr)
    }
}
