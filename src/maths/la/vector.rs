use crate::maths::la::Scalar;
use std::{
    ops::{Add, AddAssign, BitAnd, BitOr, Div, DivAssign, Mul, MulAssign, Sub, SubAssign},
    simd::{prelude::*, SimdElement},
};

#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
pub struct Vector<T: SimdElement, const N: usize> {
    pub inner: Simd<T, N>,
}
macro_rules! impl_arithmetic {
    ($trait:ident, $method:ident, $assign_trait:ident, $assign_method:ident) => {
        impl<T: SimdElement, const N: usize> $trait<Self> for Vector<T, N>
        where
            Simd<T, N>: $trait<Output = Simd<T, N>>,
        {
            type Output = Self;
            #[inline]
            fn $method(self, rhs: Self) -> Self {
                Self {
                    inner: self.inner.$method(rhs.inner),
                }
            }
        }
        impl<T: SimdElement, const N: usize> $assign_trait<Self> for Vector<T, N>
        where
            Simd<T, N>: $assign_trait,
        {
            #[inline]
            fn $assign_method(&mut self, rhs: Self) {
                self.inner.$assign_method(rhs.inner);
            }
        }
        impl<T: SimdElement, const N: usize> $trait<T> for Vector<T, N>
        where
            Simd<T, N>: $trait<Output = Simd<T, N>>,
        {
            type Output = Self;
            #[inline]
            fn $method(self, rhs: T) -> Self {
                Self {
                    inner: self.inner.$method(Simd::splat(rhs)),
                }
            }
        }
        impl<T: SimdElement, const N: usize> $assign_trait<T> for Vector<T, N>
        where
            Simd<T, N>: $assign_trait,
        {
            #[inline]
            fn $assign_method(&mut self, rhs: T) {
                self.inner.$assign_method(Simd::splat(rhs));
            }
        }
    };
}

impl_arithmetic!(Add, add, AddAssign, add_assign);
impl_arithmetic!(Sub, sub, SubAssign, sub_assign);
impl_arithmetic!(Mul, mul, MulAssign, mul_assign);
impl_arithmetic!(Div, div, DivAssign, div_assign);

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
        if len != T::ZERO {
            self / len
        } else {
            self
        }
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
impl<const N: usize> Vector<f32, N> {
    pub fn round(self) -> Self {
        let mut array = [0.0; N];
        for i in 0..N {
            array[i] = self.inner[i].round();
        }
        Self {
            inner: Simd::from_array(array),
        }
    }

    pub fn round_to_scale(self, factor: f32) -> Self {
        let mut array = [0.0; N];
        for i in 0..N {
            array[i] = (self.inner[i] * factor).round() / factor;
        }
        Self {
            inner: Simd::from_array(array),
        }
    }
}
impl<const N: usize> Vector<f64, N> {
    pub fn round(self) -> Self {
        let mut array = [0.0; N];
        for i in 0..N {
            array[i] = self.inner[i].round();
        }
        Self {
            inner: Simd::from_array(array),
        }
    }

    pub fn round_to_scale(self, factor: f64) -> Self {
        let mut array = [0.0; N];
        for i in 0..N {
            array[i] = (self.inner[i] * factor).round() / factor;
        }
        Self {
            inner: Simd::from_array(array),
        }
    }
}

impl<const N: usize> Vector<i32, N> {
    pub fn unflatten_index(index: usize, shape: &[usize; N]) -> Vector<i32, N> {
        let mut index = index;
        let mut coordinates = [0i32; N];
        for axis in 0..N {
            coordinates[axis] = (index % shape[axis]) as i32;
            index /= shape[axis];
        }
        Vector::from(coordinates)
    }
    pub fn flatten_index(position: Vector<i32, N>, shape: &[usize; N]) -> usize {
        let mut index = 0;
        let mut stride = 1;
        for axis in 0..N {
            index += position.inner[axis] as usize * stride;
            stride *= shape[axis];
        }
        index
    }
    pub fn for_each_in_range<F>(min: Vector<i32, N>, max: Vector<i32, N>, mut f: F)
    where
        F: FnMut(Vector<i32, N>),
    {
        let mut current = min;
        loop {
            f(current);
            let mut axis = 0;
            while axis < N {
                current.inner[axis] += 1;
                if current.inner[axis] < max.inner[axis] {
                    break;
                } else {
                    current.inner[axis] = min.inner[axis];
                    axis += 1;
                }
            }
            if axis == N {
                break;
            }
        }
    }
}
