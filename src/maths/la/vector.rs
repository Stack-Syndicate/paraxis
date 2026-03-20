use num_traits::{real::Real, One, Zero};

use std::{
    fmt::{Debug, Display, Formatter},
    ops::{
        Add, AddAssign, BitAnd, BitOr, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Sub,
        SubAssign,
    },
    simd::{prelude::*, SimdElement},
};

#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash, Default)]
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
    T: SimdElement,
    Simd<T, N>: Mul<Output = Simd<T, N>> + SimdFloat<Scalar = T>,
{
    type Output = T;

    #[inline]
    fn bitor(self, rhs: Self) -> Self::Output {
        (self.inner * rhs.inner).reduce_sum()
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

        let a_l = simd_swizzle!(a, [1, 2, 0]);
        let b_r = simd_swizzle!(b, [2, 0, 1]);
        let a_r = simd_swizzle!(a, [2, 0, 1]);
        let b_l = simd_swizzle!(b, [1, 2, 0]);

        Vector {
            inner: (a_l * b_r) - (a_r * b_l),
        }
    }
}

impl<T: SimdElement, const N: usize> Vector<T, N> {
    pub fn from_slice(slice: &[T; N]) -> Self {
        Self {
            inner: Simd::from_slice(slice),
        }
    }

    pub fn unit(axis: usize) -> Self
    where
        T: Zero + One,
    {
        let mut v = Simd::splat(T::zero());
        v[axis] = T::one();
        Self { inner: v }
    }

    pub fn clamp_max(&mut self, min: &[T; N], max: &[T; N])
    where
        Simd<T, N>: SimdOrd,
    {
        self.inner = self
            .inner
            .simd_clamp(Simd::from_slice(min), Simd::from_slice(max));
    }

    pub fn distance_squared(self, other: &Self) -> T
    where
        Simd<T, N>: Mul<Output = Simd<T, N>> + Sub<Output = Simd<T, N>> + SimdFloat<Scalar = T>,
    {
        let diff = self.inner - other.inner;
        (diff * diff).reduce_sum()
    }

    pub fn distance(self, other: &Self) -> T
    where
        T: Real,
        Simd<T, N>: SimdFloat<Scalar = T> + Mul<Output = Simd<T, N>> + Sub<Output = Simd<T, N>>,
    {
        self.distance_squared(other).sqrt()
    }

    pub fn length(self) -> T
    where
        T: Real,
        Simd<T, N>: SimdFloat<Scalar = T> + Mul<Output = Simd<T, N>>,
    {
        (self.inner * self.inner).reduce_sum().sqrt()
    }

    pub fn normalize(self) -> Self
    where
        T: Real,
        Simd<T, N>: SimdFloat<Scalar = T> + Mul<Output = Simd<T, N>> + Div<Output = Simd<T, N>>,
    {
        let len = self.length();
        if len > T::zero() {
            Self {
                inner: self.inner / Simd::splat(len),
            }
        } else {
            self
        }
    }

    pub fn project(self, b: Self) -> Self
    where
        T: Real,
        Simd<T, N>: SimdFloat<Scalar = T> + Mul<Output = Simd<T, N>> + Div<Output = Simd<T, N>>,
    {
        let dot = (self.inner * b.inner).reduce_sum();
        let len_sq = (b.inner * b.inner).reduce_sum();
        Self {
            inner: b.inner * Simd::splat(dot / len_sq),
        }
    }

    pub fn lerp(self, target: Self, t: T) -> Self
    where
        Simd<T, N>: Mul<Output = Simd<T, N>> + Add<Output = Simd<T, N>> + Sub<Output = Simd<T, N>>,
    {
        self + (target - self) * t
    }

    pub fn round(self) -> Self
    where
        T: Real,
    {
        let mut arr = [T::zero(); N];
        for i in 0..N {
            arr[i] = self.inner[i].round();
        }
        Self {
            inner: Simd::from_array(arr),
        }
    }

    pub fn round_to_scale(self, factor: T) -> Self
    where
        T: Real,
    {
        let mut arr = [T::zero(); N];
        for i in 0..N {
            arr[i] = (self.inner[i] * factor).round() / factor;
        }
        Self {
            inner: Simd::from_array(arr),
        }
    }
}

impl<T: SimdElement, const N: usize> Index<usize> for Vector<T, N> {
    type Output = T;

    fn index(&self, index: usize) -> &Self::Output {
        &self.inner[index]
    }
}

impl<T: SimdElement, const N: usize> IndexMut<usize> for Vector<T, N> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.inner[index]
    }
}

impl<T: SimdElement + Debug, const N: usize> Display for Vector<T, N> {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "[")?;
        for i in 0..N {
            write!(f, "{:?}", self.inner[i])?;
            if i < N - 1 {
                write!(f, ", ")?;
            }
        }
        write!(f, "]")
    }
}

impl<T: SimdElement, const N: usize> From<[T; N]> for Vector<T, N> {
    fn from(arr: [T; N]) -> Self {
        Self {
            inner: Simd::from_array(arr),
        }
    }
}
