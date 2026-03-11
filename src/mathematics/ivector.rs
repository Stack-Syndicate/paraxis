use std::ops::{Add, Div, Index, IndexMut, Mul, Sub};
use std::simd::Simd;

use rand::RngExt;
use rand_distr::{Normal, Uniform};

#[derive(Hash, Eq, PartialEq, Clone, Copy, Debug)]
pub struct IVector<const N: usize> {
    inner: Simd<i32, N>,
}
macro_rules! impl_elementwise_ops {
    ($($trait:ident => $method:ident),*) => {
        $(
            impl<const N: usize> $trait for IVector<N> {
                type Output = IVector<N>;
                fn $method(self, rhs: Self) -> Self::Output {
                    Self { inner: self.inner.$method(rhs.inner) }
                }
            }
            impl<const N: usize> $trait<&Self> for IVector<N> {
                type Output = IVector<N>;
                fn $method(self, rhs: &Self) -> Self::Output {
                    Self { inner: self.inner.$method(rhs.inner) }
                }
            }
             impl<const N: usize> $trait for &IVector<N> {
                type Output = IVector<N>;
                fn $method(self, rhs: Self) -> Self::Output {
                    IVector { inner: self.inner.$method(rhs.inner) }
                }
            }
            impl<const N: usize> $trait<&Self> for &IVector<N> {
                type Output = IVector<N>;
                fn $method(self, rhs: &Self) -> Self::Output {
                    IVector { inner: self.inner.$method(rhs.inner) }
                }
            }
        )*
    };
}
macro_rules! impl_scalar_ops {
    ($type:ty; $($trait:ident $method:ident $op:tt), *) => {
        $(
            impl<const N: usize> $trait<$type> for IVector<N> {
                type Output = IVector<N>;
                fn $method(self, rhs: $type) -> Self::Output {
                    IVector { inner: self.inner $op Simd::splat(rhs as i32) }
                }
            }
            impl<const N: usize> $trait<$type> for &IVector<N> {
                type Output = IVector<N>;
                fn $method(self, rhs: $type) -> Self::Output {
                    IVector { inner: self.inner $op Simd::splat(rhs as i32) }
                }
            }
        )*
    };
}
macro_rules! impl_scalar_ops_typed {
    ($($type:ty), *) => {
        $(
            impl_scalar_ops!($type; Add add +, Sub sub -, Mul mul *, Div div /);
        )*
    };
}
impl_elementwise_ops!(Add => add, Sub => sub, Mul => mul, Div => div);
impl_scalar_ops_typed!(f32, f64, u32, u64);
impl<const N: usize> Mul<i32> for &mut IVector<N> {
    type Output = IVector<N>;
    fn mul(self, rhs: i32) -> Self::Output {
        IVector {
            inner: self.inner * Simd::splat(rhs),
        }
    }
}
impl<const N: usize> Div<i32> for &mut IVector<N> {
    type Output = IVector<N>;
    fn div(self, rhs: i32) -> Self::Output {
        IVector {
            inner: self.inner / Simd::splat(rhs),
        }
    }
}
impl<const N: usize> Index<usize> for IVector<N> {
    type Output = i32;
    fn index(&self, index: usize) -> &Self::Output {
        let val = &self.inner[index];
        &val
    }
}
impl<const N: usize> IndexMut<usize> for IVector<N> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.inner[index]
    }
}
impl<const N: usize> IVector<N> {
    pub fn new(data: [i32; N]) -> Self {
        let inner = Simd::from_slice(data.as_slice());
        Self { inner }
    }
    pub fn random_uniform(low: i32, high: i32) -> Self {
        let mut rng = rand::rng();
        let mut slice = [0i32; N];
        for x in slice.iter_mut() {
            *x = rng.sample(Uniform::new(low, high).unwrap());
        }
        Self::new(slice)
    }
    pub fn random_normal() -> Self {
        let mut rng = rand::rng();
        let mut slice = [0i32; N];
        for x in slice.iter_mut() {
            let f: f64 = rng.sample(Normal::new(0.0, 1.0).unwrap());
            *x = f.round() as i32;
        }
        Self::new(slice)
    }
    pub fn dot(&self, other: &Self) -> i32 {
        (self * other).sum()
    }
    pub fn prod(&self) -> i32 {
        self.inner.to_array().iter().product()
    }
    pub fn sum(&self) -> i32 {
        self.inner.to_array().iter().sum()
    }
    pub fn max(&self) -> i32 {
        *self.inner.to_array().iter().max().unwrap()
    }
    pub fn to_vec(&self) -> Vec<i32> {
        self.inner.to_array().to_vec()
    }
}
impl IVector<3> {
    pub fn xxx(&self) -> Self {
        Self::new([self.inner[0]; 3])
    }
    pub fn yyy(&self) -> Self {
        Self::new([self.inner[1], self.inner[1], self.inner[1]])
    }
    pub fn zzz(&self) -> Self {
        Self::new([self.inner[2], self.inner[2], self.inner[2]])
    }
    pub fn yzx(&self) -> Self {
        Self::new([self.inner[1], self.inner[2], self.inner[0]])
    }
    pub fn zyx(&self) -> Self {
        Self::new([self.inner[2], self.inner[1], self.inner[0]])
    }
    pub fn cross(&self, other: &Self) -> Self {
        let a = self.inner;
        let b = other.inner;
        let a_yzx = Simd::from_array([a[1], a[2], a[0]]);
        let a_zxy = Simd::from_array([a[2], a[0], a[1]]);
        let b_yzx = Simd::from_array([b[1], b[2], b[0]]);
        let b_zxy = Simd::from_array([b[2], b[0], b[1]]);
        let result = a_yzx * b_zxy - a_zxy * b_yzx;
        Self { inner: result }
    }
}
