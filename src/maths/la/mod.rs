use std::simd::SimdElement;

pub mod matrix;
pub mod vector;

pub trait Scalar: SimdElement + Default + Copy {
    const ONE: Self;
    const ZERO: Self;
    fn sqrt(self) -> Self;
    fn abs(self) -> Self;
}

impl Scalar for f32 {
    const ONE: f32 = 1.0;
    const ZERO: f32 = 0.0;
    #[inline]
    fn sqrt(self) -> f32 {
        self.sqrt()
    }
    #[inline]
    fn abs(self) -> f32 {
        self.abs()
    }
}

impl Scalar for f64 {
    const ONE: f64 = 1.0;
    const ZERO: f64 = 0.0;
    #[inline]
    fn sqrt(self) -> f64 {
        self.sqrt()
    }
    #[inline]
    fn abs(self) -> f64 {
        self.abs()
    }
}

impl Scalar for i32 {
    const ONE: i32 = 1;
    const ZERO: i32 = 0;
    #[inline]
    fn sqrt(self) -> i32 {
        (self as f32).sqrt() as i32
    }
    #[inline]
    fn abs(self) -> i32 {
        self.abs()
    }
}
impl Scalar for i64 {
    const ONE: i64 = 1;
    const ZERO: i64 = 0;
    #[inline]
    fn sqrt(self) -> i64 {
        (self as f64).sqrt() as i64
    }
    #[inline]
    fn abs(self) -> i64 {
        self.abs()
    }
}
