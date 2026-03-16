use crate::maths::la::{Scalar, vector::Vector};
use std::{
    iter::Sum,
    ops::{Add, AddAssign, BitAnd, BitOr, Div, DivAssign, Mul, MulAssign, Sub, SubAssign},
    simd::{SimdElement, prelude::*},
};

#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
pub struct Matrix<T: SimdElement, const N: usize, const M: usize> {
    pub rows: [Vector<T, M>; N],
}
macro_rules! impl_matrix_arithmetic {
    ($trait:ident, $method:ident, $assign_trait:ident, $assign_method:ident) => {
        impl<T: SimdElement, const N: usize, const M: usize> $trait<Self> for Matrix<T, N, M>
        where
            Vector<T, M>: $trait<Output = Vector<T, M>> + Copy,
        {
            type Output = Self;
            #[inline]
            fn $method(self, rhs: Self) -> Self {
                Self {
                    rows: std::array::from_fn(|i| self.rows[i].$method(rhs.rows[i])),
                }
            }
        }

        impl<T: SimdElement, const N: usize, const M: usize> $assign_trait<Self> for Matrix<T, N, M>
        where
            Vector<T, M>: $assign_trait + Copy,
        {
            #[inline]
            fn $assign_method(&mut self, rhs: Self) {
                for i in 0..N {
                    self.rows[i].$assign_method(rhs.rows[i]);
                }
            }
        }

        impl<T: SimdElement, const N: usize, const M: usize> $trait<T> for Matrix<T, N, M>
        where
            Vector<T, M>: $trait<T, Output = Vector<T, M>> + Copy,
        {
            type Output = Self;
            #[inline]
            fn $method(self, rhs: T) -> Self {
                Self {
                    rows: std::array::from_fn(|i| self.rows[i].$method(rhs)),
                }
            }
        }

        impl<T: SimdElement, const N: usize, const M: usize> $assign_trait<T> for Matrix<T, N, M>
        where
            Vector<T, M>: $assign_trait<T> + Copy,
        {
            #[inline]
            fn $assign_method(&mut self, rhs: T) {
                for i in 0..N {
                    self.rows[i].$assign_method(rhs);
                }
            }
        }
    };
}

impl_matrix_arithmetic!(Mul, mul, MulAssign, mul_assign);
impl_matrix_arithmetic!(Div, div, DivAssign, div_assign);
impl_matrix_arithmetic!(Add, add, AddAssign, add_assign);
impl_matrix_arithmetic!(Sub, sub, SubAssign, sub_assign);

impl<T: SimdElement, const N: usize, const M: usize> Mul<Vector<T, M>> for Matrix<T, N, M>
where
    T: std::iter::Sum + Default + Scalar,
    Vector<T, M>: BitOr<Output = T> + Copy,
    Vector<T, N>: From<[T; N]>,
{
    type Output = Vector<T, N>;
    fn mul(self, vec: Vector<T, M>) -> Self::Output {
        let mut data = [T::ZERO; N];
        for i in 0..N {
            data[i] = self.rows[i] | vec;
        }
        Vector::from_slice(&data)
    }
}
impl<T: Scalar, const N: usize, const M: usize> Mul<Matrix<T, N, M>> for Vector<T, N>
where
    T: SimdElement + std::iter::Sum + Default + Scalar,
    Vector<T, N>: BitOr<Output = T> + Copy,
    Vector<T, M>: From<[T; M]>,
{
    type Output = Vector<T, M>;
    fn mul(self, rhs: Matrix<T, N, M>) -> Self::Output {
        let mut data = [T::ZERO; M];
        for col in 0..M {
            let mut column_vec = [T::ZERO; N];
            for row in 0..N {
                column_vec[row] = rhs.rows[row].inner[col];
            }
            data[col] = self | Vector::from_slice(&column_vec);
        }
        Vector::from_slice(&data)
    }
}
impl<T: Scalar, const N: usize> Matrix<T, N, N>
where
    T: SimdElement + Default + Scalar,
{
    pub fn identity() -> Self {
        Self {
            rows: std::array::from_fn(|i| Vector::unit(i)),
        }
    }
}
impl<T: SimdElement + Default + Scalar, const N: usize, const M: usize> Matrix<T, N, M> {
    pub fn transpose(self) -> Matrix<T, M, N> {
        let mut result_data = [[T::ZERO; N]; M];
        for r in 0..N {
            for c in 0..M {
                result_data[c][r] = self.rows[r].inner[c];
            }
        }
        Matrix {
            rows: std::array::from_fn(|i| Vector::from_slice(&result_data[i])),
        }
    }
}
impl<T: Scalar, const N: usize, const M: usize, const P: usize> BitAnd<Matrix<T, M, P>>
    for Matrix<T, N, M>
where
    T: SimdElement + std::iter::Sum + Default + Scalar,
    Vector<T, M>: BitOr<Output = T> + Copy,
{
    type Output = Matrix<T, N, P>;

    fn bitand(self, rhs: Matrix<T, M, P>) -> Self::Output {
        let rhs_t = rhs.transpose();
        let new_rows = std::array::from_fn(|i| {
            let mut row_values = [T::ZERO; P];
            for j in 0..P {
                row_values[j] = self.rows[i] | rhs_t.rows[j];
            }
            Vector::from_slice(&row_values)
        });

        Matrix { rows: new_rows }
    }
}
impl<T: SimdElement + Default + Scalar, const N: usize, const M: usize> Matrix<T, N, M> {
    pub fn from_rows(data: [[T; M]; N]) -> Self {
        Self {
            rows: std::array::from_fn(|i| Vector::from_slice(&data[i])),
        }
    }
    pub fn from_cols(data: [[T; N]; M]) -> Self
    where
        [(); M]: Sized,
    {
        let mut mat = [[T::ZERO; M]; N];
        for c in 0..M {
            for r in 0..N {
                mat[r][c] = data[c][r];
            }
        }
        Self::from_rows(mat)
    }
}
impl<T: SimdElement + Default + Scalar, const N: usize, const M: usize> Default
    for Matrix<T, N, M>
{
    fn default() -> Self {
        Self {
            rows: [Vector {
                inner: Simd::splat(T::ZERO),
            }; N],
        }
    }
}
impl<T: Scalar, const N: usize> Matrix<T, N, N>
where
    T: SimdElement + Default + Scalar + Div<Output = T>,
    Vector<T, N>: Copy + SubAssign + Mul<T, Output = Vector<T, N>>,
{
    pub fn lu(&self) -> (Self, Self) {
        let mut l = Self::identity();
        let mut u = *self;
        for i in 0..N {
            for j in (i + 1)..N {
                let factor = u.rows[j].inner[i] / u.rows[i].inner[i];
                l.rows[j].inner[i] = factor;
                let scaled_row = u.rows[i] * factor;
                u.rows[j] -= scaled_row;
            }
        }
        (l, u)
    }
}
impl<T: Scalar, const N: usize, const M: usize> Matrix<T, N, M>
where
    T: SimdElement + Default + PartialEq + Scalar + std::iter::Sum + Div<Output = T>,
    Vector<T, N>: Copy
        + SubAssign
        + BitOr<Output = T>
        + Div<T, Output = Vector<T, N>>
        + Mul<T, Output = Vector<T, N>>,
    Vector<T, M>: Copy + BitOr<Output = T> + Div<T, Output = Vector<T, M>>,
    Simd<T, N>: SimdFloat<Scalar = T>
        + Mul<Output = Simd<T, N>>
        + Sub<Output = Simd<T, N>>
        + Add<Output = Simd<T, N>>,
{
    pub fn qr(&self) -> (Matrix<T, N, M>, Matrix<T, M, M>) {
        let mut q_cols = self.transpose().rows;
        let mut r = Matrix::<T, M, M>::default();

        for i in 0..M {
            let mut v = q_cols[i];
            for j in 0..i {
                let q_j = q_cols[j];

                let dot = q_j | q_cols[i];
                r.rows[j].inner[i] = dot;

                v -= q_j * dot;
            }

            let norm = v.length();
            r.rows[i].inner[i] = norm;

            if norm != T::ZERO {
                q_cols[i] = v * (T::ONE / norm);
            } else {
                q_cols[i] = v;
            }
        }

        (Matrix { rows: q_cols }.transpose(), r)
    }
}
impl<T: Scalar, const N: usize> Matrix<T, N, N>
where
    T: SimdElement
        + Default
        + PartialEq
        + Scalar
        + Sum
        + Mul<T, Output = T>
        + PartialOrd
        + Into<f32>,
    T: Div<Output = T>,
    Vector<T, N>: Div<T, Output = Vector<T, N>>
        + Mul<T, Output = Vector<T, N>>
        + BitOr<Output = T>
        + Copy
        + SubAssign,
    Simd<T, N>: SimdFloat<Scalar = T>
        + Mul<Output = Simd<T, N>>
        + Sub<Output = Simd<T, N>>
        + Add<Output = Simd<T, N>>,
    Self: BitAnd<Output = Self>,
{
    pub fn eigenvalues(&self, max_iter: usize, tol: f32) -> Vector<T, N> {
        let mut ak = *self;

        for _ in 0..max_iter {
            let (q, r) = ak.qr();
            ak = r & q;

            if ak.has_converged(tol) {
                break;
            }
        }
        let mut evals = [T::ZERO; N];
        for i in 0..N {
            evals[i] = ak.rows[i].inner[i];
        }
        Vector::from_slice(&evals)
    }
    fn has_converged(&self, tol: f32) -> bool {
        for r in 1..N {
            for c in 0..r {
                let val = self.rows[r].inner[c].abs();
                if (val * val).into() > (tol * tol) {
                    return false;
                }
            }
        }
        true
    }
}
impl<T: Scalar, const N: usize> Matrix<T, N, N>
where
    T: SimdElement + Default + Scalar + Div<Output = T> + Mul<Output = T>,
    Vector<T, N>: Copy + SubAssign + Mul<T, Output = Vector<T, N>>,
{
    pub fn determinant(&self) -> T {
        let (_, u) = self.lu();
        let mut det = T::ONE;
        for i in 0..N {
            det = det * u.rows[i].inner[i];
        }
        det
    }
}
impl<T: Scalar, const N: usize> Matrix<T, N, N>
where
    T: SimdElement
        + Default
        + Scalar
        + Sub<Output = T>
        + Mul<Output = T>
        + Div<Output = T>
        + Add<Output = T>
        + PartialEq,
    Vector<T, N>: Copy + SubAssign + Mul<T, Output = Vector<T, N>> + BitOr<Output = T>,
{
    pub fn solve(&self, b: Vector<T, N>) -> Option<Vector<T, N>> {
        let (l, u) = self.lu();
        let mut y = [T::ZERO; N];
        for i in 0..N {
            let mut sum = T::ZERO;
            for j in 0..i {
                sum = sum + l.rows[i].inner[j] * y[j];
            }

            y[i] = b.inner[i] - sum;
        }
        let mut x = [T::ZERO; N];
        for i in (0..N).rev() {
            let mut sum = T::ZERO;
            for j in (i + 1)..N {
                sum = sum + u.rows[i].inner[j] * x[j];
            }

            let diag = u.rows[i].inner[i];
            if diag == T::ZERO {
                return None;
            }

            x[i] = (y[i] - sum) / diag;
        }
        Some(Vector::from_slice(&x))
    }
    pub fn inverse(&self) -> Option<Self> {
        let mut inv_cols = [[T::ZERO; N]; N];
        let identity = Self::identity();
        for i in 0..N {
            let col = self.solve(identity.rows[i])?;

            for j in 0..N {
                inv_cols[i][j] = col.inner[j];
            }
        }
        Some(Self::from_cols(inv_cols))
    }
    pub fn trace(&self) -> T {
        let mut sum = T::ZERO;
        for i in 0..N {
            sum = sum + self.rows[i].inner[i];
        }
        sum
    }
}
impl<const N: usize, const M: usize> Matrix<f32, N, M> {
    pub fn round(self) -> Self {
        Self {
            rows: self.rows.map(|r| r.round()),
        }
    }
    pub fn round_to_scale(self, factor: f32) -> Self {
        Self {
            rows: self.rows.map(|r| r.round_to_scale(factor)),
        }
    }
}
impl<const N: usize, const M: usize> std::fmt::Display for Matrix<f32, N, M> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let display_mat = self.round_to_scale(10000.0);
        for row in display_mat.rows {
            write!(f, "[")?;
            for i in 0..M {
                write!(f, "{:>8.4} ", row.inner[i])?;
            }
            writeln!(f, "]")?;
        }
        Ok(())
    }
}
