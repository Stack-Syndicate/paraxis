use std::ops::BitOr;

use crate::mathematics::vector::Vector;

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Matrix<const LROW: usize, const LCOL: usize> {
    pub inner: [Vector<LROW>; LCOL],
}

macro_rules! impl_elementwise_ops {
    ($($trait:ident => $method:ident),*) => {
        $(
            impl<const LROW: usize, const LCOL: usize> std::ops::$trait for Matrix<LROW, LCOL> {
                type Output = Self;

                fn $method(self, rhs: Self) -> Self::Output {
                    let inner = std::array::from_fn(|i| self.inner[i].$method(rhs.inner[i]));
                    Self { inner }
                }
            }
            impl<const LROW: usize, const LCOL: usize> std::ops::$trait<&Self> for Matrix<LROW, LCOL> {
                type Output = Self;

                fn $method(self, rhs: &Self) -> Self::Output {
                    let inner = std::array::from_fn(|i| self.inner[i].$method(rhs.inner[i]));
                    Self { inner }
                }
            }
            impl<const LROW: usize, const LCOL: usize> std::ops::$trait for &Matrix<LROW, LCOL> {
                type Output = Matrix<LROW, LCOL>;

                fn $method(self, rhs: Self) -> Self::Output {
                    let inner = std::array::from_fn(|i| self.inner[i].$method(rhs.inner[i]));
                    Matrix { inner }
                }
            }
            impl<const LROW: usize, const LCOL: usize> std::ops::$trait<&Self> for &Matrix<LROW, LCOL> {
                type Output = Matrix<LROW, LCOL>;

                fn $method(self, rhs: &Self) -> Self::Output {
                    let inner = std::array::from_fn(|i| self.inner[i].$method(rhs.inner[i]));
                    Matrix { inner }
                }
            }
        )*
    };
}

impl_elementwise_ops!(Add => add, Sub => sub, Mul => mul, Div => div);
impl<const LROW: usize, const LCOL: usize> Matrix<LROW, LCOL> {
    pub fn new(data: [[f32; LROW]; LCOL]) -> Self {
        let inner = data.map(Vector::new);
        Self { inner }
    }
    pub fn sum(&self) -> f32 {
        self.inner.iter().map(|v| v.sum()).sum()
    }
    pub fn prod(&self) -> f32 {
        self.inner.iter().map(|v| v.prod()).product()
    }
    pub fn dot(&self, other: &Self) -> f32 {
        self.inner
            .iter()
            .zip(other.inner.iter())
            .map(|(a, b)| a.dot(b))
            .sum()
    }
    pub fn transpose(&self) -> Matrix<LCOL, LROW> {
        let mut data = [[0.0_f32; LCOL]; LROW];
        for i in 0..LCOL {
            for j in 0..LROW {
                data[j][i] = self.inner[i][j];
            }
        }
        Matrix::new(data)
    }
    pub fn col(&self, i: usize) -> Vector<LCOL> {
        let mut row_data = [0.0_f32; LCOL];
        for j in 0..LCOL {
            row_data[j] = self.inner[j][i];
        }
        Vector::new(row_data)
    }
    pub fn row(&self, i: usize) -> Vector<LROW> {
        self.inner[i]
    }
    pub fn row_mut(&mut self, i: usize) -> &mut Vector<LROW> {
        &mut self.inner[i]
    }
    pub fn qr(&self) -> (Matrix<LROW, LCOL>, Matrix<LCOL, LCOL>) {
        let mut q = Matrix::new([[0.0; LROW]; LCOL]);
        let mut r = Matrix::new([[0.0; LCOL]; LCOL]);
        for k in 0..LROW {
            let mut a_k = self.col(k);
            for j in 0..k {
                let q_j = q.col(j);
                let r_jk = q_j.dot(&a_k);
                r.inner[j][k] = r_jk;
                a_k = a_k - q_j * r_jk;
            }
            let r_kk = a_k.mag();
            r.inner[k][k] = r_kk;
            for row in 0..LCOL {
                q.inner[row][k] = a_k[row] / r_kk;
            }
        }
        (q, r)
    }
}
impl<const N: usize> Matrix<N, N> {
    pub fn eye() -> Matrix<N, N> {
        let mut slice = [[0f32; N]; N];
        for x in 0..N {
            for y in 0..N {
                if x == y {
                    slice[x][y] = 1.0;
                }
            }
        }
        Matrix::new(slice)
    }
    pub fn eigen(&self) -> (Vector<N>, Matrix<N, N>) {
        let mut a = self.clone();
        let mut v = Matrix::<N, N>::eye();
        for _ in 0..(N * N) {
            let (q, r) = a.qr();
            a = r | q;
            v = v | q;
        }
        let mut eigenvalues = [0.0f32; N];
        for i in 0..N {
            eigenvalues[i] = a.inner[i][i];
        }
        (Vector::new(eigenvalues), v)
    }
}
impl<const LROW: usize, const LCOL: usize, const NZ: usize> BitOr<Matrix<NZ, LROW>>
    for Matrix<LROW, LCOL>
{
    type Output = Matrix<NZ, LCOL>;
    fn bitor(self, rhs: Matrix<NZ, LROW>) -> Self::Output {
        let rhs_t = rhs.transpose();
        let inner =
            std::array::from_fn(|i| std::array::from_fn(|j| self.row(i).dot(&rhs_t.row(j))));
        Matrix {
            inner: inner.map(Vector::new),
        }
    }
}
