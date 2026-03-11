use std::ops::BitOr;

use crate::mathematics::ivector::IVector;

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct IMatrix<const LROW: usize, const LCOL: usize> {
    pub inner: [IVector<LROW>; LCOL],
}

macro_rules! impl_elementwise_ops {
    ($($trait:ident => $method:ident),*) => {
        $(
            impl<const LROW: usize, const LCOL: usize> std::ops::$trait for IMatrix<LROW, LCOL> {
                type Output = Self;

                fn $method(self, rhs: Self) -> Self::Output {
                    let inner = std::array::from_fn(|i| self.inner[i].$method(rhs.inner[i]));
                    Self { inner }
                }
            }
            impl<const LROW: usize, const LCOL: usize> std::ops::$trait<&Self> for IMatrix<LROW, LCOL> {
                type Output = Self;

                fn $method(self, rhs: &Self) -> Self::Output {
                    let inner = std::array::from_fn(|i| self.inner[i].$method(rhs.inner[i]));
                    Self { inner }
                }
            }
            impl<const LROW: usize, const LCOL: usize> std::ops::$trait for &IMatrix<LROW, LCOL> {
                type Output = IMatrix<LROW, LCOL>;

                fn $method(self, rhs: Self) -> Self::Output {
                    let inner = std::array::from_fn(|i| self.inner[i].$method(rhs.inner[i]));
                    IMatrix { inner }
                }
            }
            impl<const LROW: usize, const LCOL: usize> std::ops::$trait<&Self> for &IMatrix<LROW, LCOL> {
                type Output = IMatrix<LROW, LCOL>;

                fn $method(self, rhs: &Self) -> Self::Output {
                    let inner = std::array::from_fn(|i| self.inner[i].$method(rhs.inner[i]));
                    IMatrix { inner }
                }
            }
        )*
    };
}

impl_elementwise_ops!(Add => add, Sub => sub, Mul => mul, Div => div);
impl<const LROW: usize, const LCOL: usize> IMatrix<LROW, LCOL> {
    pub fn new(data: [[i32; LROW]; LCOL]) -> Self {
        let inner = data.map(IVector::new);
        Self { inner }
    }
    pub fn sum(&self) -> i32 {
        self.inner.iter().map(|v| v.sum()).sum()
    }
    pub fn prod(&self) -> i32 {
        self.inner.iter().map(|v| v.prod()).product()
    }
    pub fn dot(&self, other: &Self) -> i32 {
        self.inner
            .iter()
            .zip(other.inner.iter())
            .map(|(a, b)| a.dot(b))
            .sum()
    }
    pub fn transpose(&self) -> IMatrix<LCOL, LROW> {
        let mut data = [[0i32; LCOL]; LROW];
        for i in 0..LCOL {
            for j in 0..LROW {
                data[j][i] = self.inner[i][j];
            }
        }
        IMatrix::new(data)
    }
    pub fn col(&self, i: usize) -> IVector<LCOL> {
        let mut row_data = [0i32; LCOL];
        for j in 0..LCOL {
            row_data[j] = self.inner[j][i];
        }
        IVector::new(row_data)
    }
    pub fn row(&self, i: usize) -> IVector<LROW> {
        self.inner[i]
    }
    pub fn row_mut(&mut self, i: usize) -> &mut IVector<LROW> {
        &mut self.inner[i]
    }
}
impl<const N: usize> IMatrix<N, N> {
    pub fn eye() -> IMatrix<N, N> {
        let mut slice = [[0i32; N]; N];
        for x in 0..N {
            for y in 0..N {
                if x == y {
                    slice[x][y] = 1;
                }
            }
        }
        IMatrix::new(slice)
    }
}
impl<const LROW: usize, const LCOL: usize, const NZ: usize> BitOr<IMatrix<NZ, LROW>>
    for IMatrix<LROW, LCOL>
{
    type Output = IMatrix<NZ, LCOL>;
    fn bitor(self, rhs: IMatrix<NZ, LROW>) -> Self::Output {
        let rhs_t = rhs.transpose();
        let inner =
            std::array::from_fn(|i| std::array::from_fn(|j| self.row(i).dot(&rhs_t.row(j))));
        IMatrix {
            inner: inner.map(IVector::new),
        }
    }
}
