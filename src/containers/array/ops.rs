use crate::containers::array::{Array, asserts::assert_array_shapes_eq};
use num_traits::Num;
use std::ops::{Add, Div, Mul, Neg, Sub};

impl<D: Num + Copy> Add for Array<D> {
    type Output = Array<D>;
    fn add(self, rhs: Self) -> Self::Output {
        assert_array_shapes_eq(self.len(), rhs.len());
        let data = self
            .data
            .into_iter()
            .zip(rhs.data)
            .map(|(a, b)| a.add(b))
            .collect::<Vec<D>>();
        Self {
            data,
            shape: self.shape,
            strides: self.strides,
        }
    }
}
impl<D: Num + Copy> Add<&'_ Array<D>> for &Array<D> {
    type Output = Array<D>;
    fn add(self, rhs: &'_ Array<D>) -> Self::Output {
        assert_array_shapes_eq(self.len(), rhs.len());
        let data = self
            .data
            .iter()
            .zip(rhs.data.iter())
            .map(|(a, b)| a.add(*b))
            .collect::<Vec<D>>();
        Array {
            data,
            shape: self.shape.clone(),
            strides: self.strides.clone(),
        }
    }
}
impl<D: Num + Copy> Sub for Array<D> {
    type Output = Array<D>;
    fn sub(self, rhs: Self) -> Self::Output {
        assert_array_shapes_eq(self.len(), rhs.len());
        let data = self
            .data
            .into_iter()
            .zip(rhs.data)
            .map(|(a, b)| a.sub(b))
            .collect::<Vec<D>>();
        Self {
            data,
            shape: self.shape,
            strides: self.strides,
        }
    }
}
impl<D: Num + Copy> Sub<&'_ Array<D>> for &Array<D> {
    type Output = Array<D>;
    fn sub(self, rhs: &'_ Array<D>) -> Self::Output {
        assert_array_shapes_eq(self.len(), rhs.len());
        let data = self
            .data
            .iter()
            .zip(rhs.data.iter())
            .map(|(a, b)| a.sub(*b))
            .collect::<Vec<D>>();
        Array {
            data,
            shape: self.shape.clone(),
            strides: self.strides.clone(),
        }
    }
}
impl<D: Num + Copy> Mul for Array<D> {
    type Output = Array<D>;
    fn mul(self, rhs: Self) -> Self::Output {
        assert_array_shapes_eq(self.len(), rhs.len());
        let data = self
            .data
            .into_iter()
            .zip(rhs.data)
            .map(|(a, b)| a.mul(b))
            .collect::<Vec<D>>();
        Self {
            data,
            shape: self.shape,
            strides: self.strides,
        }
    }
}
impl<D: Num + Copy> Mul<&'_ Array<D>> for &Array<D> {
    type Output = Array<D>;
    fn mul(self, rhs: &'_ Array<D>) -> Self::Output {
        assert_array_shapes_eq(self.len(), rhs.len());
        let data = self
            .data
            .iter()
            .zip(rhs.data.iter())
            .map(|(a, b)| a.mul(*b))
            .collect::<Vec<D>>();
        Array {
            data,
            shape: self.shape.clone(),
            strides: self.strides.clone(),
        }
    }
}
impl<D: Num + Copy> Div for Array<D> {
    type Output = Array<D>;
    fn div(self, rhs: Self) -> Self::Output {
        assert_array_shapes_eq(self.len(), rhs.len());
        let data = self
            .data
            .into_iter()
            .zip(rhs.data)
            .map(|(a, b)| a.div(b))
            .collect::<Vec<D>>();
        Self {
            data,
            shape: self.shape,
            strides: self.strides,
        }
    }
}
impl<D: Num + Copy> Div<&'_ Array<D>> for &Array<D> {
    type Output = Array<D>;
    fn div(self, rhs: &'_ Array<D>) -> Self::Output {
        assert_array_shapes_eq(self.len(), rhs.len());
        let data = self
            .data
            .iter()
            .zip(rhs.data.iter())
            .map(|(a, b)| a.div(*b))
            .collect::<Vec<D>>();
        Array {
            data,
            shape: self.shape.clone(),
            strides: self.strides.clone(),
        }
    }
}
impl<D: Num + Copy> Add<D> for Array<D> {
    type Output = Array<D>;
    fn add(self, rhs: D) -> Self::Output {
        let data = self.data.iter().map(|i| i.add(rhs)).collect::<Vec<_>>();
        Self {
            data,
            shape: self.shape,
            strides: self.strides,
        }
    }
}
impl<D: Num + Copy> Add<D> for &Array<D> {
    type Output = Array<D>;
    fn add(self, rhs: D) -> Self::Output {
        let data = self.data.iter().map(|i| i.add(rhs)).collect();
        Array {
            data,
            shape: self.shape.clone(),
            strides: self.strides.clone(),
        }
    }
}
impl<D: Num + Copy> Sub<D> for Array<D> {
    type Output = Array<D>;
    fn sub(self, rhs: D) -> Self::Output {
        let data = self.data.iter().map(|i| i.sub(rhs)).collect();
        Self {
            data,
            shape: self.shape,
            strides: self.strides,
        }
    }
}
impl<D: Num + Copy> Sub<D> for &Array<D> {
    type Output = Array<D>;
    fn sub(self, rhs: D) -> Self::Output {
        let data = self.data.iter().map(|i| i.sub(rhs)).collect();
        Array {
            data,
            shape: self.shape.clone(),
            strides: self.strides.clone(),
        }
    }
}
impl<D: Num + Copy> Mul<D> for Array<D> {
    type Output = Array<D>;
    fn mul(self, rhs: D) -> Self::Output {
        let data = self.data.iter().map(|i| i.mul(rhs)).collect();
        Self {
            data,
            shape: self.shape,
            strides: self.strides,
        }
    }
}
impl<D: Num + Copy> Mul<D> for &Array<D> {
    type Output = Array<D>;
    fn mul(self, rhs: D) -> Self::Output {
        let data = self.data.iter().map(|i| i.mul(rhs)).collect();
        Array {
            data,
            shape: self.shape.clone(),
            strides: self.strides.clone(),
        }
    }
}
impl<D: Num + Copy> Div<D> for Array<D> {
    type Output = Array<D>;
    fn div(self, rhs: D) -> Self::Output {
        let data = self.data.iter().map(|i| i.div(rhs)).collect();
        Self {
            data,
            shape: self.shape,
            strides: self.strides,
        }
    }
}
impl<D: Num + Copy> Div<D> for &Array<D> {
    type Output = Array<D>;
    fn div(self, rhs: D) -> Self::Output {
        let data = self.data.iter().map(|i| i.div(rhs)).collect();
        Array {
            data,
            shape: self.shape.clone(),
            strides: self.strides.clone(),
        }
    }
}
impl<D: Num + Neg<Output = D> + Copy> Neg for Array<D> {
    type Output = Array<D>;
    fn neg(self) -> Self::Output {
        let data = self.data.iter().map(|d| -*d).collect::<Vec<_>>();
        Array {
            data,
            shape: self.shape,
            strides: self.strides,
        }
    }
}
impl<D: Num + Neg<Output = D> + Copy> Neg for &Array<D> {
    type Output = Array<D>;
    fn neg(self) -> Self::Output {
        let data = self.data.iter().map(|d| -*d).collect::<Vec<_>>();
        Array {
            data,
            shape: self.shape.clone(),
            strides: self.strides.clone(),
        }
    }
}
impl<D: Num + Copy> PartialEq for Array<D> {
    fn eq(&self, other: &Self) -> bool {
        if self.shape != other.shape {
            return false;
        }
        if self.strides == other.strides {
            return self.data == other.data;
        }
        (0..self.data.len()).all(|offset| {
            let indices = Self::indices_from_offset(offset, &self.shape);
            self.data[self.offset(&indices)] == other.data[other.offset(&indices)]
        })
    }
}
