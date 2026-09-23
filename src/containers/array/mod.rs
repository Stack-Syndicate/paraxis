use std::ops::{Add, Div, Mul, Sub};

use num_traits::Num;

pub struct Array<D> {
    data: Vec<D>,
    shape: Vec<usize>,
    strides: Vec<usize>,
}
impl<D: Clone> Array<D> {
    pub fn empty() -> Self {
        Self {
            data: Vec::new(),
            shape: Vec::new(),
            strides: Vec::new(),
        }
    }
    pub fn from_slice(data: &[D]) -> Self {
        Self {
            data: data.to_vec(),
            shape: vec![data.len()],
            strides: vec![1],
        }
    }
    pub fn from_vec(data: Vec<D>) -> Self {
        let data_length = data.len();
        Self {
            data,
            shape: vec![data_length],
            strides: vec![1],
        }
    }
    pub fn strides_from_shape(shape: &[usize]) -> Vec<usize> {
        let mut strides = vec![1; shape.len()];
        for i in (0..shape.len().saturating_sub(1)).rev() {
            strides[i] = strides[i + 1] * shape[i + 1];
        }
        strides
    }
    pub fn from_slice_shape(data: &[D], shape: &[usize]) -> Self {
        if shape.iter().product::<usize>() != data.len() {
            panic!("Shape does not match the total size of the data slice.")
        }
        Self {
            data: data.to_vec(),
            shape: shape.to_vec(),
            strides: Self::strides_from_shape(shape),
        }
    }
    pub fn from_vec_shape(data: Vec<D>, shape: &[usize]) -> Self {
        if shape.iter().product::<usize>() != data.len() {
            panic!("Shape does not match the total size of the data vec.")
        }
        Self {
            data,
            shape: shape.to_vec(),
            strides: Self::strides_from_shape(shape),
        }
    }
    pub fn offset(&self, indices: &[usize]) -> usize {
        // FIX: Check that the indices match the shape
        indices
            .iter()
            .zip(&self.strides)
            .map(|(index, stride)| index * stride)
            .sum()
    }
    pub fn transpose(&mut self) {
        self.strides.reverse();
        self.shape.reverse();
    }
}
impl<D: Num + Copy> Add for Array<D> {
    type Output = Array<D>;
    fn add(self, rhs: Self) -> Self::Output {
        assert_eq!(self.shape, rhs.shape); // HACK: Add a nice error message
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
        assert_eq!(self.shape, rhs.shape); // HACK: Add a nice error message
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
        assert_eq!(self.shape, rhs.shape); // HACK: Add a nice error message
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
        assert_eq!(self.shape, rhs.shape); // HACK: Add a nice error message
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
        assert_eq!(self.shape, rhs.shape); // HACK: Add a nice error message
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
        assert_eq!(self.shape, rhs.shape); // HACK: Add a nice error message
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
        assert_eq!(self.shape, rhs.shape); // HACK: Add a nice error message
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
        assert_eq!(self.shape, rhs.shape); // HACK: Add a nice error message
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

#[test]
fn add_array() {
    let v1 = Array::from_vec(vec![1, 1, 1]);
    let v2 = Array::from_vec(vec![2, 2, 2]);
    let v3 = v1 + v2;
    assert_eq!(v3.data, vec![3, 3, 3])
}
#[test]
fn sub_array() {
    let v1 = Array::from_vec(vec![2, 2, 2]);
    let v2 = Array::from_vec(vec![1, 1, 1]);
    let v3 = v1 - v2;
    assert_eq!(v3.data, vec![1, 1, 1])
}
#[test]
fn mul_array() {
    let v1 = Array::from_vec(vec![1, 2, 1]);
    let v2 = Array::from_vec(vec![2, 3, 4]);
    let v3 = v1 * v2;
    assert_eq!(v3.data, vec![2, 6, 4])
}
#[test]
fn div_array() {
    let v1 = Array::from_vec(vec![4, 4, 4]);
    let v2 = Array::from_vec(vec![2, 2, 2]);
    let v3 = v1 / v2;
    assert_eq!(v3.data, vec![2, 2, 2])
}
#[test] // TODO: Implement array + scalar test
fn add_scalar() {}
#[test] // TODO: Implement array - scalar test
fn sub_scalar() {}
#[test] // TODO: Implement array * scalar test
fn mul_scalar() {}
#[test] // TODO: Implement array / scalar test
fn div_scalar() {}
