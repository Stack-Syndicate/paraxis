use std::{
    fmt::Debug,
    ops::{Add, Div, Index, IndexMut, Mul, Neg, Sub},
};

use num_traits::{Float, Num};

#[derive(Debug, Clone)]
pub struct Array<D> {
    data: Vec<D>,
    shape: Vec<usize>,
    strides: Vec<usize>,
}
impl<D> Array<D> {
    pub fn shape(&self) -> &[usize] {
        &self.shape
    }
    pub fn strides(&self) -> &[usize] {
        &self.strides
    }
    pub fn len(&self) -> usize {
        self.data.len()
    }
}
impl<D: Copy> Array<D> {
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
        assert_eq!(self.strides.len(), indices.len());
        indices
            .iter()
            .zip(&self.shape)
            .zip(&self.strides)
            .map(|((index, shape), stride)| {
                assert!(*index < *shape);
                index * stride
            })
            .sum()
    }
    pub fn transpose(mut self) -> Array<D> {
        self.strides.reverse();
        self.shape.reverse();
        self
    }
    pub fn permute_axes(self, permutation: &[usize]) -> Self {
        assert_eq!(permutation.len(), self.shape.len());
        let mut seen = vec![false; self.shape.len()];
        for &axis in permutation {
            assert!(axis < self.shape.len());
            assert!(!seen[axis]);
            seen[axis] = true;
        }
        let mut shape = vec![0; self.shape.len()];
        let mut strides = vec![0; self.strides.len()];
        for (i, p) in permutation.iter().enumerate() {
            shape[i] = self.shape[*p];
            strides[i] = self.strides[*p];
        }
        Self {
            data: self.data,
            shape,
            strides,
        }
    }
    fn strides_from_shape(shape: &[usize]) -> Vec<usize> {
        let mut strides = vec![1; shape.len()];
        for i in (0..shape.len().saturating_sub(1)).rev() {
            strides[i] = strides[i + 1] * shape[i + 1];
        }
        strides
    }
    fn indices_from_offset(mut offset: usize, shape: &[usize]) -> Vec<usize> {
        let mut indices = vec![0; shape.len()];
        for i in (0..shape.len()).rev() {
            indices[i] = offset % shape[i];
            offset /= shape[i];
        }
        indices
    }
    fn move_axis_to_end(ndim: usize, axis: usize) -> Vec<usize> {
        let mut permutation = Vec::with_capacity(ndim);
        for i in 0..ndim {
            if i != axis {
                permutation.push(i);
            }
        }
        permutation.push(axis);
        permutation
    }
    fn move_axis_to_start(ndim: usize, axis: usize) -> Vec<usize> {
        let mut permutation = Vec::with_capacity(ndim);
        permutation.push(axis);
        for i in 0..ndim {
            if i != axis {
                permutation.push(i);
            }
        }
        permutation
    }
}
impl<D: Copy> Index<usize> for Array<D> {
    type Output = D;
    fn index(&self, index: usize) -> &Self::Output {
        &self.data[index]
    }
}
impl<D: Copy> IndexMut<usize> for Array<D> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.data[index]
    }
}
impl<D: Copy, const N: usize> Index<&[usize; N]> for Array<D> {
    type Output = D;
    fn index(&self, indices: &[usize; N]) -> &Self::Output {
        &self.data[self.offset(indices)]
    }
}
impl<D: Copy, const N: usize> IndexMut<&[usize; N]> for Array<D> {
    fn index_mut(&mut self, indices: &[usize; N]) -> &mut Self::Output {
        let offset = self.offset(indices);
        &mut self.data[offset]
    }
}
impl<D: Num + Copy> Array<D> {
    pub fn identity(n: usize) -> Array<D> {
        let nxn = n * n;
        let mut data = vec![D::zero(); nxn];
        for r in 0..n {
            data[r * n + r] = D::one();
        }
        let shape = vec![n, n];
        let strides = Array::<D>::strides_from_shape(&shape);
        Array {
            data,
            shape,
            strides,
        }
    }
    pub fn contract(&self, other: &Array<D>) -> Self {
        let k = self.shape[self.shape.len() - 1];
        assert_eq!(k, other.shape[0]); // HACK: Add a nice error message
        let mut shape = Vec::new();
        shape.extend_from_slice(&self.shape[..self.shape.len() - 1]);
        shape.extend_from_slice(&other.shape[1..]);
        let size = shape.iter().product();
        let mut data = Vec::with_capacity(size);
        for offset in 0..size {
            let indices = Self::indices_from_offset(offset, &shape);
            let mut lhs_indices = vec![0usize; self.shape.len()];
            let mut rhs_indices = vec![0usize; other.shape.len()];
            let lhs_dims = self.shape.len() - 1;
            lhs_indices[..lhs_dims].copy_from_slice(&indices[..lhs_dims]);
            rhs_indices[1..].copy_from_slice(&indices[lhs_dims..]);
            let mut sum = D::zero();
            for ki in 0..self.shape[self.shape.len() - 1] {
                lhs_indices[lhs_dims] = ki;
                rhs_indices[0] = ki;
                sum = sum + self[self.offset(&lhs_indices)] * other[other.offset(&rhs_indices)];
            }
            data.push(sum);
        }
        Self {
            data,
            shape: shape.clone(),
            strides: Self::strides_from_shape(&shape),
        }
    }
    pub fn contract_axis(self, other: Array<D>, axis: usize, other_axis: usize) -> Array<D> {
        assert!(axis < self.shape.len()); // HACK: Add a nice error message
        assert!(other_axis < other.shape.len());
        assert_eq!(self.shape[axis], other.shape[other_axis]);
        let self_dims = self.shape.len();
        let other_dims = other.shape.len();
        let lhs = self.permute_axes(&Self::move_axis_to_end(self_dims, axis));
        let rhs = other.permute_axes(&Self::move_axis_to_start(other_dims, other_axis));
        lhs.contract(&rhs)
    }
    pub fn dot(&self, other: &Array<D>) -> D {
        assert!(self.shape.len() == 1); // HACK: Add a nice error message
        assert!(other.shape.len() == 1);
        assert_eq!(self.shape, other.shape);
        let result = self.contract(&other);
        result[0]
    }
    pub fn cross(&self, other: &Array<D>) -> Array<D> {
        assert!(self.shape.len() == 1); // HACK: Add a nice error message
        assert!(other.shape.len() == 1);
        assert_eq!(self.shape, other.shape);
        assert_eq!(self.shape[0], 3);
        assert_eq!(other.shape[0], 3);
        Array::from_vec(vec![
            self[1] * other[2] - self[2] * other[1],
            self[2] * other[0] - self[0] * other[2],
            self[0] * other[1] - self[1] * other[0],
        ])
    }
    pub fn sum(&self) -> D {
        assert!(!self.data.is_empty()); // HACK: Add a nice error message
        self.data.iter().fold(D::zero(), |acc, x| acc + *x)
    }
    pub fn product(&self) -> D {
        assert!(!self.data.is_empty()); // HACK: Add a nice error message
        self.data.iter().fold(D::one(), |acc, x| acc * *x)
    }
}
impl<D: PartialOrd + Copy> Array<D> {
    pub fn min(&self) -> Option<D> {
        self.data.iter().copied().fold(None, |acc, x| match acc {
            None => Some(x),
            Some(m) if x < m => Some(m),
            Some(m) => Some(m),
        })
    }
    pub fn max(&self) -> Option<D> {
        self.data.iter().copied().fold(None, |acc, x| match acc {
            None => Some(x),
            Some(m) if x > m => Some(x),
            Some(m) => Some(m),
        })
    }
}
impl<D: Float + Copy> Array<D> {
    pub fn norm(&self) -> D {
        D::sqrt(self.dot(self))
    }
    pub fn dist(&self, other: &Array<D>) -> D {
        (other - self).norm()
    }
    pub fn normalize(self) -> Array<D> {
        let norm = self.norm();
        assert!(norm > D::zero());
        self / norm
    }
    pub fn mean(&self) -> D {
        let n = D::from(self.data.len()).unwrap();
        self.data.iter().fold(D::zero(), |acc, &x| acc + x) / n
    }
    pub fn variance(&self) -> D {
        let m = self.mean();
        let n = D::from(self.data.len()).unwrap();
        self.data
            .iter()
            .fold(D::zero(), |acc, &x| acc + (x - m) * (x - m))
            / n
    }
    pub fn stddev(&self) -> D {
        self.variance().sqrt()
    }
    pub fn median(&self) -> D {
        let mut sorted = self.data.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let n = sorted.len();
        if n % 2 == 0 {
            (sorted[n / 2 - 1] + sorted[n / 2]) / D::from(2).unwrap()
        } else {
            sorted[n / 2]
        }
    }
    pub fn qr(&self) -> (Array<D>, Array<D>) {
        assert_eq!(self.shape.len(), 2);
        let m = self.shape[0];
        let n = self.shape[1];
        let mut r = self.clone();
        let mut q = Array::<D>::identity(m);
        for k in 0..n.min(m.saturating_sub(1)) {
            let mut x = Array::from_vec(vec![D::zero(); m - k]);
            for i in k..m {
                x[i - k] = r[&[i, k]];
            }
            let norm_x = x.norm();
            let alpha = if x[0] >= D::zero() { -norm_x } else { norm_x };
            let mut v = x.clone();
            v[0] = v[0] - alpha;
            v = v.normalize();
            if v.norm() <= D::zero() {
                continue;
            }
            for j in k..n {
                let mut dot = D::zero();
                for i in k..m {
                    dot = dot + v[i - k] * r[&[i, j]];
                }
                let factor = dot + dot;
                for i in k..m {
                    let updated = r[&[i, j]] - factor * v[i - k];
                    r[&[i, j]] = updated;
                }
            }
            for i in 0..m {
                let mut dot = D::zero();
                for j in k..m {
                    dot = dot + q[&[i, j]] * v[j - k];
                }
                let factor = dot + dot;
                for j in k..m {
                    let updated = q[&[i, j]] - factor * v[j - k];
                    q[&[i, j]] = updated;
                }
            }
        }
        (q, r)
    }
    pub fn eigvals(&self) -> Array<D> {
        assert_eq!(self.shape.len(), 2);
        assert_eq!(self.shape[0], self.shape[1]);
        let n = self.shape[0];
        let mut a = self.clone();
        let identity = Array::identity(n);
        for _ in 0..100 {
            let mu = a[&[n - 1, n - 1]];
            let shifted = &a - &(&identity * mu);
            let (q, r) = shifted.qr();
            a = &r.contract(&q) + &(&identity * mu);
            let mut off_diagonal_max = D::zero();
            for i in 0..n {
                for j in 0..n {
                    if i != j {
                        let val = a[&[i, j]].abs();
                        if val > off_diagonal_max {
                            off_diagonal_max = val;
                        }
                    }
                }
            }
            if off_diagonal_max < D::from(1e-9).unwrap() {
                break;
            }
        }
        let mut eigenvalues = vec![D::zero(); n];
        for i in 0..n {
            eigenvalues[i] = a[&[i, i]];
        }
        Array::from_vec(eigenvalues)
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
#[test]
fn add_scalar() {
    let v = Array::from_vec(vec![1, 1, 1]);
    let s = 10;
    let r = v + s;
    assert_eq!(r.data, vec![11, 11, 11])
}
#[test]
fn sub_scalar() {
    let v = Array::from_vec(vec![1, 1, 1]);
    let s = 10;
    let r = v - s;
    assert_eq!(r.data, vec![-9, -9, -9])
}
#[test]
fn mul_scalar() {
    let v = Array::from_vec(vec![1, 1, 1]);
    let s = 10;
    let r = v * s;
    assert_eq!(r.data, vec![10, 10, 10])
}
#[test]
fn div_scalar() {
    let v = Array::from_vec(vec![1.0, 1.0, 1.0]);
    let s = 10.0;
    let r = v / s;
    assert_eq!(r.data, vec![0.1, 0.1, 0.1])
}
#[test]
fn dot_product() {
    let v1 = Array::from_vec(vec![1, 1, 1]);
    let v2 = Array::from_vec(vec![1, 1, 1]);
    let r1 = v1.dot(&v2);
    assert_eq!(r1, 3);
}
#[test]
fn negation() {
    let v = Array::from_vec(vec![1, 1, 1]);
    let r = -v;
    assert_eq!(r.data, vec![-1, -1, -1]);
}
#[test]
fn qr_decomposition() {
    let a = Array::from_vec_shape(
        vec![12.0, -51.0, 4.0, 6.0, 167.0, -68.0, -4.0, 24.0, -41.0],
        &[3, 3],
    );
    let (q, r) = a.qr();
    // Q should be orthogonal: Q^T * Q ≈ I
    let qt = q.clone().transpose();
    let qtq = qt.contract(&q);
    let identity = Array::<f64>::identity(3);
    for i in 0..3 {
        for j in 0..3 {
            let diff = (qtq[&[i, j]] - identity[&[i, j]]).abs();
            assert!(diff < 1e-9,);
        }
    }
    // R should be upper triangular: entries below the diagonal ≈ 0
    for i in 0..3 {
        for j in 0..i {
            let val = r[&[i, j]];
            assert!(val.abs() < 1e-9,);
        }
    }
    // Q * R should reconstruct A
    let qr = q.contract(&r);
    for i in 0..3 {
        for j in 0..3 {
            let diff = (qr[&[i, j]] - a[&[i, j]]).abs();
            assert!(diff < 1e-9,);
        }
    }
}
#[test]
fn eigenvalues_symmetric() {
    let a = Array::from_vec_shape(vec![4.0, 1.0, 1.0, 1.0, 4.0, 1.0, 1.0, 1.0, 4.0], &[3, 3]);
    let eigenvalues = a.eigvals();
    let mut computed: Vec<f64> = eigenvalues.data.clone();
    computed.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let mut expected = vec![3.0, 3.0, 6.0];
    expected.sort_by(|a, b| a.partial_cmp(b).unwrap());
    for (c, e) in computed.iter().zip(expected.iter()) {
        assert!((c - e).abs() < 1e-9);
    }
}
