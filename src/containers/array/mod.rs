pub mod index;
pub mod ops;

use std::fmt::Debug;

use num_traits::{Float, Num};

#[derive(Debug)]
pub struct EigenResult<D> {
    pub values: Array<D>,
    pub vectors: Array<D>,
    pub converged: bool,
    pub iterations: usize,
}

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
    pub fn to_vec(self) -> Vec<D> {
        self.data
    }
}
impl<D: Copy> Array<D> {
    pub fn column(&self, column: usize) -> Array<D> {
        assert_eq!(self.shape.len(), 2);
        let m = self.shape[0];
        let mut data = Vec::with_capacity(m);
        for i in 0..m {
            data.push(self[&[i, column]]);
        }
        Array::from_vec(data)
    }
    pub fn to_cloned_vec(&self) -> Vec<D> {
        self.data.clone()
    }
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
impl<D: Num + Copy> Array<D> {
    pub fn trace(&self) -> D {
        assert_eq!(self.shape.len(), 2); // HACK: Add a nice error message
        assert_eq!(self.shape[0], self.shape[1]);
        let n = self.shape[0];
        let mut trace = D::zero();
        for i in 0..n {
            trace = trace + self[&[i, i]];
        }
        trace
    }
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
    pub fn eigen(&self) -> EigenResult<D> {
        assert_eq!(self.shape.len(), 2);
        assert_eq!(self.shape[0], self.shape[1]);
        let n = self.shape[0];
        let mut a = self.clone();
        let identity = Array::identity(n);
        let mut eigenvectors = identity.clone();
        let mut converged = false;
        let mut iterations = 0;
        for i in 0..100 {
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
                converged = true;
                break;
            }
            eigenvectors = eigenvectors.contract(&q);
            iterations = i;
        }
        let mut eigenvalues = vec![D::zero(); n];
        for i in 0..n {
            eigenvalues[i] = a[&[i, i]];
        }
        let eigenvalues = Array::from_vec(eigenvalues);
        EigenResult {
            values: eigenvalues,
            vectors: eigenvectors,
            converged,
            iterations,
        }
    }
    pub fn solve(&self, b: &Array<D>) -> Array<D> {
        assert_eq!(self.shape.len(), 2);
        assert_eq!(self.shape[0], self.shape[1]);
        assert_eq!(b.shape.len(), 1);
        assert_eq!(self.shape[0], b.shape[0]);
        let (q, r) = self.qr();
        let n = r.shape[0];
        let qt = q.clone().transpose();
        let qtb = qt.contract(b);
        let mut x = vec![D::zero(); n];
        for i in (0..n).rev() {
            let diag = r[&[i, i]];
            assert!(diag.abs() > D::from(1e-12).unwrap());
            let mut sum = qtb[i];
            for j in (i + 1)..n {
                sum = sum - r[&[i, j]] * x[j];
            }
            x[i] = sum / diag;
        }
        Array::from_vec(x)
    }
}
