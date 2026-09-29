//! This module holds the main tensor/vector abstraction [`Array`]

mod index;
mod iter;
mod ops;
use num_complex::ComplexFloat;
use num_traits::{Float, Num, NumCast, One, Zero};
use std::fmt::Debug;

/// Stores the result of [`Array::eigen`]
#[derive(Debug, Clone)]
pub struct EigenResult<D> {
    /// Eigenvalues
    pub values: Array<D>,
    /// Eigenvectors
    pub vectors: Array<D>,
    /// Tracks whether or not [`Array::eigen`] had successfully converged
    pub converged: bool,
    /// Tracks the number of iterations that [`Array::eigen`] performed before constructing this result
    pub iterations: usize,
}
/// Stores the result of [`Array::qr`]
#[derive(Debug, Clone)]
pub struct QrResult<D> {
    /// Orthonormal matrix Q
    pub q: Array<D>,
    /// Upper triangular matrix R
    pub r: Array<D>,
    /// Column permutation applied by pivoting, `permutation[k]` is the original index of column `k`
    pub permutation: Vec<usize>,
    /// Number of Householder reflections applied
    pub reflection_count: usize,
}
/// Stores the result of [`Array::lu`]
#[derive(Debug, Clone)]
pub struct LuResult<D> {
    /// Unit lower triangular matrix L
    pub l: Array<D>,
    /// Upper triangular matrix U
    pub u: Array<D>,
    /// Row permutation, row `i` of the permuted matrix is row `pivots[i]` of the original
    pub pivots: Vec<usize>,
    /// Number of row swaps performed
    pub swap_count: usize,
}
/// Stores the result of [`Array::svd`]
#[derive(Debug, Clone)]
pub struct SvdResult<D: ComplexFloat> {
    /// Left singular vectors, stored as columns
    pub u: Array<D>,
    /// Singular values in descending order
    pub s: Array<D::Real>,
    /// Right singular vectors, stored as columns
    pub v: Array<D>,
    /// Tracks whether or not [`Array::svd`] had successfully converged
    pub converged: bool,
    /// Tracks the number of sweeps that [`Array::svd`] performed
    pub iterations: usize,
}
/// N-dimensional array of `D`, stored in row-major order with explicit strides
#[derive(Debug, Clone)]
pub struct Array<D> {
    data: Vec<D>,
    shape: Vec<usize>,
    strides: Vec<usize>,
}
impl<D> Array<D> {
    /// Returns the size of each dimension
    pub fn shape(&self) -> &[usize] {
        &self.shape
    }
    /// Returns the stride of each dimension
    pub fn strides(&self) -> &[usize] {
        &self.strides
    }
    /// Returns the total number of elements
    pub fn len(&self) -> usize {
        self.data.len()
    }
    /// Consumes the array and returns its underlying storage
    pub fn to_vec(self) -> Vec<D> {
        self.data
    }
}
impl<D: Copy> Array<D> {
    /// Copies the block `[row_start, row_end) x [col_start, col_end)` into a new matrix
    ///
    /// Panics if the array is not 2D or the range is out of bounds
    pub fn submatrix(
        &self,
        row_start: usize,
        row_end: usize,
        col_start: usize,
        col_end: usize,
    ) -> Array<D> {
        assert_eq!(self.shape.len(), 2);
        assert!(row_start <= row_end && row_end <= self.shape[0]);
        assert!(col_start <= col_end && col_end <= self.shape[1]); // HACK: Add a nice error message
        let rows = row_end - row_start;
        let cols = col_end - col_start;
        let mut data = Vec::with_capacity(rows * cols);
        for i in row_start..row_end {
            for j in col_start..col_end {
                data.push(self[&[i, j]]);
            }
        }
        Array::from_vec_shape(data, &[rows, cols])
    }
    /// Overwrites the block starting at `(row_start, col_start)` with `value`
    ///
    /// Panics if either array is not 2D or the block does not fit
    pub fn set_submatrix(&mut self, row_start: usize, col_start: usize, value: &Array<D>) {
        assert_eq!(self.shape.len(), 2);
        assert_eq!(value.shape.len(), 2);
        let (rows, cols) = (value.shape[0], value.shape[1]);
        assert!(row_start + rows <= self.shape[0]);
        assert!(col_start + cols <= self.shape[1]); // HACK: Add a nice error message
        for i in 0..rows {
            for j in 0..cols {
                self[&[row_start + i, col_start + j]] = value[&[i, j]];
            }
        }
    }
    /// Returns column `column` of a matrix as a vector
    ///
    /// Panics if the array is not 2D
    pub fn column(&self, column: usize) -> Array<D> {
        assert_eq!(self.shape.len(), 2); // HACK: Add a nice error message
        let m = self.shape[0];
        let mut data = Vec::with_capacity(m);
        for i in 0..m {
            data.push(self[&[i, column]]);
        }
        Array::from_vec(data)
    }
    /// Clones the underlying storage into a `Vec`
    pub fn to_cloned_vec(&self) -> Vec<D> {
        self.data.clone()
    }
    /// Creates an array with no elements and no dimensions
    pub fn empty() -> Self {
        Self {
            data: Vec::new(),
            shape: Vec::new(),
            strides: Vec::new(),
        }
    }
    /// Creates a 1D array by copying `data`
    pub fn from_slice(data: &[D]) -> Self {
        Self {
            data: data.to_vec(),
            shape: vec![data.len()],
            strides: vec![1],
        }
    }
    /// Creates a 1D array from `data`
    pub fn from_vec(data: Vec<D>) -> Self {
        let data_length = data.len();
        Self {
            data,
            shape: vec![data_length],
            strides: vec![1],
        }
    }
    /// Creates an array of the given shape by copying `data`
    ///
    /// Panics if the product of `shape` does not equal `data.len()`
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
    /// Creates an array of the given shape from `data`
    ///
    /// Panics if the product of `shape` does not equal `data.len()`
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
    /// Converts multi-dimensional `indices` into an offset into the underlying storage
    ///
    /// Panics if `indices` has the wrong length or is out of bounds
    pub fn offset(&self, indices: &[usize]) -> usize {
        assert_eq!(self.strides.len(), indices.len()); // HACK: Add a nice error message
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
    /// Reverses the order of the axes without copying data
    pub fn transpose(mut self) -> Array<D> {
        self.strides.reverse();
        self.shape.reverse();
        self
    }
    /// Reorders the axes so that axis `i` of the result is axis `permutation[i]` of `self`
    ///
    /// Panics if `permutation` is not a valid permutation of the axes
    pub fn permute_axes(self, permutation: &[usize]) -> Self {
        assert_eq!(permutation.len(), self.shape.len()); // HACK: Add a nice error message
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
    /// Swaps the elements at indices `a` and `b`
    ///
    /// Panics if `a` or `b` does not have one index per axis
    pub fn swap_elements(&mut self, a: &[usize], b: &[usize]) {
        assert_eq!(a.len(), self.shape.len());
        assert_eq!(b.len(), self.shape.len()); // HACK: Add a nice error message
        let a_offset = a
            .iter()
            .zip(&self.strides)
            .map(|(&i, &stride)| i * stride)
            .sum::<usize>();
        let b_offset = b
            .iter()
            .zip(&self.strides)
            .map(|(&i, &stride)| i * stride)
            .sum::<usize>();
        self.data.swap(a_offset, b_offset);
    }
    /// Swaps rows `a` and `b` of a matrix
    ///
    /// Panics if the array has fewer than 2 dimensions or a row is out of bounds
    pub fn swap_rows(&mut self, a: usize, b: usize) {
        assert!(self.shape.len() >= 2);
        assert!(a < self.shape[0]);
        assert!(b < self.shape[0]); // HACK: Add a nice error message
        for j in 0..self.shape[1] {
            self.swap_elements(&[a, j], &[b, j]);
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
    /// Returns the sum of the diagonal of a square matrix
    ///
    /// Panics if the array is not a square matrix
    pub fn trace(&self) -> D {
        assert_eq!(self.shape.len(), 2);
        assert_eq!(self.shape[0], self.shape[1]); // HACK: Add a nice error message
        let n = self.shape[0];
        let mut trace = D::zero();
        for i in 0..n {
            trace = trace + self[&[i, i]];
        }
        trace
    }
    /// Creates an `n x n` identity matrix
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
    /// Contracts the last axis of `self` with the first axis of `other`
    ///
    /// Panics if the contracted axes differ in size
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
    /// Contracts `axis` of `self` with `other_axis` of `other`
    ///
    /// Panics if an axis is out of bounds or the contracted axes differ in size
    pub fn contract_axis(self, other: Array<D>, axis: usize, other_axis: usize) -> Array<D> {
        assert!(axis < self.shape.len());
        assert!(other_axis < other.shape.len());
        assert_eq!(self.shape[axis], other.shape[other_axis]); // HACK: Add a nice error message
        let self_dims = self.shape.len();
        let other_dims = other.shape.len();
        let lhs = self.permute_axes(&Self::move_axis_to_end(self_dims, axis));
        let rhs = other.permute_axes(&Self::move_axis_to_start(other_dims, other_axis));
        lhs.contract(&rhs)
    }
    /// Returns the cross product of two 3-element vectors
    ///
    /// Panics if either array is not a vector of length 3
    pub fn cross(&self, other: &Array<D>) -> Array<D> {
        assert!(self.shape.len() == 1);
        assert!(other.shape.len() == 1);
        assert_eq!(self.shape, other.shape);
        assert_eq!(self.shape[0], 3);
        assert_eq!(other.shape[0], 3); // HACK: Add a nice error message
        Array::from_vec(vec![
            self[1] * other[2] - self[2] * other[1],
            self[2] * other[0] - self[0] * other[2],
            self[0] * other[1] - self[1] * other[0],
        ])
    }
    /// Returns the sum of all elements
    ///
    /// Panics if the array is empty
    pub fn sum(&self) -> D {
        assert!(!self.data.is_empty()); // HACK: Add a nice error message
        self.data.iter().fold(D::zero(), |acc, x| acc + *x)
    }
    /// Returns the product of all elements
    ///
    /// Panics if the array is empty
    pub fn product(&self) -> D {
        assert!(!self.data.is_empty()); // HACK: Add a nice error message
        self.data.iter().fold(D::one(), |acc, x| acc * *x)
    }
}
impl<D: PartialOrd + Copy> Array<D> {
    /// Returns the smallest element, or `None` if the array is empty
    pub fn min(&self) -> Option<D> {
        self.data.iter().copied().fold(None, |acc, x| match acc {
            None => Some(x),
            Some(m) if x < m => Some(x),
            Some(m) => Some(m),
        })
    }
    /// Returns the largest element, or `None` if the array is empty
    pub fn max(&self) -> Option<D> {
        self.data.iter().copied().fold(None, |acc, x| match acc {
            None => Some(x),
            Some(m) if x > m => Some(x),
            Some(m) => Some(m),
        })
    }
}
impl<D> Array<D>
where
    D: ComplexFloat + NumCast,
    D::Real: Float,
{
    #[inline]
    fn real(x: f64) -> D::Real {
        NumCast::from(x).unwrap()
    }
    #[inline]
    fn scalar(x: D::Real) -> D {
        NumCast::from(x).unwrap()
    }
    /// Returns the inner product of `self` and `other`, conjugating `other`
    pub fn dot(&self, other: &Array<D>) -> D {
        // FIXME: Add some assertions
        self.data
            .iter()
            .zip(other.data.iter())
            .fold(D::zero(), |acc, (&a, &b)| acc + a * b.conj())
    }
    /// Returns the Euclidean norm
    pub fn norm(&self) -> D::Real {
        Float::sqrt(self.dot(self).re())
    }
    /// Returns the Euclidean distance between `self` and `other`
    pub fn dist(&self, other: &Array<D>) -> D::Real {
        // FIXME: Add some assertions
        (other - self).norm()
    }
    /// Scales the array to unit norm
    ///
    /// Panics if the norm is zero
    pub fn normalize(self) -> Array<D> {
        let norm = self.norm();
        assert!(norm > D::Real::zero()); // HACK: Add a nice error message
        self / Self::scalar(norm)
    }
    /// Returns the mean of all elements
    pub fn mean(&self) -> D {
        // FIXME: Add some assertions/early exits
        let n: D = NumCast::from(self.data.len()).unwrap();
        self.data.iter().fold(D::zero(), |acc, &x| acc + x) / n
    }
    /// Returns the population variance
    pub fn variance(&self) -> D::Real {
        // FIXME: Add some assertions/early exits
        let m = self.mean();
        let n = Self::real(self.data.len() as f64);
        self.data.iter().fold(D::Real::zero(), |acc, &x| {
            let diff = x - m;
            acc + (diff * diff.conj()).re()
        }) / n
    }
    /// Returns the population standard deviation
    pub fn stddev(&self) -> D::Real {
        Float::sqrt(self.variance())
    }
    /// Returns the conjugate transpose
    pub fn conj_transpose(self) -> Array<D> {
        // FIXME: Add an early exit
        let t = self.transpose();
        let mut data = Vec::with_capacity(t.data.len());
        for offset in 0..t.data.len() {
            let indices = Self::indices_from_offset(offset, &t.shape);
            data.push(t.data[t.offset(&indices)].conj());
        }
        let shape = t.shape.clone();
        Array::from_vec_shape(data, &shape)
    }
    /// Computes the QR decomposition using Householder reflections with column pivoting
    ///
    /// Panics if the array is not 2D
    pub fn qr(&self) -> QrResult<D> {
        self.qr_impl(true)
    }
    fn qr_impl(&self, pivoting: bool) -> QrResult<D> {
        assert_eq!(self.shape.len(), 2); // HACK: Add a nice error message
        let threshold = Float::sqrt(<D::Real as Float>::epsilon());
        let m = self.shape[0];
        let n = self.shape[1];
        let steps = m.min(n);
        let mut work = vec![D::zero(); m * n + m * m + m + 2 * steps];
        let (cols, work) = work.split_at_mut(m * n);
        let (qcols, work) = work.split_at_mut(m * m);
        let (v, work) = work.split_at_mut(m);
        let (taus, betas) = work.split_at_mut(steps);
        let mut vn = vec![D::Real::zero(); if pivoting { 2 * n } else { 0 }];
        let vn_len = vn.len();
        let (vn1, vn2) = vn.split_at_mut(vn_len / 2);
        let mut permutation = (0..n).collect::<Vec<_>>();
        for i in 0..m {
            for j in 0..n {
                cols[j * m + i] = self[&[i, j]];
            }
        }
        for i in 0..m {
            qcols[i * m + i] = D::one();
        }
        let norm_sq = |col: &[D]| -> D::Real {
            let mut s0 = D::Real::zero();
            let mut s1 = D::Real::zero();
            let mut s2 = D::Real::zero();
            let mut s3 = D::Real::zero();
            let chunks = col.chunks_exact(4);
            let rem = chunks.remainder();
            for c in chunks {
                s0 = s0 + (c[0].re() * c[0].re() + c[0].im() * c[0].im());
                s1 = s1 + (c[1].re() * c[1].re() + c[1].im() * c[1].im());
                s2 = s2 + (c[2].re() * c[2].re() + c[2].im() * c[2].im());
                s3 = s3 + (c[3].re() * c[3].re() + c[3].im() * c[3].im());
            }
            for &x in rem {
                s0 = s0 + (x.re() * x.re() + x.im() * x.im());
            }
            (s0 + s1) + (s2 + s3)
        };
        let reflect = |block: &mut [D], v: &[D], tau: D, k: usize| {
            let len = v.len();
            let mut quads = block.chunks_exact_mut(4 * m);
            for quad in &mut quads {
                let (c0, tail) = quad.split_at_mut(m);
                let (c1, tail) = tail.split_at_mut(m);
                let (c2, c3) = tail.split_at_mut(m);
                let c0 = &mut c0[k..k + len];
                let c1 = &mut c1[k..k + len];
                let c2 = &mut c2[k..k + len];
                let c3 = &mut c3[k..k + len];
                let mut d0 = D::zero();
                let mut d1 = D::zero();
                let mut d2 = D::zero();
                let mut d3 = D::zero();
                for i in 0..len {
                    let w = v[i].conj();
                    d0 = d0 + w * c0[i];
                    d1 = d1 + w * c1[i];
                    d2 = d2 + w * c2[i];
                    d3 = d3 + w * c3[i];
                }
                let t0 = tau * d0;
                let t1 = tau * d1;
                let t2 = tau * d2;
                let t3 = tau * d3;
                for i in 0..len {
                    let w = v[i];
                    c0[i] = c0[i] - t0 * w;
                    c1[i] = c1[i] - t1 * w;
                    c2[i] = c2[i] - t2 * w;
                    c3[i] = c3[i] - t3 * w;
                }
            }
            for col in quads.into_remainder().chunks_exact_mut(m) {
                let col = &mut col[k..k + len];
                let mut d = D::zero();
                for i in 0..len {
                    d = d + v[i].conj() * col[i];
                }
                let t = tau * d;
                for i in 0..len {
                    col[i] = col[i] - t * v[i];
                }
            }
        };
        if pivoting {
            for j in 0..n {
                let s = norm_sq(&cols[j * m..(j + 1) * m]);
                vn1[j] = s;
                vn2[j] = s;
            }
        }
        let mut reflection_count = 0;
        for k in 0..steps {
            if pivoting {
                let mut pivot = k;
                let mut pivot_norm = vn1[k];
                for j in (k + 1)..n {
                    if vn1[j] > pivot_norm {
                        pivot = j;
                        pivot_norm = vn1[j];
                    }
                }
                if pivot != k {
                    let (head, tail) = cols.split_at_mut(pivot * m);
                    head[k * m..(k + 1) * m].swap_with_slice(&mut tail[..m]);
                    vn1.swap(k, pivot);
                    vn2.swap(k, pivot);
                    permutation.swap(k, pivot);
                }
            }
            let x_norm = Float::sqrt(norm_sq(&cols[k * m + k..(k + 1) * m]));
            if x_norm <= D::Real::zero() {
                continue;
            }
            let x0 = cols[k * m + k];
            let x0_abs = ComplexFloat::abs(x0);
            let v_half = x_norm * (x_norm + x0_abs);
            if v_half <= D::Real::zero() {
                continue;
            }
            let alpha = if x0_abs > D::Real::zero() {
                -(x0 * Self::scalar(x_norm / x0_abs))
            } else {
                -Self::scalar(x_norm)
            };
            let tau = Self::scalar(D::Real::one() / v_half);
            v[k] = x0 - alpha;
            v[k + 1..m].copy_from_slice(&cols[k * m + k + 1..(k + 1) * m]);
            reflection_count += 1;
            taus[k] = tau;
            betas[k] = v[k];
            cols[k * m + k] = alpha;
            reflect(&mut cols[(k + 1) * m..], &v[k..m], tau, k);
            if pivoting {
                for j in (k + 1)..n {
                    let old = vn1[j];
                    if old <= D::Real::zero() {
                        continue;
                    }
                    let top = cols[j * m + k];
                    let updated =
                        (old - (top.re() * top.re() + top.im() * top.im())).max(D::Real::zero());
                    if updated <= threshold * vn2[j] {
                        let exact = norm_sq(&cols[j * m + k + 1..(j + 1) * m]);
                        vn1[j] = exact;
                        vn2[j] = exact;
                    } else {
                        vn1[j] = updated;
                    }
                }
            }
        }
        for k in (0..steps).rev() {
            if taus[k] == D::zero() {
                continue;
            }
            let tau = taus[k];
            v[k] = betas[k];
            v[k + 1..m].copy_from_slice(&cols[k * m + k + 1..(k + 1) * m]);
            let factor = tau * v[k].conj();
            let col = &mut qcols[k * m..(k + 1) * m];
            col[k] = D::one() - factor * v[k];
            for i in (k + 1)..m {
                col[i] = D::zero() - factor * v[i];
            }
            reflect(&mut qcols[(k + 1) * m..], &v[k..m], tau, k);
        }
        let mut r_data = vec![D::zero(); m * n];
        for j in 0..n {
            for i in 0..(j + 1).min(steps) {
                r_data[i * n + j] = cols[j * m + i];
            }
        }
        let mut q_data = vec![D::zero(); m * m];
        for jb in (0..m).step_by(8) {
            for ib in (0..m).step_by(8) {
                for j in jb..(jb + 8).min(m) {
                    for i in ib..(ib + 8).min(m) {
                        q_data[i * m + j] = qcols[j * m + i];
                    }
                }
            }
        }
        let r = Array::from_vec_shape(r_data, &[m, n]);
        let q = Array::from_vec_shape(q_data, &[m, m]);
        QrResult {
            q,
            r,
            permutation,
            reflection_count,
        }
    }
    /// Computes the LU decomposition with partial pivoting
    ///
    /// Panics if the array is not a square matrix
    pub fn lu(&self) -> LuResult<D> {
        assert_eq!(self.shape.len(), 2);
        assert_eq!(self.shape[0], self.shape[1]); // HACK: Add a nice error message
        let n = self.shape[0];
        let mut l = Array::identity(n);
        let mut u = self.clone();
        let mut pivots = (0..n).collect::<Vec<_>>();
        let mut swap_count = 0;
        for k in 0..n {
            let mut pivot = k;
            for i in (k + 1)..n {
                if ComplexFloat::abs(u[&[i, k]]) > ComplexFloat::abs(u[&[pivot, k]]) {
                    pivot = i;
                }
            }
            if pivot != k {
                u.swap_rows(k, pivot);
                pivots.swap(k, pivot);
                for j in 0..k {
                    l.swap_elements(&[k, j], &[pivot, j]);
                }
                swap_count += 1;
            }
            for i in (k + 1)..n {
                l[&[i, k]] = u[&[i, k]] / u[&[k, k]];
                for j in (k + 1)..n {
                    u[&[i, j]] = u[&[i, j]] - l[&[i, k]] * u[&[k, j]];
                }
                u[&[i, k]] = D::zero();
            }
        }
        LuResult {
            l,
            u,
            pivots,
            swap_count,
        }
    }
    fn solve_with_lu(&self, b: &Array<D>, lu: &LuResult<D>) -> Array<D> {
        assert_eq!(self.shape.len(), 2);
        assert_eq!(self.shape[0], self.shape[1]);
        assert_eq!(b.shape.len(), 1);
        assert_eq!(b.shape[0], self.shape[0]); // HACK: Add a nice error message
        let n = self.shape[0];
        let (l, u, pivots) = (&lu.l, &lu.u, &lu.pivots);
        let mut y = vec![D::zero(); n];
        for i in 0..n {
            let mut sum = b[pivots[i]];
            for j in 0..i {
                sum = sum - l[&[i, j]] * y[j];
            }
            y[i] = sum / l[&[i, i]];
        }
        let mut x = vec![D::zero(); n];
        for i in (0..n).rev() {
            let diag = u[&[i, i]];
            assert!(ComplexFloat::abs(diag) > Self::real(1e-12));
            let mut sum = y[i];
            for j in (i + 1)..n {
                sum = sum - u[&[i, j]] * x[j];
            }
            x[i] = sum / diag;
        }
        Array::from_vec(x)
    }
    /// Solves `self * x = b` for `x` using LU decomposition
    ///
    /// Panics if `self` is not square, `b` is not a matching vector, or `self` is singular
    pub fn solve(&self, b: &Array<D>) -> Array<D> {
        let lu = self.lu();
        self.solve_with_lu(b, &lu)
    }
    /// Returns the numerical rank, estimated from the diagonal of R in [`Array::qr`]
    ///
    /// Panics if the array is not 2D
    pub fn rank(&self) -> usize {
        assert_eq!(self.shape.len(), 2); // HACK: Add a nice error message
        let qr = self.qr();
        let n = qr.r.shape[0].min(qr.r.shape[1]);
        let scale = (0..n)
            .map(|i| ComplexFloat::abs(qr.r[&[i, i]]))
            .fold(D::Real::zero(), |a, b| a.max(b));
        let tol = scale * Self::real(1e-12);
        (0..n)
            .filter(|&i| ComplexFloat::abs(qr.r[&[i, i]]) > tol)
            .count()
    }
    /// Returns the determinant of a square matrix
    ///
    /// Panics if the array is not a square matrix
    pub fn det(&self) -> D {
        assert_eq!(self.shape.len(), 2);
        assert_eq!(self.shape[0], self.shape[1]);
        let lu = self.lu();
        let mut det = D::one();
        for i in 0..self.shape[0] {
            det = det * lu.u[&[i, i]];
        }
        if lu.swap_count % 2 == 1 {
            det = -det;
        }
        det
    }
    /// Returns the inverse of a square matrix
    ///
    /// Panics if the array is not square or is singular
    pub fn inverse(&self) -> Array<D> {
        assert_eq!(self.shape.len(), 2);
        assert_eq!(self.shape[0], self.shape[1]);
        let n = self.shape[0];
        let lu = self.lu();
        let mut data = vec![D::zero(); n * n];
        for j in 0..n {
            let mut e_j = vec![D::zero(); n];
            e_j[j] = D::one();
            let x = self.solve_with_lu(&Array::from_vec(e_j), &lu);
            for i in 0..n {
                data[i * n + j] = x[i];
            }
        }
        Array::from_vec_shape(data, &[n, n])
    }
    /// Returns the Euclidean norm of each column of a matrix
    ///
    /// Panics if the array is not 2D
    pub fn column_norms(&self) -> Vec<D::Real> {
        assert_eq!(self.shape.len(), 2);
        let (rows, cols) = (self.shape[0], self.shape[1]);
        (0..cols)
            .map(|j| {
                let norm_sq = (0..rows).fold(D::Real::zero(), |acc, i| {
                    let x = self[&[i, j]];
                    acc + (x * x.conj()).re()
                });
                Float::sqrt(norm_sq)
            })
            .collect()
    }
    /// Scales each column of a matrix to unit norm, returning the result and the original column norms
    ///
    /// Columns with a norm of at most `zero_floor` are set to zero.
    ///
    /// Panics if the array is not 2D
    pub fn normalize_columns(&self, zero_floor: D::Real) -> (Array<D>, Vec<D::Real>) {
        assert_eq!(self.shape.len(), 2);
        let (rows, cols) = (self.shape[0], self.shape[1]);
        let norms = self.column_norms();
        let mut data = vec![D::zero(); rows * cols];
        for j in 0..cols {
            if norms[j] > zero_floor {
                let norm_d = Self::scalar(norms[j]);
                for i in 0..rows {
                    data[i * cols + j] = self[&[i, j]] / norm_d;
                }
            }
        }
        (Array::from_vec_shape(data, &[rows, cols]), norms)
    }
    /// Computes the eigenvalues and eigenvectors of a square matrix using the shifted QR algorithm
    ///
    /// Panics if the array is not a square matrix
    pub fn eigen(&self) -> EigenResult<D> {
        assert_eq!(self.shape.len(), 2);
        assert_eq!(self.shape[0], self.shape[1]);
        let n = self.shape[0];
        let mut a = vec![D::zero(); n * n];
        let mut scale_sq = D::Real::zero();
        for i in 0..n {
            for j in 0..n {
                let x = self[&[i, j]];
                a[i * n + j] = x;
                scale_sq = scale_sq.max(x.re() * x.re() + x.im() * x.im());
            }
        }
        let scale = Float::sqrt(scale_sq);
        let tol = scale * Self::real(1e-12);
        let max_total_iterations = 200 * n.max(1);
        let reflect = |block: &mut [D], v: &[D], tau: D, k: usize| {
            let len = v.len();
            let mut quads = block.chunks_exact_mut(4 * n);
            for quad in &mut quads {
                let (c0, tail) = quad.split_at_mut(n);
                let (c1, tail) = tail.split_at_mut(n);
                let (c2, c3) = tail.split_at_mut(n);
                let c0 = &mut c0[k..k + len];
                let c1 = &mut c1[k..k + len];
                let c2 = &mut c2[k..k + len];
                let c3 = &mut c3[k..k + len];
                let mut d0 = D::zero();
                let mut d1 = D::zero();
                let mut d2 = D::zero();
                let mut d3 = D::zero();
                for i in 0..len {
                    let w = v[i].conj();
                    d0 = d0 + w * c0[i];
                    d1 = d1 + w * c1[i];
                    d2 = d2 + w * c2[i];
                    d3 = d3 + w * c3[i];
                }
                let t0 = tau * d0;
                let t1 = tau * d1;
                let t2 = tau * d2;
                let t3 = tau * d3;
                for i in 0..len {
                    let w = v[i];
                    c0[i] = c0[i] - t0 * w;
                    c1[i] = c1[i] - t1 * w;
                    c2[i] = c2[i] - t2 * w;
                    c3[i] = c3[i] - t3 * w;
                }
            }
            for col in quads.into_remainder().chunks_exact_mut(n) {
                let col = &mut col[k..k + len];
                let mut d = D::zero();
                for i in 0..len {
                    d = d + v[i].conj() * col[i];
                }
                let t = tau * d;
                for i in 0..len {
                    col[i] = col[i] - t * v[i];
                }
            }
        };
        let rotate = |a: &mut [D], vecs: &mut [D], j: usize, low: usize, c: D, s: D| {
            let cc = c.conj();
            let sc = s.conj();
            for r in low..=j + 1 {
                let p = a[r * n + j];
                let q = a[r * n + j + 1];
                a[r * n + j] = p * c + q * s;
                a[r * n + j + 1] = q * cc - p * sc;
            }
            let (left, right) = vecs[j * n..(j + 2) * n].split_at_mut(n);
            for (p, q) in left.iter_mut().zip(right.iter_mut()) {
                let x = *p;
                let y = *q;
                *p = x * c + y * s;
                *q = y * cc - x * sc;
            }
        };
        let mut eigenvectors = vec![D::zero(); n * n];
        for i in 0..n {
            eigenvectors[i * n + i] = D::one();
        }
        let mut v = vec![D::zero(); n];
        let mut dots = vec![D::zero(); n];
        let mut taus = vec![D::zero(); n];
        let mut betas = vec![D::zero(); n];
        for k in 0..n.saturating_sub(2) {
            let mut tail_sq = D::Real::zero();
            for i in (k + 2)..n {
                let x = a[i * n + k];
                tail_sq = tail_sq + (x.re() * x.re() + x.im() * x.im());
            }
            if tail_sq <= D::Real::zero() {
                continue;
            }
            let x0 = a[(k + 1) * n + k];
            let x0_sq = x0.re() * x0.re() + x0.im() * x0.im();
            let x0_abs = Float::sqrt(x0_sq);
            let x_norm = Float::sqrt(x0_sq + tail_sq);
            let alpha = if x0_abs > D::Real::zero() {
                -(x0 * Self::scalar(x_norm / x0_abs))
            } else {
                -Self::scalar(x_norm)
            };
            let tau = Self::scalar(D::Real::one() / (x_norm * (x_norm + x0_abs)));
            v[k + 1] = x0 - alpha;
            for i in (k + 2)..n {
                v[i] = a[i * n + k];
            }
            taus[k] = tau;
            betas[k] = v[k + 1];
            for d in dots[k + 1..n].iter_mut() {
                *d = D::zero();
            }
            for i in (k + 1)..n {
                let vc = v[i].conj();
                let row = &a[i * n..(i + 1) * n];
                for (d, &x) in dots[k + 1..].iter_mut().zip(&row[k + 1..]) {
                    *d = *d + vc * x;
                }
            }
            for i in (k + 1)..n {
                let tv = tau * v[i];
                let row = &mut a[i * n..(i + 1) * n];
                for (x, &d) in row[k + 1..].iter_mut().zip(&dots[k + 1..]) {
                    *x = *x - tv * d;
                }
            }
            for r in 0..n {
                let row = &mut a[r * n..(r + 1) * n];
                let mut s = D::zero();
                for (&x, &w) in row[k + 1..].iter().zip(&v[k + 1..n]) {
                    s = s + x * w;
                }
                let ts = tau * s;
                for (x, &w) in row[k + 1..].iter_mut().zip(&v[k + 1..n]) {
                    *x = *x - ts * w.conj();
                }
            }
            a[(k + 1) * n + k] = alpha;
            for i in (k + 2)..n {
                a[i * n + k] = v[i];
            }
        }
        for k in (0..n.saturating_sub(2)).rev() {
            if taus[k] == D::zero() {
                continue;
            }
            let tau = taus[k];
            v[k + 1] = betas[k];
            for i in (k + 2)..n {
                v[i] = a[i * n + k];
                a[i * n + k] = D::zero();
            }
            let factor = tau * v[k + 1].conj();
            let col = &mut eigenvectors[(k + 1) * n..(k + 2) * n];
            col[k + 1] = D::one() - factor * v[k + 1];
            for i in (k + 2)..n {
                col[i] = D::zero() - factor * v[i];
            }
            reflect(&mut eigenvectors[(k + 2) * n..], &v[k + 1..n], tau, k + 1);
        }
        let mut cs = vec![D::zero(); n];
        let mut sn = vec![D::zero(); n];
        let mut converged = true;
        let mut iterations = 0;
        let mut active = n;
        'deflate: while active > 1 {
            let mut local_iterations = 0;
            loop {
                let a21 = a[(active - 1) * n + active - 2];
                if ComplexFloat::abs(a21) <= tol {
                    a[(active - 1) * n + active - 2] = D::zero();
                    active -= 1;
                    break;
                }
                let mut low = 0;
                for i in (1..active - 1).rev() {
                    if ComplexFloat::abs(a[i * n + i - 1]) <= tol {
                        a[i * n + i - 1] = D::zero();
                        low = i;
                        break;
                    }
                }
                let mu = {
                    let p = a[(active - 2) * n + active - 2];
                    let b = a[(active - 2) * n + active - 1];
                    let c = a[(active - 1) * n + active - 2];
                    let d = a[(active - 1) * n + active - 1];
                    let delta = (p - d) * Self::scalar(Self::real(0.5));
                    let disc = ComplexFloat::sqrt(delta * delta + b * c);
                    let e1 = delta + disc;
                    let e2 = delta - disc;
                    let shift = if ComplexFloat::abs(e1) <= ComplexFloat::abs(e2) {
                        d + e1
                    } else {
                        d + e2
                    };
                    if ComplexFloat::is_nan(shift) {
                        d
                    } else {
                        shift
                    }
                };
                for i in low..active {
                    a[i * n + i] = a[i * n + i] - mu;
                }
                for i in low..active - 1 {
                    let x = a[i * n + i];
                    let y = a[(i + 1) * n + i];
                    let ny = y.re() * y.re() + y.im() * y.im();
                    if ny > D::Real::zero() {
                        let nx = x.re() * x.re() + x.im() * x.im();
                        let r = Float::sqrt(nx + ny);
                        let inv = Self::scalar(D::Real::one() / r);
                        let c = x * inv;
                        let s = y * inv;
                        let cc = c.conj();
                        let sc = s.conj();
                        cs[i] = c;
                        sn[i] = s;
                        a[i * n + i] = Self::scalar(r);
                        a[(i + 1) * n + i] = D::zero();
                        let (top, bottom) = a[i * n..(i + 2) * n].split_at_mut(n);
                        for j in (i + 1)..active {
                            let p = top[j];
                            let q = bottom[j];
                            top[j] = cc * p + sc * q;
                            bottom[j] = c * q - s * p;
                        }
                    } else {
                        cs[i] = D::one();
                        sn[i] = D::zero();
                    }
                    if i > low {
                        rotate(
                            a.as_mut_slice(),
                            eigenvectors.as_mut_slice(),
                            i - 1,
                            low,
                            cs[i - 1],
                            sn[i - 1],
                        );
                    }
                }
                rotate(
                    a.as_mut_slice(),
                    eigenvectors.as_mut_slice(),
                    active - 2,
                    low,
                    cs[active - 2],
                    sn[active - 2],
                );
                for i in low..active {
                    a[i * n + i] = a[i * n + i] + mu;
                }
                iterations += 1;
                local_iterations += 1;
                if local_iterations > 200 || iterations > max_total_iterations {
                    converged = false;
                    break 'deflate;
                }
            }
        }
        let eigenvalues = Array::from_vec((0..n).map(|i| a[i * n + i]).collect());
        let mut data = vec![D::zero(); n * n];
        for jb in (0..n).step_by(8) {
            for ib in (0..n).step_by(8) {
                for j in jb..(jb + 8).min(n) {
                    for i in ib..(ib + 8).min(n) {
                        data[i * n + j] = eigenvectors[j * n + i];
                    }
                }
            }
        }
        let eigenvectors = Array::from_vec_shape(data, &[n, n]);
        EigenResult {
            values: eigenvalues,
            vectors: eigenvectors,
            converged,
            iterations,
        }
    }
    /// Computes the thin singular value decomposition using one-sided Jacobi rotations
    ///
    /// Panics if the array is not 2D
    pub fn svd(&self) -> SvdResult<D> {
        assert_eq!(self.shape.len(), 2);
        let wide = self.shape[0] < self.shape[1];
        let (m, n) = if wide {
            (self.shape[1], self.shape[0])
        } else {
            (self.shape[0], self.shape[1])
        };
        let (rstride, cstride) = (self.strides[0], self.strides[1]);
        let mut a = vec![D::zero(); m * n];
        if wide {
            for j in 0..n {
                for i in 0..m {
                    a[j * m + i] = self.data[j * rstride + i * cstride].conj();
                }
            }
        } else {
            for i in 0..m {
                for j in 0..n {
                    a[j * m + i] = self.data[i * rstride + j * cstride];
                }
            }
        }
        let mut v = vec![D::zero(); n * n];
        for j in 0..n {
            v[j * n + j] = D::one();
        }
        let norm_sq = |col: &[D]| -> D::Real {
            let mut s0 = D::Real::zero();
            let mut s1 = D::Real::zero();
            let mut s2 = D::Real::zero();
            let mut s3 = D::Real::zero();
            let chunks = col.chunks_exact(4);
            let rem = chunks.remainder();
            for c in chunks {
                s0 = s0 + (c[0].re() * c[0].re() + c[0].im() * c[0].im());
                s1 = s1 + (c[1].re() * c[1].re() + c[1].im() * c[1].im());
                s2 = s2 + (c[2].re() * c[2].re() + c[2].im() * c[2].im());
                s3 = s3 + (c[3].re() * c[3].re() + c[3].im() * c[3].im());
            }
            for &x in rem {
                s0 = s0 + (x.re() * x.re() + x.im() * x.im());
            }
            (s0 + s1) + (s2 + s3)
        };
        let dotc = |x: &[D], y: &[D]| -> D {
            let mut s0 = D::zero();
            let mut s1 = D::zero();
            let mut s2 = D::zero();
            let mut s3 = D::zero();
            let xc = x.chunks_exact(4);
            let yc = y.chunks_exact(4);
            let (xr, yr) = (xc.remainder(), yc.remainder());
            for (xs, ys) in xc.zip(yc) {
                s0 = s0 + xs[0].conj() * ys[0];
                s1 = s1 + xs[1].conj() * ys[1];
                s2 = s2 + xs[2].conj() * ys[2];
                s3 = s3 + xs[3].conj() * ys[3];
            }
            for (&xs, &ys) in xr.iter().zip(yr) {
                s0 = s0 + xs.conj() * ys;
            }
            (s0 + s1) + (s2 + s3)
        };
        let mut sq = (0..n)
            .map(|j| norm_sq(&a[j * m..(j + 1) * m]))
            .collect::<Vec<_>>();
        let scale = Float::sqrt(sq.iter().fold(D::Real::zero(), |acc, &x| acc.max(x)));
        let eps = <D::Real as Float>::epsilon();
        let floor = scale * eps * Self::real(m.max(1) as f64);
        let max_sweeps = 60;
        let mut converged = false;
        let mut iterations = 0;
        while iterations < max_sweeps {
            let mut rotated = false;
            for p in 0..n.saturating_sub(1) {
                for q in (p + 1)..n {
                    let (head, tail) = a.split_at_mut(q * m);
                    let ap = &mut head[p * m..(p + 1) * m];
                    let aq = &mut tail[..m];
                    let gamma = dotc(&*ap, &*aq);
                    let g = ComplexFloat::abs(gamma);
                    let alpha = sq[p];
                    let beta = sq[q];
                    if g <= eps * Float::sqrt(alpha * beta) || g <= floor * floor {
                        continue;
                    }
                    rotated = true;
                    let phase = gamma / Self::scalar(g);
                    let zeta = (beta - alpha) / (Self::real(2.0) * g);
                    let t = zeta.signum()
                        / (ComplexFloat::abs(zeta) + Float::sqrt(D::Real::one() + zeta * zeta));
                    let c = D::Real::one() / Float::sqrt(D::Real::one() + t * t);
                    let s = c * t;
                    let cd = Self::scalar(c);
                    let sd = Self::scalar(s) * phase;
                    let sdc = sd.conj();
                    for (x, y) in ap.iter_mut().zip(aq.iter_mut()) {
                        let (xp, xq) = (*x, *y);
                        *x = cd * xp - sdc * xq;
                        *y = sd * xp + cd * xq;
                    }
                    let (vhead, vtail) = v.split_at_mut(q * n);
                    let vp = &mut vhead[p * n..(p + 1) * n];
                    let vq = &mut vtail[..n];
                    for (x, y) in vp.iter_mut().zip(vq.iter_mut()) {
                        let (xp, xq) = (*x, *y);
                        *x = cd * xp - sdc * xq;
                        *y = sd * xp + cd * xq;
                    }
                    sq[p] = (alpha - t * g).max(D::Real::zero());
                    sq[q] = (beta + t * g).max(D::Real::zero());
                }
            }
            iterations += 1;
            if !rotated {
                converged = true;
                break;
            }
            for j in 0..n {
                sq[j] = norm_sq(&a[j * m..(j + 1) * m]);
            }
        }
        let mut u_raw = a;
        let norms = (0..n)
            .map(|j| Float::sqrt(norm_sq(&u_raw[j * m..(j + 1) * m])))
            .collect::<Vec<_>>();
        for j in 0..n {
            let col = &mut u_raw[j * m..(j + 1) * m];
            if norms[j] > floor {
                let inv = Self::scalar(D::Real::one() / norms[j]);
                for x in col.iter_mut() {
                    *x = *x * inv;
                }
            } else {
                col.fill(D::zero());
            }
        }
        // complete zeroed columns of u to an orthonormal set
        let mut w = vec![D::zero(); m];
        for j in 0..n {
            if norms[j] > floor {
                continue;
            }
            for k in 0..m {
                w.fill(D::zero());
                w[k] = D::one();
                for _ in 0..2 {
                    for l in 0..n {
                        if l == j || (norms[l] <= floor && l > j) {
                            continue;
                        }
                        let col = &u_raw[l * m..(l + 1) * m];
                        let proj = dotc(col, &w);
                        for (x, &c) in w.iter_mut().zip(col) {
                            *x = *x - proj * c;
                        }
                    }
                }
                let w_norm = Float::sqrt(norm_sq(&w));
                if w_norm > Self::real(0.5) / Self::real(m as f64) {
                    let inv = Self::scalar(D::Real::one() / w_norm);
                    for (x, &y) in u_raw[j * m..(j + 1) * m].iter_mut().zip(&w) {
                        *x = y * inv;
                    }
                    break;
                }
            }
        }
        let mut order = (0..n).collect::<Vec<_>>();
        order.sort_by(|&i, &j| norms[j].partial_cmp(&norms[i]).unwrap());
        let mut u_data = vec![D::zero(); m * n];
        let mut v_data = vec![D::zero(); n * n];
        let mut s_data = Vec::with_capacity(n);
        for (new_j, &old_j) in order.iter().enumerate() {
            s_data.push(if norms[old_j] > floor {
                norms[old_j]
            } else {
                D::Real::zero()
            });
            for i in 0..m {
                u_data[i * n + new_j] = u_raw[old_j * m + i];
            }
            for i in 0..n {
                v_data[i * n + new_j] = v[old_j * n + i];
            }
        }
        let s = Array::from_vec(s_data);
        let u = Array::from_vec_shape(u_data, &[m, n]);
        let v = Array::from_vec_shape(v_data, &[n, n]);
        if wide {
            SvdResult {
                u: v,
                s,
                v: u,
                converged,
                iterations,
            }
        } else {
            SvdResult {
                u,
                s,
                v,
                converged,
                iterations,
            }
        }
    }
}

impl<D> Array<D>
where
    D: ComplexFloat + NumCast + PartialOrd,
    D::Real: Float,
{
    /// Returns the median element
    ///
    /// Panics if the array is empty or contains values that cannot be ordered
    pub fn median(&self) -> D {
        let mut sorted = self.data.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let n = sorted.len();
        if n % 2 == 0 {
            let two: D = NumCast::from(2).unwrap();
            (sorted[n / 2 - 1] + sorted[n / 2]) / two
        } else {
            sorted[n / 2]
        }
    }
}
