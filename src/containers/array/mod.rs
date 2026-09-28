pub mod index;
pub mod ops;
use num_complex::ComplexFloat;
use num_traits::{Float, Num, NumCast, One, Zero};
use std::fmt::Debug;

#[derive(Debug, Clone)]
pub struct EigenResult<D> {
    pub values: Array<D>,
    pub vectors: Array<D>,
    pub converged: bool,
    pub iterations: usize,
}
#[derive(Debug, Clone)]
pub struct QrResult<D> {
    pub q: Array<D>,
    pub r: Array<D>,
    pub permutation: Vec<usize>,
    pub reflection_count: usize,
}
#[derive(Debug, Clone)]
pub struct LuResult<D> {
    pub l: Array<D>,
    pub u: Array<D>,
    pub pivots: Vec<usize>,
    pub swap_count: usize,
}
pub struct SvdResult<D: ComplexFloat> {
    pub u: Array<D>,
    pub s: Array<D::Real>,
    pub v: Array<D>,
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
    pub fn submatrix(
        &self,
        row_start: usize,
        row_end: usize,
        col_start: usize,
        col_end: usize,
    ) -> Array<D> {
        assert_eq!(self.shape.len(), 2);
        assert!(row_start <= row_end && row_end <= self.shape[0]);
        assert!(col_start <= col_end && col_end <= self.shape[1]);
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
    pub fn set_submatrix(&mut self, row_start: usize, col_start: usize, value: &Array<D>) {
        assert_eq!(self.shape.len(), 2);
        assert_eq!(value.shape.len(), 2);
        let (rows, cols) = (value.shape[0], value.shape[1]);
        assert!(row_start + rows <= self.shape[0]);
        assert!(col_start + cols <= self.shape[1]);
        for i in 0..rows {
            for j in 0..cols {
                self[&[row_start + i, col_start + j]] = value[&[i, j]];
            }
        }
    }
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
    pub fn swap_elements(&mut self, a: &[usize], b: &[usize]) {
        assert_eq!(a.len(), self.shape.len());
        assert_eq!(b.len(), self.shape.len());
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
    pub fn swap_rows(&mut self, a: usize, b: usize) {
        assert!(self.shape.len() >= 2);
        assert!(a < self.shape[0]);
        assert!(b < self.shape[0]);
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
    pub fn trace(&self) -> D {
        assert_eq!(self.shape.len(), 2);
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
        assert_eq!(k, other.shape[0]);
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
        assert!(axis < self.shape.len());
        assert!(other_axis < other.shape.len());
        assert_eq!(self.shape[axis], other.shape[other_axis]);
        let self_dims = self.shape.len();
        let other_dims = other.shape.len();
        let lhs = self.permute_axes(&Self::move_axis_to_end(self_dims, axis));
        let rhs = other.permute_axes(&Self::move_axis_to_start(other_dims, other_axis));
        lhs.contract(&rhs)
    }
    pub fn cross(&self, other: &Array<D>) -> Array<D> {
        assert!(self.shape.len() == 1);
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
        assert!(!self.data.is_empty());
        self.data.iter().fold(D::zero(), |acc, x| acc + *x)
    }
    pub fn product(&self) -> D {
        assert!(!self.data.is_empty());
        self.data.iter().fold(D::one(), |acc, x| acc * *x)
    }
}
impl<D: PartialOrd + Copy> Array<D> {
    pub fn min(&self) -> Option<D> {
        self.data.iter().copied().fold(None, |acc, x| match acc {
            None => Some(x),
            Some(m) if x < m => Some(x),
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

    pub fn dot(&self, other: &Array<D>) -> D {
        self.data
            .iter()
            .zip(other.data.iter())
            .fold(D::zero(), |acc, (&a, &b)| acc + a * b.conj())
    }
    pub fn norm(&self) -> D::Real {
        Float::sqrt(self.dot(self).re())
    }
    pub fn dist(&self, other: &Array<D>) -> D::Real {
        (other - self).norm()
    }
    pub fn normalize(self) -> Array<D> {
        let norm = self.norm();
        assert!(norm > D::Real::zero());
        self / Self::scalar(norm)
    }
    pub fn mean(&self) -> D {
        let n: D = NumCast::from(self.data.len()).unwrap();
        self.data.iter().fold(D::zero(), |acc, &x| acc + x) / n
    }
    pub fn variance(&self) -> D::Real {
        let m = self.mean();
        let n = Self::real(self.data.len() as f64);
        self.data.iter().fold(D::Real::zero(), |acc, &x| {
            let diff = x - m;
            acc + (diff * diff.conj()).re()
        }) / n
    }
    pub fn stddev(&self) -> D::Real {
        Float::sqrt(self.variance())
    }
    pub fn conj_transpose(self) -> Array<D> {
        let t = self.transpose();
        let mut data = Vec::with_capacity(t.data.len());
        for offset in 0..t.data.len() {
            let indices = Self::indices_from_offset(offset, &t.shape);
            data.push(t.data[t.offset(&indices)].conj());
        }
        let shape = t.shape.clone();
        Array::from_vec_shape(data, &shape)
    }
    pub fn qr(&self) -> QrResult<D> {
        self.qr_impl(true)
    }
    fn qr_unpivoted(&self) -> QrResult<D> {
        self.qr_impl(false)
    }
    fn qr_impl(&self, pivoting: bool) -> QrResult<D> {
        assert_eq!(self.shape.len(), 2);
        let m = self.shape[0];
        let n = self.shape[1];
        let steps = m.min(n);
        let mut r_data = Vec::with_capacity(m * n);
        for i in 0..m {
            for j in 0..n {
                r_data.push(self[&[i, j]]);
            }
        }
        let mut r = Array::from_vec_shape(r_data, &[m, n]);
        let mut q = Array::<D>::identity(m);
        let mut permutation = (0..n).collect::<Vec<_>>();
        let mut v = vec![D::zero(); m];
        let mut vn1 = vec![D::Real::zero(); n];
        let mut vn2 = vec![D::Real::zero(); n];
        if pivoting {
            for j in 0..n {
                let mut norm_sq = D::Real::zero();
                for i in 0..m {
                    let x = r.data[i * n + j];
                    norm_sq = norm_sq + (x * x.conj()).re();
                }
                let norm = Float::sqrt(norm_sq);
                vn1[j] = norm;
                vn2[j] = norm;
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
                    for i in 0..m {
                        r.data.swap(i * n + k, i * n + pivot);
                    }
                    vn1.swap(k, pivot);
                    vn2.swap(k, pivot);
                    permutation.swap(k, pivot);
                }
            }
            let mut x_norm_sq = D::Real::zero();
            for i in k..m {
                let x = r.data[i * n + k];
                x_norm_sq = x_norm_sq + (x * x.conj()).re();
            }
            let x_norm = Float::sqrt(x_norm_sq);
            if x_norm <= D::Real::zero() {
                continue;
            }
            let x0 = r.data[k * n + k];
            let x0_abs = ComplexFloat::abs(x0);
            let phase = if x0_abs > D::Real::zero() {
                x0 / Self::scalar(x0_abs)
            } else {
                D::one()
            };
            let alpha = -phase * Self::scalar(x_norm);
            for i in k..m {
                v[i] = r.data[i * n + k];
            }
            v[k] = v[k] - alpha;
            let mut v_norm_sq = D::Real::zero();
            for i in k..m {
                let vi = v[i];
                v_norm_sq = v_norm_sq + (vi * vi.conj()).re();
            }
            if v_norm_sq <= D::Real::zero() {
                continue;
            }
            let tau = Self::scalar(Self::real(2.0) / v_norm_sq);
            reflection_count += 1;
            for j in k..n {
                let mut dot = D::zero();
                for i in k..m {
                    dot = dot + v[i].conj() * r.data[i * n + j];
                }
                let factor = tau * dot;
                for i in k..m {
                    r.data[i * n + j] = r.data[i * n + j] - factor * v[i];
                }
            }
            for i in 0..m {
                let mut dot = D::zero();
                for j in k..m {
                    dot = dot + q.data[i * m + j] * v[j];
                }
                let factor = tau * dot;
                for j in k..m {
                    q.data[i * m + j] = q.data[i * m + j] - factor * v[j].conj();
                }
            }
            if pivoting {
                let threshold = Self::real(0.05);
                for j in (k + 1)..n {
                    let old_norm = vn1[j];
                    if old_norm <= D::Real::zero() {
                        continue;
                    }
                    let top = ComplexFloat::abs(r.data[k * n + j]);
                    let ratio = top / old_norm;
                    let temp = (D::Real::one() - ratio * ratio).max(D::Real::zero());
                    let reference_ratio = if vn2[j] > D::Real::zero() {
                        old_norm / vn2[j]
                    } else {
                        D::Real::zero()
                    };
                    let accuracy = temp * reference_ratio * reference_ratio;
                    if accuracy <= threshold {
                        let mut norm_sq = D::Real::zero();
                        for i in (k + 1)..m {
                            let x = r.data[i * n + j];
                            norm_sq = norm_sq + (x * x.conj()).re();
                        }
                        let exact_norm = Float::sqrt(norm_sq);
                        vn1[j] = exact_norm;
                        vn2[j] = exact_norm;
                    } else {
                        vn1[j] = old_norm * Float::sqrt(temp);
                    }
                }
                vn1[k] = D::Real::zero();
            }
        }
        QrResult {
            q,
            r,
            permutation,
            reflection_count,
        }
    }
    pub fn lu(&self) -> LuResult<D> {
        assert_eq!(self.shape.len(), 2);
        assert_eq!(self.shape[0], self.shape[1]);
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
        assert_eq!(b.shape[0], self.shape[0]);
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
    pub fn solve(&self, b: &Array<D>) -> Array<D> {
        let lu = self.lu();
        self.solve_with_lu(b, &lu)
    }
    pub fn rank(&self) -> usize {
        assert_eq!(self.shape.len(), 2);
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
    pub fn eigen(&self) -> EigenResult<D> {
        assert_eq!(self.shape.len(), 2);
        assert_eq!(self.shape[0], self.shape[1]);
        let n = self.shape[0];
        let mut a = self.clone();
        let mut eigenvectors = Array::<D>::identity(n);
        let mut converged = true;
        let mut iterations = 0;
        let scale = a
            .to_cloned_vec()
            .iter()
            .fold(D::Real::zero(), |acc, &x| acc.max(ComplexFloat::abs(x)));
        let tol = scale * Self::real(1e-12);
        let max_total_iterations = 200 * n.max(1);
        let mut active = n;
        'deflate: while active > 1 {
            let mut local_iterations = 0;
            loop {
                let a21 = a[&[active - 1, active - 2]];
                if ComplexFloat::abs(a21) <= tol {
                    active -= 1;
                    break;
                }
                let mu = a[&[active - 1, active - 1]];
                let sub = a.submatrix(0, active, 0, active);
                let identity_active = Array::<D>::identity(active);
                let shifted = &sub - &(&identity_active * mu);
                let qr = shifted.qr_unpivoted();
                let new_sub = &qr.r.contract(&qr.q) + &(&identity_active * mu);
                a.set_submatrix(0, 0, &new_sub);
                let mut full_q = Array::<D>::identity(n);
                full_q.set_submatrix(0, 0, &qr.q);
                eigenvectors = eigenvectors.contract(&full_q);
                iterations += 1;
                local_iterations += 1;
                if local_iterations > 200 || iterations > max_total_iterations {
                    converged = false;
                    break 'deflate;
                }
            }
        }
        let eigenvalues = Array::from_vec((0..n).map(|i| a[&[i, i]]).collect());
        EigenResult {
            values: eigenvalues,
            vectors: eigenvectors,
            converged,
            iterations,
        }
    }
    pub fn svd(&self) -> SvdResult<D> {
        assert_eq!(self.shape.len(), 2);
        let m = self.shape[0];
        let n = self.shape[1];
        if m < n {
            let t = self.clone().conj_transpose().svd();
            return SvdResult {
                u: t.v,
                s: t.s,
                v: t.u,
                converged: t.converged,
                iterations: t.iterations,
            };
        }
        let mut a = self.clone(); // copy of self to be orthogonalized
        let mut v = Array::<D>::identity(n); // accumulates rotations
        let scale = self
            .column_norms()
            .into_iter()
            .fold(D::Real::zero(), |acc, x| acc.max(x));
        let eps = <D::Real as Float>::epsilon();
        let floor = scale * eps * Self::real(m.max(1) as f64);
        let max_sweeps = 60;
        let mut converged = false;
        let mut iterations = 0;
        while iterations < max_sweeps {
            let mut rotated = false;
            for p in 0..n.saturating_sub(1) {
                for q in (p + 1)..n {
                    let mut alpha = D::Real::zero();
                    let mut beta = D::Real::zero();
                    let mut gamma = D::zero();
                    for i in 0..m {
                        let ap = a[&[i, p]];
                        let aq = a[&[i, q]];
                        alpha = alpha + (ap * ap.conj()).re(); // squared length of column p
                        beta = beta + (aq * aq.conj()).re(); // squared length of column q
                        gamma = gamma + ap.conj() * aq; // inner product
                    }
                    let g = ComplexFloat::abs(gamma);
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
                    for i in 0..m {
                        let ap = a[&[i, p]];
                        let aq = a[&[i, q]];
                        a[&[i, p]] = cd * ap - sd.conj() * aq;
                        a[&[i, q]] = sd * ap + cd * aq;
                    }
                    for i in 0..n {
                        let vp = v[&[i, p]];
                        let vq = v[&[i, q]];
                        v[&[i, p]] = cd * vp - sd.conj() * vq;
                        v[&[i, q]] = sd * vp + cd * vq;
                    }
                }
            }
            iterations += 1;
            if !rotated {
                converged = true;
                break;
            }
        }
        let (mut u_raw, norms) = a.normalize_columns(floor);
        // complete zeroed columns of u to an orthonormal set
        for j in 0..n {
            if norms[j] > floor {
                continue;
            }
            for k in 0..m {
                let mut w = vec![D::zero(); m];
                w[k] = D::one();
                for _ in 0..2 {
                    for l in 0..n {
                        if l == j || (norms[l] <= floor && l > j) {
                            continue;
                        }
                        let mut proj = D::zero();
                        for i in 0..m {
                            proj = proj + u_raw[&[i, l]].conj() * w[i];
                        }
                        for i in 0..m {
                            w[i] = w[i] - proj * u_raw[&[i, l]];
                        }
                    }
                }
                let mut w_norm_sq = D::Real::zero();
                for i in 0..m {
                    w_norm_sq = w_norm_sq + (w[i] * w[i].conj()).re();
                }
                let w_norm = Float::sqrt(w_norm_sq);
                if w_norm > Self::real(0.5) / Self::real(m as f64) {
                    for i in 0..m {
                        u_raw[&[i, j]] = w[i] / Self::scalar(w_norm);
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
                u_data[i * n + new_j] = u_raw[&[i, old_j]];
            }
            for i in 0..n {
                v_data[i * n + new_j] = v[&[i, old_j]];
            }
        }
        SvdResult {
            u: Array::from_vec_shape(u_data, &[m, n]),
            s: Array::from_vec(s_data),
            v: Array::from_vec_shape(v_data, &[n, n]),
            converged,
            iterations,
        }
    }
}

impl<D> Array<D>
where
    D: ComplexFloat + NumCast + PartialOrd,
    D::Real: Float,
{
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
