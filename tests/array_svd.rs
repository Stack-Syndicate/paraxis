use num_complex::Complex64;
use paraxis::containers::array::*;

#[test]
fn svd_reconstructs_matrix() {
    let m = Array::from_slice_shape(&[4.0, -2.0, 1.0, 3.0, 1.0, 7.0, -5.0, 2.0, 6.0], &[3, 3]);
    let r = m.svd();
    assert!(r.converged);
    let s = r.s.to_cloned_vec();
    let mut sigma = vec![0.0; 9];
    for i in 0..3 {
        sigma[i * 3 + i] = s[i];
    }
    let sigma = Array::from_vec_shape(sigma, &[3, 3]);
    let rebuilt = r.u.contract(&sigma).contract(&r.v.conj_transpose());
    for i in 0..3 {
        for j in 0..3 {
            assert!((rebuilt[&[i, j]] as f64 - m[&[i, j]]).abs() < 1e-10);
        }
    }
}

#[test]
fn svd_factors_have_correct_properties() {
    let m = Array::from_slice_shape(&[1.0, 2.0, 3.0, 2.0, 4.0, 6.0, 1.0, 0.0, 1.0], &[3, 3]);
    let r = m.svd();
    assert!(r.converged);
    let s = r.s.to_cloned_vec();
    assert_eq!(s.len(), 3);
    // singular values are nonnegative
    for k in 0..3 {
        assert!(s[k] >= 0.0);
    }
    // singular values are sorted descending
    for k in 0..2 {
        assert!(s[k] >= s[k + 1]);
    }
    // rank 2, so two clearly positive values and one exact zero
    assert!(s[0] > 1e-3);
    assert!(s[1] > 1e-3);
    assert_eq!(s[2], 0.0);
    // sum of squared singular values equals squared frobenius norm
    let fro_sq: f64 = m.to_cloned_vec().iter().map(|x| x * x).sum();
    let s_sq: f64 = s.iter().map(|x| x * x).sum();
    assert!((fro_sq - s_sq).abs() < 1e-10);
    // u and v have orthonormal columns
    for a in 0..3 {
        for b in 0..3 {
            let mut utu = 0.0;
            let mut vtv = 0.0;
            for i in 0..3 {
                utu += r.u[&[i, a]] * r.u[&[i, b]];
                vtv += r.v[&[i, a]] * r.v[&[i, b]];
            }
            let expected = if a == b { 1.0 } else { 0.0 };
            assert!((utu - expected).abs() < 1e-10);
            assert!((vtv - expected).abs() < 1e-10);
        }
    }
    // u and v have orthonormal rows, so they are fully unitary
    for a in 0..3 {
        for b in 0..3 {
            let mut uut = 0.0;
            let mut vvt = 0.0;
            for k in 0..3 {
                uut += r.u[&[a, k]] * r.u[&[b, k]];
                vvt += r.v[&[a, k]] * r.v[&[b, k]];
            }
            let expected = if a == b { 1.0 } else { 0.0 };
            assert!((uut - expected).abs() < 1e-10);
            assert!((vvt - expected).abs() < 1e-10);
        }
    }
    // u * s * v^T reproduces m
    for i in 0..3 {
        for j in 0..3 {
            let mut sum = 0.0;
            for k in 0..3 {
                sum += r.u[&[i, k]] * s[k] * r.v[&[j, k]];
            }
            assert!((sum - m[&[i, j]]).abs() < 1e-10);
        }
    }
    // m maps each column of v to a vector of length s
    for k in 0..3 {
        let mut mv_norm_sq = 0.0;
        for i in 0..3 {
            let mut mv = 0.0;
            for j in 0..3 {
                mv += m[&[i, j]] * r.v[&[j, k]];
            }
            mv_norm_sq += mv * mv;
        }
        assert!((mv_norm_sq.sqrt() - s[k]).abs() < 1e-10);
    }
}

#[test]
fn svd_complex_input_has_correct_properties() {
    let m = Array::from_slice_shape(
        &[
            Complex64::new(1.0, 2.0),
            Complex64::new(2.0, 4.0),
            Complex64::new(3.0, 6.0),
            Complex64::new(0.5, -1.0),
            Complex64::new(1.0, -2.0),
            Complex64::new(1.5, -3.0),
            Complex64::new(1.0, 0.0),
            Complex64::new(0.0, 1.0),
            Complex64::new(-1.0, 2.5),
        ],
        &[3, 3],
    );
    let r = m.svd();
    assert!(r.converged);
    let s = r.s.to_cloned_vec();
    // singular values are real, nonnegative and sorted descending
    for k in 0..3 {
        assert!(s[k] >= 0.0);
    }
    for k in 0..2 {
        assert!(s[k] >= s[k + 1]);
    }
    // rank 2 by construction (row 2 is a multiple of row 1), so one exact zero
    assert!(s[1] > 1e-3);
    assert_eq!(s[2], 0.0);
    // u and v have orthonormal columns and rows under the hermitian inner product
    for a in 0..3 {
        for b in 0..3 {
            let mut uhu = Complex64::new(0.0, 0.0);
            let mut vhv = Complex64::new(0.0, 0.0);
            let mut uuh = Complex64::new(0.0, 0.0);
            let mut vvh = Complex64::new(0.0, 0.0);
            for i in 0..3 {
                uhu += r.u[&[i, a]].conj() * r.u[&[i, b]];
                vhv += r.v[&[i, a]].conj() * r.v[&[i, b]];
                uuh += r.u[&[a, i]] * r.u[&[b, i]].conj();
                vvh += r.v[&[a, i]] * r.v[&[b, i]].conj();
            }
            let expected = Complex64::new(if a == b { 1.0 } else { 0.0 }, 0.0);
            assert!((uhu - expected).norm() < 1e-10);
            assert!((vhv - expected).norm() < 1e-10);
            assert!((uuh - expected).norm() < 1e-10);
            assert!((vvh - expected).norm() < 1e-10);
        }
    }
    // u * s * v^H reproduces m
    for i in 0..3 {
        for j in 0..3 {
            let mut sum = Complex64::new(0.0, 0.0);
            for k in 0..3 {
                sum += r.u[&[i, k]] * s[k] * r.v[&[j, k]].conj();
            }
            assert!((sum - m[&[i, j]]).norm() < 1e-10);
        }
    }
}

#[test]
fn svd_wide_input_has_correct_properties() {
    let m = Array::from_slice_shape(
        &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 10.0, -1.0, 0.5, 2.0],
        &[3, 4],
    );
    let r = m.svd();
    assert!(r.converged);
    let s = r.s.to_cloned_vec();
    // shapes are thin: u is 3x3, s has 3 values, v is 4x3
    assert_eq!(r.u.shape(), &[3, 3]);
    assert_eq!(s.len(), 3);
    assert_eq!(r.v.shape(), &[4, 3]);
    // singular values are nonnegative and sorted descending
    for k in 0..3 {
        assert!(s[k] >= 0.0);
    }
    for k in 0..2 {
        assert!(s[k] >= s[k + 1]);
    }
    // u is unitary and v has orthonormal columns
    for a in 0..3 {
        for b in 0..3 {
            let mut utu = 0.0;
            let mut uut = 0.0;
            let mut vtv = 0.0;
            for i in 0..3 {
                utu += r.u[&[i, a]] * r.u[&[i, b]];
                uut += r.u[&[a, i]] * r.u[&[b, i]];
            }
            for i in 0..4 {
                vtv += r.v[&[i, a]] * r.v[&[i, b]];
            }
            let expected = if a == b { 1.0 } else { 0.0 };
            assert!((utu - expected as f64).abs() < 1e-10);
            assert!((uut - expected).abs() < 1e-10);
            assert!((vtv - expected).abs() < 1e-10);
        }
    }
    // u * s * v^T reproduces m
    for i in 0..3 {
        for j in 0..4 {
            let mut sum = 0.0;
            for k in 0..3 {
                sum += r.u[&[i, k]] * s[k] * r.v[&[j, k]];
            }
            assert!((sum - m[&[i, j]]).abs() < 1e-10);
        }
    }
}

#[test]
fn svd_singular_values_scale_with_input() {
    let base = [4.0, -2.0, 1.0, 3.0, 1.0, 7.0, -5.0, 2.0, 6.0];
    let s_base = Array::from_slice_shape(&base, &[3, 3])
        .svd()
        .s
        .to_cloned_vec();
    for factor in [1e-12, 1e12] {
        let scaled = base.iter().map(|x| x * factor).collect::<Vec<_>>();
        let r = Array::from_slice_shape(&scaled, &[3, 3]).svd();
        assert!(r.converged);
        let s = r.s.to_cloned_vec();
        // singular values scale by the same factor as the input
        for k in 0..3 {
            assert!((s[k] / factor - s_base[k] as f64).abs() < 1e-8 * s_base[0]);
        }
        // u and v are unaffected by scale and stay orthonormal
        for a in 0..3 {
            for b in 0..3 {
                let mut utu = 0.0;
                let mut vtv = 0.0;
                for i in 0..3 {
                    utu += r.u[&[i, a]] * r.u[&[i, b]];
                    vtv += r.v[&[i, a]] * r.v[&[i, b]];
                }
                let expected = if a == b { 1.0 } else { 0.0 };
                assert!((utu - expected).abs() < 1e-10);
                assert!((vtv - expected).abs() < 1e-10);
            }
        }
    }
}

#[test]
fn svd_zero_matrix() {
    let m = Array::from_slice_shape(&[0.0; 9], &[3, 3]);
    let r = m.svd();
    assert!(r.converged);
    let s = r.s.to_cloned_vec();
    // every singular value is exactly zero
    for k in 0..3 {
        assert_eq!(s[k], 0.0);
    }
    // u is still a complete orthonormal basis and v is unitary
    for a in 0..3 {
        for b in 0..3 {
            let mut utu = 0.0;
            let mut vtv = 0.0;
            for i in 0..3 {
                utu += r.u[&[i, a]] * r.u[&[i, b]];
                vtv += r.v[&[i, a]] * r.v[&[i, b]];
            }
            let expected = if a == b { 1.0 } else { 0.0 };
            assert!((utu - expected as f64).abs() < 1e-10);
            assert!((vtv - expected).abs() < 1e-10);
        }
    }
}

#[test]
fn svd_vector_shapes() {
    // a column vector has one singular value equal to its norm
    let col = Array::from_slice_shape(&[3.0, 4.0, 12.0], &[3, 1]);
    let r = col.svd();
    assert!(r.converged);
    assert_eq!(r.u.shape(), &[3, 1]);
    assert_eq!(r.v.shape(), &[1, 1]);
    let s = r.s.to_cloned_vec();
    assert_eq!(s.len(), 1);
    assert!((s[0] - 13.0 as f64).abs() < 1e-12);
    // u * s * v^T reproduces the column
    for i in 0..3 {
        assert!((r.u[&[i, 0]] * s[0] * r.v[&[0, 0]] - col[&[i, 0]]).abs() < 1e-12);
    }
    // a row vector goes through the wide path and also has one singular value equal to its norm
    let row = Array::from_slice_shape(&[3.0, 4.0, 12.0], &[1, 3]);
    let r = row.svd();
    assert!(r.converged);
    assert_eq!(r.u.shape(), &[1, 1]);
    assert_eq!(r.v.shape(), &[3, 1]);
    let s = r.s.to_cloned_vec();
    assert_eq!(s.len(), 1);
    assert!((s[0] - 13.0 as f64).abs() < 1e-12);
    // u * s * v^T reproduces the row
    for j in 0..3 {
        assert!((r.u[&[0, 0]] * s[0] * r.v[&[j, 0]] - row[&[0, j]]).abs() < 1e-12);
    }
}
