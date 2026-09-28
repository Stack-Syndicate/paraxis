use paraxis::containers::array::Array;

// TODO: Test against other crates (nalgebra maybe)

#[test]
fn min_max() {
    let v = Array::from_vec(vec![3.0, 1.0, 4.0, 1.0, 5.0, 9.0, 2.0, 6.0]);
    assert_eq!(v.min(), Some(1.0));
    assert_eq!(v.max(), Some(9.0));
}
#[test]
fn dot_product() {
    let v1 = Array::from_vec(vec![1.0, 1.0, 1.0]);
    let v2 = Array::from_vec(vec![1.0, 1.0, 1.0]);
    let r1 = v1.dot(&v2);
    assert_eq!(r1, 3.0);
}
#[test]
fn negation() {
    let v = Array::from_vec(vec![1, 1, 1]);
    let r = -v;
    assert_eq!(r.to_vec(), vec![-1, -1, -1]);
}
#[test]
fn qr_decomposition() {
    let a = Array::from_vec_shape(
        vec![12.0, -51.0, 4.0, 6.0, 167.0, -68.0, -4.0, 24.0, -41.0],
        &[3, 3],
    );
    let qr = a.qr();
    let q = qr.q;
    let r = qr.r;
    let permutation = qr.permutation;
    // Q should be orthogonal: Q^T * Q = I
    let qt = q.clone().transpose();
    let qtq = qt.contract(&q);
    let identity = Array::<f64>::identity(3);

    for i in 0..3 {
        for j in 0..3 {
            let diff = (qtq[&[i, j]] - identity[&[i, j]]).abs();
            assert!(diff < 1e-9);
        }
    }
    // R should be upper triangular
    for i in 0..3 {
        for j in 0..i {
            let val = r[&[i, j]];
            assert!(val.abs() < 1e-9);
        }
    }
    // Q * R should reconstruct the columns of A in pivoted order
    let qr_product = q.contract(&r);
    for i in 0..3 {
        for j in 0..3 {
            let original_column = permutation[j];
            let diff = (qr_product[&[i, j]] - a[&[i, original_column]]).abs();
            assert!(diff < 1e-9);
        }
    }
}
#[test]
fn eigenvalues_symmetric() {
    let a = Array::from_vec_shape(vec![4.0, 1.0, 1.0, 1.0, 4.0, 1.0, 1.0, 1.0, 4.0], &[3, 3]);
    let eigenresult = a.eigen();
    let mut computed: Vec<f64> = eigenresult.values.to_vec().clone();
    computed.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let mut expected = vec![3.0, 3.0, 6.0];
    expected.sort_by(|a, b| a.partial_cmp(b).unwrap());
    for (c, e) in computed.iter().zip(expected.iter()) {
        assert!((c - e).abs() < 1e-9);
    }
}
#[test]
fn eigenvectors() {
    let a = Array::from_vec_shape(vec![4.0, 1.0, 1.0, 1.0, 4.0, 1.0, 1.0, 1.0, 4.0], &[3, 3]);
    let result = a.eigen();
    for i in 0..3 {
        let v = result.vectors.column(i);
        let lambda = result.values[i];
        let av = a.contract(&v);
        for j in 0..3 {
            let diff = (av[j] as f64 - lambda * v[j] as f64).abs();
            assert!(diff < 1e-6);
        }
    }
}
#[test]
fn solve_linear_system() {
    let a = Array::from_vec_shape(
        vec![12.0, -51.0, 4.0, 6.0, 167.0, -68.0, -4.0, 24.0, -41.0],
        &[3, 3],
    );
    let x_expected = Array::from_vec(vec![1.0, 2.0, 3.0]);
    let b_matrix = a.contract(&x_expected);
    let b = Array::from_vec(b_matrix.to_cloned_vec());
    let x_computed = a.solve(&b);
    for i in 0..3 {
        let diff = ((x_computed[i] - x_expected[i]) as f64).abs();
        assert!(diff < 1e-6,);
    }
}
#[test]
fn determinant_known_value() {
    let a = Array::from_vec_shape(vec![2.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 4.0], &[3, 3]);
    let det = a.det();
    assert!((det - 24.0f64).abs() < 1e-9);
}

#[test]
fn determinant_sign() {
    let a = Array::from_vec_shape(vec![0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0], &[3, 3]);
    let det = a.det();
    assert!((det - (-1.0f64)).abs() < 1e-9);
}
#[test]
fn determinant_negative_definite() {
    let a = Array::from_vec_shape(vec![1.0, 2.0, 3.0, 4.0], &[2, 2]);
    let det = a.det();
    assert!((det - (-2.0f64)).abs() < 1e-9);
}
#[test]
fn rank_full_rank_matrix() {
    let a = Array::from_vec_shape(
        vec![12.0, -51.0, 4.0, 6.0, 167.0, -68.0, -4.0, 24.0, -41.0],
        &[3, 3],
    );
    assert_eq!(a.rank(), 3);
}

#[test]
fn rank_deficient_matrix() {
    let a = Array::from_vec_shape(vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 2.0, 4.0, 6.0], &[3, 3]);
    assert_eq!(a.rank(), 2);
}
#[test]
fn rank_zero_matrix() {
    let a = Array::from_vec_shape(vec![0.0; 9], &[3, 3]);
    assert_eq!(a.rank(), 0);
}
#[test]
fn rank_identity() {
    let a = Array::<f64>::identity(3);
    assert_eq!(a.rank(), 3);
}
#[test]
fn inverse_reconstructs_identity() {
    let a = Array::from_vec_shape(
        vec![12.0, -51.0, 4.0, 6.0, 167.0, -68.0, -4.0, 24.0, -41.0],
        &[3, 3],
    );
    let inv = a.inverse();
    let product = a.contract(&inv);
    let identity = Array::<f64>::identity(3);
    for i in 0..3 {
        for j in 0..3 {
            let diff = (product[&[i, j]] - identity[&[i, j]]).abs();
            assert!(diff < 1e-6);
        }
    }
    let product_rev = inv.contract(&a);
    for i in 0..3 {
        for j in 0..3 {
            let diff = (product_rev[&[i, j]] - identity[&[i, j]]).abs();
            assert!(diff < 1e-6);
        }
    }
}
#[test]
fn determinant_three_cycle() {
    let a = Array::from_vec_shape(vec![0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0], &[3, 3]);
    assert!((a.det() - 1.0f64).abs() < 1e-9);
}
#[test]
fn rank_rectangular_full_row_rank() {
    let a = Array::from_vec_shape(vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0], &[2, 3]);
    assert_eq!(a.rank(), 2);
}
#[test]
fn rank_rectangular_full_column_rank() {
    let a = Array::from_vec_shape(vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0], &[3, 2]);
    assert_eq!(a.rank(), 2);
}
