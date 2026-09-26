use paraxis::containers::array::Array;

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
    assert_eq!(r.to_vec(), vec![-1, -1, -1]);
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
    let mut computed: Vec<f64> = eigenvalues.to_vec().clone();
    computed.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let mut expected = vec![3.0, 3.0, 6.0];
    expected.sort_by(|a, b| a.partial_cmp(b).unwrap());
    for (c, e) in computed.iter().zip(expected.iter()) {
        assert!((c - e).abs() < 1e-9);
    }
}
