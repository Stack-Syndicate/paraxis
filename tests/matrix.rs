use paraxis::la::{matrix::Matrix, vector::Vector};

#[test]
fn identity_multiplication() {
    let a = Matrix::from_rows([[1.0, 2.0], [3.0, 4.0]]);
    let identity = Matrix::identity();

    let result = a & identity;
    assert_eq!(result.rows[0].inner[0], 1.0);
    assert_eq!(result.rows[1].inner[1], 4.0);
}

#[test]
fn determinant_and_inverse() {
    let a = Matrix::from_rows([[4.0, 7.0], [2.0, 6.0]]);

    assert!((a.determinant() - 10.0f32).abs() < 1e-6);

    let inv = a.inverse().expect("Matrix should be invertible");
    let identity_check = a & inv;

    assert!((identity_check.rows[0].inner[0] - 1.0).abs() < 1e-6);
    assert!(identity_check.rows[0].inner[1].abs() < 1e-6);
}

#[test]
fn eigenvalues() {
    let a = Matrix::<f32, 2, 2>::from_rows([[2.0, 1.0], [1.0, 2.0]]);
    let mut ak = a;

    for i in 0..10 {
        let (q, r) = ak.qr();
        ak = r & q;
        println!(
            "Iter {}: off-diag: {}, diag: [{}, {}]",
            i, ak.rows[1].inner[0], ak.rows[0].inner[0], ak.rows[1].inner[1]
        );
    }

    let evals = a.eigenvalues(1000, 1e-6);
    let mut results = [evals.inner[0], evals.inner[1]];
    results.sort_by(|x, y| x.partial_cmp(y).unwrap());

    assert!((results[0] - 1.0).abs() < 1e-2);
    assert!((results[1] - 3.0).abs() < 1e-2);
}

#[test]
fn transpose_identity() {
    let a = Matrix::<f32, 3, 3>::from_rows([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]]);
    let at = a.transpose();

    assert_eq!(at.transpose().rows[0].inner[0], a.rows[0].inner[0]);

    assert_eq!(at.rows[0].inner[1], 4.0);
    assert_eq!(at.rows[1].inner[0], 2.0);
}

#[test]
fn multiplication_associativity() {
    let a = Matrix::<f32, 2, 2>::from_rows([[1.0, 2.0], [3.0, 4.0]]);
    let b = Matrix::<f32, 2, 2>::from_rows([[5.0, 6.0], [7.0, 8.0]]);
    let c = Matrix::<f32, 2, 2>::from_rows([[9.0, 1.0], [2.0, 3.0]]);

    let res1 = (a & b) & c;
    let res2 = a & (b & c);

    for r in 0..2 {
        for col in 0..2 {
            assert!((res1.rows[r].inner[col] - res2.rows[r].inner[col]).abs() < 1e-5);
        }
    }
}

#[test]
fn singular_matrix_inverse() {
    let a = Matrix::<f32, 2, 2>::from_rows([[1.0, 2.0], [2.0, 4.0]]);
    assert!(a.inverse().is_none());
    assert!((a.determinant()).abs() < 1e-6);
}

#[test]
fn large_values_stability() {
    let a = Matrix::<f32, 2, 2>::from_rows([[1e10, 0.0], [0.0, 1e-10]]);
    let evals = a.eigenvalues(100, 1e-6);

    let mut res = [evals.inner[0], evals.inner[1]];
    res.sort_by(|a, b| a.partial_cmp(b).unwrap());

    assert!(res[1] > 0.9e10 && res[1] < 1.1e10);
}

#[test]
fn lu_reconstruction() {
    let a = Matrix::<f32, 3, 3>::from_rows([[4.0, 3.0, 2.0], [3.0, 2.0, 1.0], [2.0, 1.0, 3.0]]);
    let (l, u) = a.lu();
    let recon = l & u;

    for r in 0..3 {
        for c in 0..3 {
            assert!((recon.rows[r].inner[c] - a.rows[r].inner[c]).abs() < 1e-5);
        }
    }
}

#[test]
fn qr_orthogonality() {
    let a = Matrix::<f32, 3, 3>::from_rows([
        [12.0, -51.0, 4.0],
        [6.0, 167.0, -68.0],
        [-4.0, 24.0, -41.0],
    ]);
    let (q, _) = a.qr();
    let qt = q.transpose();
    let identity_check = q & qt;

    for r in 0..3 {
        for c in 0..3 {
            let expected = if r == c { 1.0 } else { 0.0 };
            assert!((identity_check.rows[r].inner[c] - expected).abs() < 1e-5);
        }
    }
}
#[test]
fn linear_solver() {
    let a = Matrix::<f32, 3, 3>::from_rows([[2.0, 1.0, -1.0], [-3.0, -1.0, 2.0], [-2.0, 1.0, 2.0]]);
    let b = Vector::<f32, 3>::from_slice(&[8.0, -11.0, -3.0]);
    if let Some(x) = a.solve(b) {
        let b_prime = a * x;
        for i in 0..3 {
            assert!((b_prime.inner[i] - b.inner[i]).abs() < 1e-4);
        }
    } else {
        panic!("Non-singular matrix failed to solve!");
    }
}
