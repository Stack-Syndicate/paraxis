use paraxis::mathematics::{matrix::Matrix, vector::Vector};

#[test]
fn vector_add_prod_dot_cross() {
    let v1 = Vector::new([1.0, 2.0, 3.0]);
    let v2 = Vector::new([1.0; 3]);
    assert_eq!(v1 + v2, Vector::new([2.0, 3.0, 4.0]));
    assert_eq!(v1.dot(&v2), (v1 * v2).sum());
    assert_eq!(v1.cross(&v2), Vector::new([-1.0, 2.0, -1.0]));
}

#[test]
fn matrix_operations_various_shapes() {
    let row1 = Matrix::<3, 1>::new([[1.0, 2.0, 3.0]]);
    let row2 = Matrix::<3, 1>::new([[1.0, 1.0, 1.0]]);

    assert_eq!(row1 + row2, Matrix::<3, 1>::new([[2.0, 3.0, 4.0]]));
    assert_eq!((row1 * row2).sum(), row1.dot(&row2));

    let col1 = Matrix::<1, 3>::new([[1.0], [2.0], [3.0]]);
    let col2 = Matrix::<1, 3>::new([[1.0], [1.0], [1.0]]);

    assert_eq!(col1 + col2, Matrix::<1, 3>::new([[2.0], [3.0], [4.0]]));
    assert_eq!(col1.dot(&col2), (col1 * col2).sum());

    let a = Matrix::<3, 2>::new([[1., 2., 3.], [4., 5., 6.]]);
    let b = Matrix::<2, 3>::new([[7., 8.], [9., 10.], [11., 12.]]);

    let c = a | b;
    let expected = Matrix::<2, 2>::new([[58., 64.], [139., 154.]]);
    assert_eq!(c, expected);

    let sum = a + a;
    let prod = a * a;
    let total_sum = sum.sum();
    let total_prod = prod.prod();
    assert!(total_sum > 0.0);
    assert!(total_prod > 0.0);

    let dot = a.dot(&a);
    assert!(dot > 0.0);
}

#[test]
fn eigen_qr() {
    let data = [[2.0, 1.0, 0.0], [1.0, 2.0, 1.0], [0.0, 1.0, 2.0]];
    let a = Matrix::<3, 3>::new(data);
    let (eigenvalues, eigenvectors) = a.eigen();
    let expected = [0.5858_f32, 2.0, 3.4142];
    let mut computed: Vec<f32> = eigenvalues.to_vec();
    computed.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let mut expected_sorted = expected.to_vec();
    expected_sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
    for (c, e) in computed.iter().zip(expected_sorted.iter()) {
        assert!(
            (c - e).abs() < 1e-2,
            "Eigenvalue mismatch: got {}, expected {}",
            c,
            e
        );
    }
    println!("Eigenvalues passed: {:?}", computed);
}

#[test]
fn lu_decomposition() {
    let a = Matrix::<3, 3>::new([[2.0, 1.0, 1.0], [4.0, -6.0, 0.0], [-2.0, 7.0, 2.0]]);
    let (l, u) = a.lu();
    let mut reconstructed = Matrix::<3, 3>::new([[0.0; 3]; 3]);
    for i in 0..3 {
        for j in 0..3 {
            let mut sum = 0.0;
            for k in 0..3 {
                sum += l.inner[i][k] * u.inner[k][j];
            }
            reconstructed.inner[i][j] = sum;
        }
    }
    for i in 0..3 {
        for j in 0..3 {
            let diff = (reconstructed.inner[i][j] - a.inner[i][j]).abs();
            assert!(
                diff < 1e-5,
                "Mismatch at row {}, col {}: {} vs {} (diff={})",
                i,
                j,
                reconstructed.inner[i][j],
                a.inner[i][j],
                diff
            );
        }
    }
}

#[test]
fn solve_linear() {
    let a = Matrix::<3, 3>::new([[2.0, 1.0, 1.0], [4.0, -6.0, 0.0], [-2.0, 7.0, 2.0]]);
    let b = Vector::new([5.0, -2.0, 9.0]);
    let x = a.solve_linear(&b);
    for i in 0..3 {
        let mut sum = 0.0;
        for j in 0..3 {
            sum += a.inner[i][j] * x[j];
        }
        assert!(
            (sum - b[i]).abs() < 1e-5,
            "Mismatch at row {}: {} vs {}",
            i,
            sum,
            b[i]
        );
    }
}

#[test]
fn inverse() {
    let a = Matrix::<3, 3>::new([[4.0, 7.0, 2.0], [3.0, 6.0, 1.0], [2.0, 5.0, 1.0]]);
    let inverse = a.inverse();
    let identity = Matrix::<3, 3>::eye();
    for i in 0..3 {
        for j in 0..3 {
            let mut sum = 0.0;
            for k in 0..3 {
                sum += a.inner[i][k] * inverse.inner[k][j];
            }
            assert!(
                (sum - identity.inner[i][j]).abs() < 1e-5,
                "Mismatch at ({},{}) : {} vs {}",
                i,
                j,
                sum,
                identity.inner[i][j]
            );
        }
    }
}
