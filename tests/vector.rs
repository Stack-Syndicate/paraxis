use nalgebra::SVector;
use paraxis::maths::la::vector::Vector;

type NAVector<T, const N: usize> = SVector<T, N>;

#[test]
fn vector_arithmetic() {
    let v1 = Vector::from_slice(&[1.0, 2.0, 3.0, 4.0]);
    let v2 = Vector::from_slice(&[4.0, 3.0, 2.0, 1.0]);
    let nv1 = NAVector::<f64, 4>::new(1.0, 2.0, 3.0, 4.0);
    let nv2 = NAVector::<f64, 4>::new(4.0, 3.0, 2.0, 1.0);
    let dot = v1 | v2;
    let nadot = nv1.dot(&nv2);
    assert_eq!(dot, nadot);
}

#[test]
fn vector_geometry() {
    let v = Vector::from_slice(&[3.0, 4.0, 0.0, 0.0]);
    assert_eq!(v.length(), 5.0);
    let unit = v.normalize();
    assert_eq!(unit.length(), 1.0);
    assert_eq!(unit.inner[0], 0.6);
}
