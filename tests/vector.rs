use paraxis::la::vector::Vector;

#[test]
fn vector_arithmetic() {
    let v1 = Vector::from_slice(&[1.0, 2.0, 3.0, 4.0]);
    let v2 = Vector::from_slice(&[4.0, 3.0, 2.0, 1.0]);

    let sum = v1 + v2;
    assert_eq!(sum.inner[0], 5.0);
    assert_eq!(sum.inner[3], 5.0);

    let dot = v1 | v2; // Dot product
    assert_eq!(dot, 20.0); // (1*4 + 2*3 + 3*2 + 4*1)
}

#[test]
fn vector_geometry() {
    let v = Vector::from_slice(&[3.0, 4.0, 0.0, 0.0]);
    assert_eq!(v.length(), 5.0);

    let unit = v.normalize();
    assert_eq!(unit.length(), 1.0);
    assert_eq!(unit.inner[0], 0.6);
}
