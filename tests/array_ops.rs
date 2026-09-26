use paraxis::containers::array::Array;

#[test]
fn add_array() {
    let v1 = Array::from_vec(vec![1, 1, 1]);
    let v2 = Array::from_vec(vec![2, 2, 2]);
    let v3 = v1 + v2;
    assert_eq!(v3.to_vec(), vec![3, 3, 3])
}
#[test]
fn sub_array() {
    let v1 = Array::from_vec(vec![2, 2, 2]);
    let v2 = Array::from_vec(vec![1, 1, 1]);
    let v3 = v1 - v2;
    assert_eq!(v3.to_vec(), vec![1, 1, 1])
}
#[test]
fn mul_array() {
    let v1 = Array::from_vec(vec![1, 2, 1]);
    let v2 = Array::from_vec(vec![2, 3, 4]);
    let v3 = v1 * v2;
    assert_eq!(v3.to_vec(), vec![2, 6, 4])
}
#[test]
fn div_array() {
    let v1 = Array::from_vec(vec![4, 4, 4]);
    let v2 = Array::from_vec(vec![2, 2, 2]);
    let v3 = v1 / v2;
    assert_eq!(v3.to_vec(), vec![2, 2, 2])
}
#[test]
fn add_scalar() {
    let v = Array::from_vec(vec![1, 1, 1]);
    let s = 10;
    let r = v + s;
    assert_eq!(r.to_vec(), vec![11, 11, 11])
}
#[test]
fn sub_scalar() {
    let v = Array::from_vec(vec![1, 1, 1]);
    let s = 10;
    let r = v - s;
    assert_eq!(r.to_vec(), vec![-9, -9, -9])
}
#[test]
fn mul_scalar() {
    let v = Array::from_vec(vec![1, 1, 1]);
    let s = 10;
    let r = v * s;
    assert_eq!(r.to_vec(), vec![10, 10, 10])
}
#[test]
fn div_scalar() {
    let v = Array::from_vec(vec![1.0, 1.0, 1.0]);
    let s = 10.0;
    let r = v / s;
    assert_eq!(r.to_vec(), vec![0.1, 0.1, 0.1])
}
