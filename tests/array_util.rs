use paraxis::containers::array::Array;

#[test]
pub fn linspace() {
    let v1 = Array::linspace(10.0, 20.0, 10, false);
    let v2 = Array::linspace(10.0, 20.0, 11, true);
    assert_eq!(
        v1,
        Array::from_vec((10..20).map(|n| n as f32).collect::<Vec<_>>())
    );
    assert_eq!(
        v2,
        Array::from_vec((10..21).map(|n| n as f32).collect::<Vec<_>>())
    );
}
