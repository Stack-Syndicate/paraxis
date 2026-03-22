use crate::maths::la::vector::Vector;

pub mod matrix;
pub mod vector;

pub fn unflatten_index<const N: usize>(mut index: usize, shape: &[usize; N]) -> Vector<i32, N> {
    let mut coords = [0i32; N];
    for i in (0..N).rev() {
        coords[i] = (index % shape[i]) as i32;
        index /= shape[i];
    }
    Vector::from(coords)
}
pub fn flatten_index<const N: usize>(pos: Vector<i32, N>, shape: &[usize; N]) -> usize {
    let mut index = 0;
    let mut stride = 1;
    for i in (0..N).rev() {
        index += pos.inner[i] as usize * stride;
        stride *= shape[i];
    }
    index
}
