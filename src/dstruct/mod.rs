use crate::maths::la::{self, vector::Vector};

pub mod gridmap;
pub mod kdtree;

pub fn unflatten_slice<T, const N: usize>(
    slice: &[T],
    shape: [usize; N],
) -> Vec<(Vector<i32, N>, &T)> {
    slice
        .iter()
        .enumerate()
        .map(|(i, item)| {
            let coords = la::unflatten_index(i, &shape);
            (coords, item)
        })
        .collect::<Vec<_>>()
}
