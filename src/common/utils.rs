use std::{
    iter::Sum,
    ops::{Mul, Sub},
};

pub fn grid_id<const N: usize>(size: [i32; N], position: [i32; N]) -> usize {
    let mut index = 0usize;
    let mut stride = 1usize;
    for i in (0..N).rev() {
        index += position[i] as usize * stride;
        stride *= size[i] as usize;
    }
    index
}

pub fn squared_distance<T: Sub<T, Output = T> + Mul<T, Output = T> + Sum + Copy, const N: usize>(
    a: &[T; N],
    b: &[T; N],
) -> T {
    a.iter()
        .zip(b.iter())
        .map(|(&x, &y)| {
            let diff = x - y;
            diff * diff
        })
        .sum()
}
