use std::iter::zip;

pub fn grid_id<const N: usize>(size: [i32; N], position: [i32; N]) -> usize {
    (position[0]
        + zip(
            position[0..position.len()].iter(),
            size[0..size.len() - 1].iter(),
        )
        .map(|(p, s)| p * s)
        .sum::<i32>()) as usize
}
