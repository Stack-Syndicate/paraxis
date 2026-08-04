pub fn grid_id<const N: usize>(size: [i32; N], position: [i32; N]) -> usize {
    let mut index = 0usize;
    let mut stride = 1usize;
    for i in (0..N).rev() {
        index += position[i] as usize * stride;
        stride *= size[i] as usize;
    }
    index
}

#[inline(always)]
pub fn squared_distance<const N: usize>(a: &[f32; N], b: &[f32; N]) -> f32 {
    let mut sum = 0.0;
    for i in 0..N {
        let diff = a[i] - b[i];
        sum = diff.mul_add(diff, sum);
    }
    sum
}
