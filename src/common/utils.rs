pub fn grid_id<const N: usize>(size: [i32; N], position: [i32; N]) -> usize {
    let mut index = 0usize;
    let mut stride = 1usize;
    for i in (0..N).rev() {
        index += position[i] as usize * stride;
        stride *= size[i] as usize;
    }
    index
}
