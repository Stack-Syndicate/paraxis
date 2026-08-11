use crate::common::structs::Ray;

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

pub fn intersect_voxel<const N: usize>(
    ray: &Ray<N>,
    point: &[f32; N],
    voxel_size: f32,
    min_dist: f32,
    max_dist: f32,
) -> Option<f32> {
    let half_size = voxel_size * 0.5;
    let mut entry_dist = min_dist;
    let mut exit_dist = max_dist;
    for (i, p) in point.iter().enumerate().take(N) {
        let box_min = p - half_size;
        let box_max = p + half_size;
        let t0 = (box_min - ray.origin[i]) * ray.inv_direction[i];
        let t1 = (box_max - ray.origin[i]) * ray.inv_direction[i];
        let (near_dist, far_dist) = if ray.inv_direction[i] < 0.0 {
            (t1, t0)
        } else {
            (t0, t1)
        };
        entry_dist = entry_dist.max(near_dist);
        exit_dist = exit_dist.min(far_dist);
        if entry_dist > exit_dist {
            return None;
        }
    }
    Some(entry_dist)
}
