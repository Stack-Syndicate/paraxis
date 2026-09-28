use crate::common::structs::Ray;
use num_traits::{Float, PrimInt, ToBytes};
use std::hash::{DefaultHasher, Hash, Hasher};

pub fn grid_id<T: PrimInt, const N: usize>(size: [T; N], position: [T; N]) -> usize {
    let mut index = T::zero();
    let mut stride = T::one();
    for i in (0..N).rev() {
        index = index + position[i] * stride;
        stride = stride * size[i];
    }
    index.to_usize().unwrap()
}

#[inline(always)]
pub fn squared_distance<T: Float, const N: usize>(a: &[T; N], b: &[T; N]) -> T {
    let mut sum = T::zero();
    for i in 0..N {
        let diff = a[i] - b[i];
        sum = diff.mul_add(diff, sum);
    }
    sum
}

pub fn intersect_voxel<T: Float, const N: usize>(
    ray: &Ray<T, N>,
    point: &[T; N],
    voxel_size: T,
    min_dist: T,
    max_dist: T,
) -> Option<T> {
    let half_size = voxel_size * T::from(0.5).unwrap();
    let mut entry_dist = min_dist;
    let mut exit_dist = max_dist;
    for (i, p) in point.iter().enumerate().take(N) {
        let box_min = *p - half_size;
        let box_max = *p + half_size;
        let t0 = (box_min - ray.origin[i]) * ray.inv_direction[i];
        let t1 = (box_max - ray.origin[i]) * ray.inv_direction[i];
        let (near_dist, far_dist) = if ray.inv_direction[i] < T::zero() {
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

pub fn position_key<T: Float + ToBytes, const N: usize>(position: &[T; N]) -> u64 {
    let mut hasher = DefaultHasher::new();
    for &x in position {
        x.to_ne_bytes().hash(&mut hasher);
    }
    hasher.finish()
}
