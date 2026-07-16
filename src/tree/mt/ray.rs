use std::fmt::Debug;

use crate::{encoding::MortonCode, tree::mt::MortonTree};
use nalgebra::{RealField, SVector, Scalar};
use num_traits::{FromPrimitive, ToPrimitive};

pub struct RaycastIter<'a, T, P, const N: usize>
where
    T: Scalar + Debug + RealField + Copy + ToPrimitive,
{
    tree: &'a MortonTree<T, P, N>,
    start: SVector<T, N>,
    direction: SVector<T, N>,
    inverse_direction: SVector<T, N>,
    max_distance: T,
    target_depth: usize,
    distance: T,
    depth_offset: usize,
}
impl<'a, T: PartialOrd + ToPrimitive + Copy + Scalar + RealField, P, const N: usize>
    MortonTree<T, P, N>
{
    pub fn raycast_iter(
        &'a self,
        start: SVector<T, N>,
        direction: SVector<T, N>,
        max_distance: T,
        target_depth: usize,
    ) -> RaycastIter<'a, T, P, N> {
        let direction = direction.normalize();
        let inverse_direction = direction.map(|d| T::one() / d);
        RaycastIter {
            tree: self,
            start,
            direction,
            inverse_direction,
            max_distance,
            target_depth,
            distance: T::zero(),
            depth_offset: 0,
        }
    }
}
impl<'a, T, P, const N: usize> Iterator for RaycastIter<'a, T, P, N>
where
    T: Scalar + RealField + PartialOrd + FromPrimitive + Copy + ToPrimitive,
{
    type Item = (T, &'a [(MortonCode<T, N>, P)]);
    fn next(&mut self) -> Option<Self::Item> {
        while self.distance < self.max_distance {
            let position = self.start + self.direction * self.distance;
            let cell_size = T::from_u32(self.tree.cell_sizes[self.depth_offset]).unwrap();
            let mut step_distance = self.max_distance;
            let mut axis_hit = false;
            for i in 0..N {
                let positive_axis_position = position[i] - self.tree.min[i];
                let cell_id = (positive_axis_position / cell_size).floor();
                let next_plane_distance = if self.direction[i] > T::zero() {
                    (cell_id + T::one()) * cell_size
                } else if self.direction[i] < T::zero() {
                    cell_id * cell_size
                } else {
                    continue;
                };
                let mut distance_to_plane =
                    (next_plane_distance - positive_axis_position) * self.inverse_direction[i];
                if distance_to_plane <= T::zero() {
                    distance_to_plane = cell_size * self.inverse_direction[i].abs();
                }
                if distance_to_plane < step_distance {
                    step_distance = distance_to_plane;
                    axis_hit = true;
                }
            }
            if !axis_hit {
                return None;
            }
            let mid_cell_distance = step_distance * T::from_f32(0.5).unwrap();
            let mid_cell_position = position + (self.direction * mid_cell_distance);
            let entities = self
                .tree
                .search_level(&mid_cell_position, self.depth_offset);
            if entities.is_empty() {
                self.distance += step_distance;
                self.depth_offset = (self.depth_offset + 1).min(32);
            } else {
                if self.depth_offset <= self.target_depth {
                    let result = Some((self.distance, entities));
                    self.distance += step_distance;
                    return result;
                } else {
                    self.depth_offset -= 1;
                }
            }
        }
        None
    }
}

impl<T: Scalar + PartialOrd + FromPrimitive + ToPrimitive + RealField + Copy, P, const N: usize>
    MortonTree<T, P, N>
{
    pub fn raycast(
        &self,
        start: SVector<T, N>,
        direction: SVector<T, N>,
        max_distance: T,
        target_depth: usize,
    ) -> Vec<(T, &[(MortonCode<T, N>, P)])> {
        let mut results = Vec::new();
        let direction = direction.normalize();
        let inverse_direction = direction.map(|d| T::one() / d);
        let mut depth_offset = 0;
        let mut distance = T::zero();
        while distance < max_distance {
            let position = start + direction * distance;
            let cell_size = T::from_u32(self.cell_sizes[depth_offset]).unwrap();
            let mut step_distance = max_distance;
            let mut axis_hit = false;
            for i in 0..N {
                let positive_axis_position = position[i] - self.min[i];
                let cell_id = (positive_axis_position / cell_size).floor();
                let next_plane_distance = if direction[i] > T::zero() {
                    (cell_id + T::one()) * cell_size
                } else if direction[i] < T::zero() {
                    cell_id * cell_size
                } else {
                    continue;
                };
                let mut distance_to_plane =
                    (next_plane_distance - positive_axis_position) * inverse_direction[i];
                if distance_to_plane <= T::zero() {
                    distance_to_plane = cell_size * inverse_direction[i].abs()
                }
                if distance_to_plane < step_distance {
                    step_distance = distance_to_plane;
                    axis_hit = true;
                }
            }
            if !axis_hit {
                break;
            }
            let mid_cell_distance = step_distance * T::from_f32(0.5).unwrap();
            let mid_cell_position = position + (direction * mid_cell_distance);
            let entities_at_depth = self.search_level(&mid_cell_position, depth_offset);
            if entities_at_depth.is_empty() {
                distance += step_distance;
                depth_offset = (depth_offset + 1).min(32);
            } else {
                if depth_offset <= target_depth {
                    results.push((distance, entities_at_depth));
                    distance += step_distance;
                } else {
                    depth_offset -= 1;
                }
            }
        }
        results
    }
}
