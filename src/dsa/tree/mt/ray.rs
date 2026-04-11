use crate::{dsa::tree::mt::MortonTree, encoding::MortonCode, maths::vec::Vector};
use num_traits::{Float, FromPrimitive};

impl<T: Float + PartialOrd + FromPrimitive, P, const N: usize> MortonTree<T, P, N> {
    pub fn raycast(
        &self,
        start: Vector<T, N>,
        direction: Vector<T, N>,
        max_distance: T,
        target_depth: usize,
    ) -> Vec<(T, &[(MortonCode<T, N>, P)])> {
        let mut results = Vec::new();
        let direction = direction.normalize();
        let inverse_direction = Vector {
            inner: direction.inner.map(|d| T::from(1.0).unwrap() / d),
        };
        let mut depth_offset = 0;
        let mut distance = T::zero();
        while distance < max_distance {
            let position = start + direction * distance;
            let cell_size = T::from(self.cell_sizes[depth_offset]).unwrap();
            let mut step_distance = max_distance;
            let mut axis_hit = false;
            for i in 0..N {
                let positive_axis_position = position.inner[i] - self.min.inner[i];
                let cell_id = (positive_axis_position / cell_size).floor();
                let next_plane_distance = if direction.inner[i] > T::zero() {
                    (cell_id + T::one()) * cell_size
                } else if direction.inner[i] < T::zero() {
                    cell_id * cell_size
                } else {
                    continue;
                };
                let mut distance_to_plane =
                    (next_plane_distance - positive_axis_position) * inverse_direction.inner[i];
                if distance_to_plane <= T::zero() {
                    distance_to_plane = cell_size * inverse_direction.inner[i].abs()
                }
                if distance_to_plane < step_distance {
                    step_distance = distance_to_plane;
                    axis_hit = true;
                }
            }
            if !axis_hit {
                break;
            }
            let mid_cell_distance = step_distance * T::from(0.5).unwrap();
            let mid_cell_position = position + (direction * mid_cell_distance);
            let entities_at_depth = self.search_level(&mid_cell_position, depth_offset);
            if entities_at_depth.is_empty() {
                distance = distance + step_distance;
                depth_offset = (depth_offset + 1).min(32);
            } else {
                if depth_offset <= target_depth {
                    results.push((distance, entities_at_depth));
                    distance = distance + step_distance;
                } else {
                    depth_offset -= 1;
                }
            }
        }
        results
    }
}
