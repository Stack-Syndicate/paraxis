use crate::{dsa::tree::skd::SKDTree, maths::vec::Vector};
use num_traits::Float;

impl<T: Float + PartialOrd, P: Clone + Copy, const N: usize> SKDTree<T, P, N> {
    pub fn nearest_neighbour_euclidean(&self, point: &Vector<T, N>) -> (Vector<T, N>, P) {
        let mut stack = Vec::with_capacity(64);
        stack.push((0, self.data.len(), 0));
        let mut closest_id = 0;
        let mut closest_dist = T::max_value();
        while let Some((start, end, depth)) = stack.pop() {
            let len = end - start;
            if len == 0 {
                continue;
            }
            if len == 1 {
                let d = point.dist_euclidean_squared(self.data[start].0);
                if d < closest_dist {
                    closest_dist = d;
                    closest_id = start;
                }
                continue;
            }
            let axis = depth % N;
            let mid_offset = len / 2;
            let median = start + mid_offset;
            let median_val = &self.data[median];
            let d = point.dist_euclidean_squared(median_val.0);
            if d < closest_dist {
                closest_dist = d;
                closest_id = median;
            }
            let point_axis_val = point.inner[axis];
            let median_axis_val = median_val.0.inner[axis];
            let axis_diff = point_axis_val - median_axis_val;
            let left = (start, median);
            let right = (median + 1, end);
            if point_axis_val <= median_axis_val {
                if (axis_diff * axis_diff) < closest_dist {
                    stack.push((right.0, right.1, depth + 1));
                }
                stack.push((left.0, left.1, depth + 1));
            } else {
                if (axis_diff * axis_diff) < closest_dist {
                    stack.push((left.0, left.1, depth + 1));
                }
                stack.push((right.0, right.1, depth + 1));
            }
        }

        self.data[closest_id]
    }
}
