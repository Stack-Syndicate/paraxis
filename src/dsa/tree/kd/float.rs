use num_traits::Float;

use crate::{dsa::tree::kd::KDTree, maths::vec::Vector};

impl<T: Float + PartialOrd, P: Clone, const N: usize> KDTree<T, P, N> {
    pub fn nearest_neighbour_euclidean(&self, point: Vector<T, N>) -> (Vector<T, N>, P) {
        let mut stack = Vec::new();
        stack.push((0, self.data.len(), 0));
        let mut closest_id = 0;
        let mut closest_dist = T::max_value();
        while let Some((start, end, depth)) = stack.pop() {
            if start >= end {
                continue;
            }
            let axis = depth % N;
            let mid = start + (end - start) / 2;
            let mid_val = &self.data[mid];
            let d = point.dist_euclidean(mid_val.0);
            if d < closest_dist {
                closest_dist = d;
                closest_id = mid;
            }
            let axis_diff = point.inner[axis] - mid_val.0.inner[axis];
            let (near, far) = if axis_diff <= T::zero() {
                ((start, mid), (mid + 1, end))
            } else {
                ((mid + 1, end), (start, mid))
            };
            if axis_diff.abs() < closest_dist {
                stack.push((far.0, far.1, depth + 1));
            }
            stack.push((near.0, near.1, depth + 1));
        }
        return self.data[closest_id].clone();
    }
}
