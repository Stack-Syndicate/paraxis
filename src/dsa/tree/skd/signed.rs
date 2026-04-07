use num_traits::{Bounded, PrimInt, Signed};

use crate::{dsa::tree::skd::SKDTree, maths::vec::Vector};

impl<T: PrimInt + Signed + Bounded + PartialOrd + Clone + Copy, P: Clone, const N: usize>
    SKDTree<T, P, N>
{
    pub fn nearest_neighbour_manhattan(&self, point: Vector<T, N>) -> (Vector<T, N>, P) {
        let mut stack = Vec::new();
        stack.push((0, self.data.len(), 0));
        let mut closest_id = 0;
        let mut closest_dist = T::max_value();
        while let Some((start, end, depth)) = stack.pop() {
            if start >= end {
                continue;
            }
            let len = end - start;
            if len == 1 {
                let d = point.dist_manhattan(self.data[start].0);
                if d < closest_dist {
                    closest_dist = d;
                    closest_id = start;
                }
                continue;
            }
            let axis = depth % N;
            let capped_split_count = self.split_count.min(len - 1);
            let splits = (1..=capped_split_count)
                .map(|s| (s * len) / (capped_split_count + 1))
                .collect::<Vec<_>>();
            let mut previous_split = 0;
            for split in splits {
                let split_id = start + split;
                if split_id >= end {
                    previous_split = split + 1;
                    continue;
                }
                let split_val = &self.data[split_id];
                let d = point.dist_manhattan(split_val.0);
                if d < closest_dist {
                    closest_dist = d;
                    closest_id = split_id;
                }
                let axis_diff = point.inner[axis] - split_val.0.inner[axis];
                let (near_start, near_end, far_start, far_end) = if axis_diff <= T::zero() {
                    (start + previous_split, split_id, split_id + 1, end)
                } else {
                    (split_id + 1, end, start + previous_split, split_id)
                };
                if axis_diff.abs() < closest_dist {
                    stack.push((far_start, far_end, depth + 1));
                }
                stack.push((near_start, near_end, depth + 1));
                previous_split = split + 1;
            }
            if previous_split < len {
                stack.push((start + previous_split, end, depth + 1));
            }
        }
        return self.data[closest_id].clone();
    }
}
