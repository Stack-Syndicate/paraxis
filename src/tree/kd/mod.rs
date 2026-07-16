use nalgebra::{RealField, SVector};

pub struct KDTree<T, P, const N: usize> {
    pub data: Vec<(SVector<T, N>, P)>,
}
impl<T: RealField + Copy, P: Clone, const N: usize> KDTree<T, P, N> {
    pub fn new(raw_data: Vec<(SVector<T, N>, P)>) -> Self {
        let mut structured_data = raw_data;
        KDTree::<T, P, N>::build(&mut structured_data);
        Self {
            data: structured_data,
        }
    }
    fn build(raw_data: &mut [(SVector<T, N>, P)]) {
        let mut stack = Vec::with_capacity(64);
        if !raw_data.is_empty() {
            stack.push((0, raw_data.len(), 0));
        }
        while let Some((start, end, depth)) = stack.pop() {
            let len = end - start;
            if len <= 1 {
                continue;
            }
            let axis = depth % N;
            let mid_offset = len / 2;
            let median = start + mid_offset;
            raw_data[start..end].select_nth_unstable_by(mid_offset, |a, b| {
                a.0[axis]
                    .partial_cmp(&b.0[axis])
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
            stack.push((start, median, depth + 1));
            stack.push((median + 1, end, depth + 1));
        }
    }
    pub fn nearest_neighbour(&self, point: SVector<T, N>) -> Result<(SVector<T, N>, P), String> {
        let mut stack = Vec::new();
        if self.data.is_empty() {
            return Err("KDTree is empty!".to_string());
        }
        stack.push((0, self.data.len(), 0));
        let mut closest_id = 0;
        let mut closest_dist = T::max_value().unwrap();
        while let Some((start, end, depth)) = stack.pop() {
            let len = end - start;
            if len == 0 {
                continue;
            }
            if len == 1 {
                let d = point.metric_distance(&self.data[start].0);
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
            let d = point.metric_distance(&median_val.0);
            if d < closest_dist {
                closest_dist = d;
                closest_id = median;
            }
            let point_axis_val = point[axis];
            let median_axis_val = median_val.0[axis];
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
        let closest = self.data[closest_id].clone();
        Ok(closest)
    }
}
