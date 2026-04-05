use crate::maths::vec::Vector;
use num_traits::Num;

mod float;
mod signed;

pub struct KDTree<T: Num, P, const N: usize> {
    pub data: Vec<(Vector<T, N>, P)>,
}
impl<T: Num + PartialOrd + Clone, P: Clone, const N: usize> KDTree<T, P, N> {
    pub fn new(raw_data: &[(Vector<T, N>, P)]) -> Self {
        let mut owned_data = (&raw_data).to_vec();
        if !owned_data.is_empty() {
            Self::build_iterative(&mut owned_data);
        }
        Self { data: owned_data }
    }
    fn build_iterative(data: &mut [(Vector<T, N>, P)]) {
        let mut stack = Vec::new();
        if !data.is_empty() {
            stack.push((0, data.len(), 0));
        }
        while let Some((start, end, depth)) = stack.pop() {
            let len = end - start;
            if len <= 1 {
                continue;
            };
            let axis = depth % N;
            let mid_relative = len / 2;
            let mid_absolute = start + mid_relative;
            let segment = &mut data[start..end];
            segment.select_nth_unstable_by(mid_relative, |a, b| {
                a.0.inner[axis]
                    .partial_cmp(&b.0.inner[axis])
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
            stack.push((start, mid_absolute, depth + 1));
            stack.push((mid_absolute + 1, end, depth + 1));
        }
    }
}
