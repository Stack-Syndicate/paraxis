use crate::maths::vec::Vector;
use num_traits::Num;
mod float;
mod signed;

pub struct SKDTree<T: Num, P, const N: usize> {
    pub data: Vec<(Vector<T, N>, P)>,
    pub split_count: usize,
}
impl<T: Num + PartialOrd + Clone, P: Clone, const N: usize> SKDTree<T, P, N> {
    pub fn new(raw_data: &[(Vector<T, N>, P)], split_count: usize) -> Self {
        let mut owned_data = (&raw_data).to_vec();
        if !owned_data.is_empty() {
            Self::build_iterative(&mut owned_data, split_count);
        }
        Self {
            data: owned_data,
            split_count,
        }
    }
    fn build_iterative(data: &mut [(Vector<T, N>, P)], _split_count: usize) {
        let mut stack = Vec::with_capacity(64);
        if !data.is_empty() {
            stack.push((0, data.len(), 0));
        }
        while let Some((start, end, depth)) = stack.pop() {
            let len = end - start;
            if len <= 1 {
                continue;
            }
            let axis = depth % N;
            let mid_offset = len / 2;
            let median = start + mid_offset;
            data[start..end].select_nth_unstable_by(mid_offset, |a, b| {
                a.0.inner[axis]
                    .partial_cmp(&b.0.inner[axis])
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
            stack.push((start, median, depth + 1));
            stack.push((median + 1, end, depth + 1));
        }
    }
}
