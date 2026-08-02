use ordered_float::OrderedFloat;

use crate::common::{errors::ParaxisError, structs::Node, traits::Tree, utils::squared_distance};
use std::collections::BinaryHeap;

pub struct KDTree<P, D> {
    data: Vec<Node<P, D>>,
    insertions: usize,
}
impl<D: Clone, const N: usize> Tree<[f32; N], D> for KDTree<[f32; N], D> {
    fn new(mut raw_data: Vec<([f32; N], D)>) -> Self {
        let data_len = raw_data.len();
        if data_len == 0 {
            return Self {
                data: Vec::new(),
                insertions: 0,
            };
        }
        let mut data = Vec::with_capacity(data_len);
        struct StackItem<'a, D, const N: usize> {
            list: &'a mut [([f32; N], D)],
            depth: usize,
            parent_opt: Option<usize>,
            is_left: bool,
        }
        let mut stack = vec![StackItem {
            list: raw_data.as_mut_slice(),
            depth: 0,
            parent_opt: None,
            is_left: false,
        }];
        while let Some(item) = stack.pop() {
            if item.list.is_empty() {
                continue;
            }
            let axis = item.depth % N;
            let median_index = item.list.len() / 2;
            let (left, median, right) = item
                .list
                .select_nth_unstable_by(median_index, |a, b| a.0[axis].total_cmp(&b.0[axis]));
            let (position, payload) = unsafe { std::ptr::read(median) };
            let current_index = data.len();
            data.push(Node::new(position, Some(payload)));
            if let Some(parent_index) = item.parent_opt {
                if item.is_left {
                    data[parent_index].prev = Some(current_index);
                } else {
                    data[parent_index].next = Some(current_index);
                }
            }
            if !right.is_empty() {
                stack.push(StackItem {
                    list: right,
                    depth: item.depth + 1,
                    parent_opt: Some(current_index),
                    is_left: false,
                });
            }
            if !left.is_empty() {
                stack.push(StackItem {
                    list: left,
                    depth: item.depth + 1,
                    parent_opt: Some(current_index),
                    is_left: true,
                });
            }
        }
        unsafe {
            raw_data.set_len(0);
        }
        Self {
            data,
            insertions: 0,
        }
    }
    fn add(&mut self, position: &[f32; N], data: D) {
        if self.data.is_empty() {
            self.data.push(Node::new(*position, Some(data)));
            return;
        }
        self.insertions += 1;
        let mut current_index = 0;
        while let Some(node) = self.data.get(current_index) {
            let left_opt = node.prev;
            let right_opt = node.next;
            match (left_opt, right_opt) {
                (Some(left_index), Some(right_index)) => {
                    let dist_left = squared_distance(position, &self.data[left_index].position);
                    let dist_right = squared_distance(position, &self.data[right_index].position);

                    if dist_left <= dist_right {
                        current_index = left_index;
                    } else {
                        current_index = right_index;
                    }
                }
                (Some(left_index), None) => {
                    current_index = left_index;
                }
                (None, Some(right_index)) => {
                    current_index = right_index;
                }
                (None, None) => {
                    self.data[current_index].next = Some(self.data.len());
                    self.data.push(Node::new(*position, Some(data)));
                    break;
                }
            }
        }
        if self.insertions > N * N {
            self.rebalance();
        }
    }
    fn rebalance(&mut self) {
        if self.data.is_empty() {
            return;
        }
        let old_data = std::mem::take(&mut self.data);
        let items: Vec<([f32; N], D)> = old_data
            .into_iter()
            .filter_map(|node| {
                node.read()
                    .inner
                    .clone()
                    .map(|inner| (node.position, inner))
            })
            .collect();
        if items.is_empty() {
            return;
        }
        let mut new_tree = Self::new(items);
        new_tree.insertions = 0;
        *self = new_tree;
    }
    fn k_nearest_neighbours(
        &self,
        position: &[f32; N],
        k: usize,
    ) -> Result<Vec<&Node<[f32; N], D>>, ParaxisError> {
        if k == 0 {
            return Ok(vec![]);
        }
        let mut heap: BinaryHeap<(OrderedFloat<f32>, usize)> = BinaryHeap::with_capacity(k);
        let mut stack = [(0, 0); 128];
        let mut stack_pointer = 1;
        let mut worst_dist = f32::MAX;
        while stack_pointer > 0 {
            stack_pointer -= 1;
            let (index, depth) = stack[stack_pointer];
            let node = &self.data[index];
            let dist = squared_distance(&node.position, position);
            let ordered_dist = OrderedFloat::from(dist);
            if heap.len() < k {
                heap.push((ordered_dist, index));
                if heap.len() == k {
                    worst_dist = heap.peek().unwrap().0.into_inner();
                }
            } else if dist < worst_dist {
                heap.pop();
                heap.push((ordered_dist, index));
                worst_dist = heap.peek().unwrap().0.into_inner();
            }
            let axis = depth % N;
            let diff = position[axis] - node.position[axis];
            let axis_dist = diff * diff;
            let (near, far) = if diff <= 0.0 {
                (node.prev, node.next)
            } else {
                (node.next, node.prev)
            };
            if let Some(near_index) = near {
                stack[stack_pointer] = (near_index, depth + 1);
                stack_pointer += 1;
            }
            if axis_dist < worst_dist
                && let Some(far_index) = far
            {
                stack[stack_pointer] = (far_index, depth + 1);
                stack_pointer += 1;
            }
        }
        let result: Vec<&Node<[f32; N], D>> =
            heap.into_iter().map(|(_, idx)| &self.data[idx]).collect();
        Ok(result)
    }
}
