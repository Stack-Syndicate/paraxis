use crate::common::{
    errors::ParaxisError, structs::Node, traits::Tree, utils::squared_distance_int,
};

pub struct KDTree<P, D> {
    data: Vec<Node<P, D>>,
    insertions: usize,
}
impl<D: Clone, const N: usize> Tree<[i32; N], D> for KDTree<[i32; N], D> {
    fn new(mut raw_data: Vec<([i32; N], D)>) -> Self {
        let data_len = raw_data.len();
        if data_len == 0 {
            return Self {
                data: Vec::new(),
                insertions: 0,
            };
        }
        let mut data = Vec::with_capacity(data_len);
        struct StackItem<'a, D, const N: usize> {
            list: &'a mut [([i32; N], D)],
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
                .select_nth_unstable_by(median_index, |a, b| a.0[axis].cmp(&b.0[axis]));
            let (position, payload) = unsafe { std::ptr::read(median) };
            let current_index = data.len();
            data.push(Node {
                position,
                inner: Some(payload),
                next: None,
                prev: None,
            });
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
    fn add(&mut self, position: &[i32; N], data: D) {
        if self.data.is_empty() {
            self.data.push(Node {
                position: *position,
                inner: Some(data),
                next: None,
                prev: None,
            });
            return;
        }
        self.insertions += 1;
        let mut current_index = 0;
        while let Some(node) = self.data.get(current_index) {
            let left_opt = node.prev;
            let right_opt = node.next;
            match (left_opt, right_opt) {
                (Some(left_index), Some(right_index)) => {
                    let dist_left = squared_distance_int(position, &self.data[left_index].position);
                    let dist_right =
                        squared_distance_int(position, &self.data[right_index].position);

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
                    self.data.push(Node {
                        position: *position,
                        inner: Some(data),
                        next: None,
                        prev: None,
                    });
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
        let items: Vec<([i32; N], D)> = old_data
            .into_iter()
            .filter_map(|node| node.inner.map(|inner| (node.position, inner)))
            .collect();
        if items.is_empty() {
            return;
        }
        let mut new_tree = Self::new(items);
        new_tree.insertions = 0;
        *self = new_tree;
    }
    fn nearest_neighbour(&self, position: &[i32; N]) -> Result<&Node<[i32; N], D>, ParaxisError> {
        let mut best_index = 0;
        let mut best_distance = i32::MAX;
        let mut stack = vec![(0, 0)];
        while let Some((index, depth)) = stack.pop() {
            let node = &self.data[index];
            let dist = squared_distance_int(&node.position, position);
            if dist < best_distance {
                best_distance = dist;
                best_index = index;
            }
            let axis = depth % N;
            let diff = (position[axis] - node.position[axis]) as i64;
            let (near, far) = if diff <= 0 {
                (node.prev, node.next)
            } else {
                (node.next, node.prev)
            };
            if diff * diff < best_distance.into()
                && let Some(far_index) = far
            {
                stack.push((far_index, depth + 1));
            }
            if let Some(near_index) = near {
                stack.push((near_index, depth + 1));
            }
        }

        Ok(&self.data[best_index])
    }
    fn nearest_neighbour_mut(
        &mut self,
        position: &[i32; N],
    ) -> Result<&mut Node<[i32; N], D>, ParaxisError> {
        let mut best_index = 0;
        let mut best_distance = i32::MAX;
        let mut stack = vec![(0, 0)];
        while let Some((index, depth)) = stack.pop() {
            let node = &self.data[index];
            let dist = squared_distance_int(&node.position, position);
            if dist < best_distance {
                best_distance = dist;
                best_index = index;
            }
            let axis = depth % N;
            let diff = (position[axis] - node.position[axis]) as i64;
            let (near, far) = if diff <= 0 {
                (node.prev, node.next)
            } else {
                (node.next, node.prev)
            };
            if diff * diff < best_distance.into()
                && let Some(far_index) = far
            {
                stack.push((far_index, depth + 1));
            }
            if let Some(near_index) = near {
                stack.push((near_index, depth + 1));
            }
        }

        Ok(&mut self.data[best_index])
    }
}
