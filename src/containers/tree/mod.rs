use crate::common::{
    errors::ParaxisError,
    structs::{Node, Ray},
    traits::Tree,
    utils::{intersect_voxel, squared_distance},
};

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
                    data[parent_index].prev = current_index;
                } else {
                    data[parent_index].next = current_index;
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
        let mut depth = 0;
        loop {
            let axis = depth % N;
            let node = &self.data[current_index];
            let go_prev = position[axis] <= node.position[axis];
            let child = if go_prev { node.prev } else { node.next };
            if child == usize::MAX {
                let new_index = self.data.len();
                self.data.push(Node::new(*position, Some(data)));
                if go_prev {
                    self.data[current_index].prev = new_index;
                } else {
                    self.data[current_index].next = new_index;
                }
                break;
            }
            current_index = child;
            depth += 1;
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
        let mut best_dists = vec![(f32::MAX, 0); k];
        let mut found = 0;
        let mut stack = [(0, 0, 0); 128];
        let mut stack_pointer = 1;
        let mut worst_dist = f32::MAX;
        while stack_pointer > 0 {
            stack_pointer -= 1;
            let (index, depth, axis) = stack[stack_pointer];
            let next_axis = if axis + 1 == N { 0 } else { axis + 1 };
            let node = &self.data[index];
            let dist = squared_distance(&node.position, position);
            if found < k {
                let mut i = found;
                while i > 0 && dist < best_dists[i - 1].0 {
                    best_dists[i] = best_dists[i - 1];
                    i -= 1;
                }
                best_dists[i] = (dist, index);
                found += 1;

                if found == k {
                    worst_dist = best_dists[k - 1].0;
                }
            } else if dist < worst_dist {
                let mut i = k - 1;
                while i > 0 && dist < best_dists[i - 1].0 {
                    best_dists[i] = best_dists[i - 1];
                    i -= 1;
                }
                best_dists[i] = (dist, index);
                worst_dist = best_dists[k - 1].0;
            }
            let diff = position[axis] - node.position[axis];
            let axis_dist = diff * diff;
            let (near, far) = if diff <= 0.0 {
                (node.prev, node.next)
            } else {
                (node.next, node.prev)
            };
            if axis_dist < worst_dist && far != usize::MAX {
                stack[stack_pointer] = (far, depth + 1, next_axis);
                stack_pointer += 1;
            }
            if near != usize::MAX {
                stack[stack_pointer] = (near, depth + 1, next_axis);
                stack_pointer += 1;
            }
        }
        let mut out = Vec::with_capacity(found);
        for &(_, index) in &best_dists[..found] {
            out.push(&self.data[index]);
        }
        Ok(out)
    }
}

pub struct BIHierarchy<P, D> {
    pub data: Vec<Node<P, D>>,
    insertions: usize,
}
impl<D: Clone, const N: usize> Tree<[f32; N], D> for BIHierarchy<[f32; N], D> {
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
            parent_opt: Option<usize>,
            is_left: bool,
        }
        let mut stack = vec![StackItem {
            list: raw_data.as_mut_slice(),
            parent_opt: None,
            is_left: false,
        }];
        while let Some(item) = stack.pop() {
            if item.list.is_empty() {
                continue;
            }
            let current_index = data.len();

            if item.list.len() == 1 {
                let position = item.list[0].0;
                let payload = unsafe { std::ptr::read(&item.list[0].1) };
                data.push(Node::new(position, Some(payload)));
                if let Some(parent_index) = item.parent_opt {
                    if item.is_left {
                        data[parent_index].prev = current_index;
                    } else {
                        data[parent_index].next = current_index;
                    }
                }
                continue;
            }

            let (axis, _, min, max) = (0..N)
                .map(|axis| {
                    let (min, max) = item
                        .list
                        .iter()
                        .fold((f32::INFINITY, -f32::INFINITY), |(min, max), p| {
                            (min.min(p.0[axis]), max.max(p.0[axis]))
                        });
                    (axis, max - min, min, max)
                })
                .max_by(|a, b| a.1.total_cmp(&b.1))
                .unwrap();

            let split_plane = (min + max) * 0.5;

            let middle_index = item
                .list
                .iter_mut()
                .partition_in_place(|p| p.0[axis] <= split_plane);

            let middle_index = if middle_index == 0 || middle_index == item.list.len() {
                item.list.len() / 2
            } else {
                middle_index
            };

            let (left, right) = item.list.split_at_mut(middle_index);

            let l_max = left
                .iter()
                .map(|p| p.0[axis])
                .fold(-f32::INFINITY, f32::max);

            let r_min = right
                .iter()
                .map(|p| p.0[axis])
                .fold(f32::INFINITY, f32::min);

            let mut l_bound = [0.0; N];
            let mut r_bound = [0.0; N];
            l_bound[axis] = l_max;
            r_bound[axis] = r_min;
            let mut position = [0.0; N];
            position[0] = axis as f32;
            data.push(Node::new_bounded(position, (l_bound, r_bound), None));
            if let Some(parent_index) = item.parent_opt {
                if item.is_left {
                    data[parent_index].prev = current_index;
                } else {
                    data[parent_index].next = current_index;
                }
            }
            if !right.is_empty() {
                stack.push(StackItem {
                    list: right,
                    parent_opt: Some(current_index),
                    is_left: false,
                });
            }
            if !left.is_empty() {
                stack.push(StackItem {
                    list: left,
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
        let mut depth = 0;
        loop {
            let axis = depth % N;
            let node = &self.data[current_index];
            let go_prev = position[axis] <= node.position[axis];
            let child = if go_prev { node.prev } else { node.next };
            if child == usize::MAX {
                let new_index = self.data.len();
                self.data.push(Node::new(*position, Some(data)));
                if go_prev {
                    self.data[current_index].prev = new_index;
                } else {
                    self.data[current_index].next = new_index;
                }
                break;
            }
            current_index = child;
            depth += 1;
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
        if k == 0 || self.data.is_empty() {
            return Ok(vec![]);
        }
        let mut best_dists = vec![(f32::MAX, 0); k];
        let mut found = 0;
        let mut stack = [0; 128];
        let mut stack_pointer = 1;
        let mut worst_dist = f32::MAX;

        while stack_pointer > 0 {
            stack_pointer -= 1;
            let index = stack[stack_pointer];
            let node = &self.data[index];
            if let Some((l_bound, r_bound)) = &node.bounds {
                let axis = node.position[0] as usize;
                let l_max = l_bound[axis];
                let r_min = r_bound[axis];
                let q_val = position[axis];

                let left_dist_1d = if q_val > l_max {
                    (q_val - l_max) * (q_val - l_max)
                } else {
                    0.0
                };

                let right_dist_1d = if q_val < r_min {
                    (r_min - q_val) * (r_min - q_val)
                } else {
                    0.0
                };

                let (near_child, near_dist, far_child, far_dist) = if q_val <= (l_max + r_min) * 0.5
                {
                    (node.prev, left_dist_1d, node.next, right_dist_1d)
                } else {
                    (node.next, right_dist_1d, node.prev, left_dist_1d)
                };

                if far_child != usize::MAX && (found < k || far_dist < worst_dist) {
                    stack[stack_pointer] = far_child;
                    stack_pointer += 1;
                }

                if near_child != usize::MAX && (found < k || near_dist < worst_dist) {
                    stack[stack_pointer] = near_child;
                    stack_pointer += 1;
                }
            } else {
                let dist = squared_distance(&node.position, position);

                if found < k {
                    let mut i = found;

                    while i > 0 && dist < best_dists[i - 1].0 {
                        best_dists[i] = best_dists[i - 1];
                        i -= 1;
                    }
                    best_dists[i] = (dist, index);
                    found += 1;

                    if found == k {
                        worst_dist = best_dists[k - 1].0;
                    }
                } else if dist < worst_dist {
                    let mut i = k - 1;

                    while i > 0 && dist < best_dists[i - 1].0 {
                        best_dists[i] = best_dists[i - 1];
                        i -= 1;
                    }
                    best_dists[i] = (dist, index);
                    worst_dist = best_dists[k - 1].0;
                }
            }
        }

        let mut out = Vec::with_capacity(found);
        for &(_, index) in &best_dists[..found] {
            out.push(&self.data[index]);
        }
        Ok(out)
    }
}
impl<const N: usize, D> BIHierarchy<[f32; N], D> {
    pub fn trace_ray(
        &self,
        origin: [f32; N],
        direction: [f32; N],
        min_dist: f32,
        max_dist: f32,
        voxel_size: f32,
    ) -> Option<(f32, &Node<[f32; N], D>)> {
        if self.data.is_empty() {
            return None;
        }
        let mut max_dist = max_dist;
        let ray = Ray::new(origin, direction);
        let mut stack = [(0usize, min_dist, max_dist); 128];
        let mut stack_ptr = 1;
        let mut closest_hit: Option<(f32, &Node<[f32; N], D>)> = None;

        let radius = voxel_size * 0.5;

        while stack_ptr > 0 {
            stack_ptr -= 1;
            let (node_index, node_min_dist, mut node_max_dist) = stack[stack_ptr];
            node_max_dist = node_max_dist.min(max_dist);

            if node_min_dist >= node_max_dist {
                continue;
            }

            let node = &self.data[node_index];
            if let Some((l_bound, r_bound)) = &node.bounds {
                let axis = node.position[0] as usize;

                let l_max = l_bound[axis] + radius;
                let r_min = r_bound[axis] - radius;

                let origin = ray.origin[axis];
                let inv_direction = ray.inv_direction[axis];

                let dist_left = (l_max - origin) * inv_direction;
                let dist_right = (r_min - origin) * inv_direction;

                let (near_child, far_child, near_clip_dist, far_clip_dist) =
                    if ray.direction[axis] >= 0.0 {
                        (node.prev, node.next, dist_left, dist_right)
                    } else {
                        (node.next, node.prev, dist_right, dist_left)
                    };

                let far_t_min = node_min_dist.max(far_clip_dist);
                if far_child != usize::MAX && far_t_min < node_max_dist {
                    stack[stack_ptr] = (far_child, far_t_min, node_max_dist);
                    stack_ptr += 1;
                }

                let near_t_max = node_max_dist.min(near_clip_dist);
                if near_child != usize::MAX && node_min_dist < near_t_max {
                    stack[stack_ptr] = (near_child, node_min_dist, near_t_max);
                    stack_ptr += 1;
                }
            } else {
                if let Some(hit_dist) =
                    intersect_voxel(&ray, &node.position, voxel_size, node_min_dist, max_dist)
                    && hit_dist < max_dist
                {
                    max_dist = hit_dist;
                    closest_hit = Some((hit_dist, node));
                }
            }
        }
        closest_hit
    }
}
