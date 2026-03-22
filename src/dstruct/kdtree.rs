use crate::maths::la::vector::Vector;
use num_traits::{float::TotalOrder, real::Real};
use std::{
    fmt::Debug,
    ops::{Mul, Sub},
    simd::{num::SimdFloat, Simd, SimdElement},
};

#[derive(Clone)]
enum Side {
    Left,
    Right,
    None,
}

struct Depth(u32);

struct Parent(Option<usize>);

pub struct KDNode {
    pub axis: usize,
    pub id: usize,
    pub left: Option<usize>,
    pub right: Option<usize>,
}

pub struct KDTree<P: SimdElement, T, const N: usize> {
    pub points: Vec<Vector<P, N>>,
    pub data: Vec<T>,
    pub nodes: Vec<KDNode>,
    pub root: Option<usize>,
}
impl<P, T, const N: usize> KDTree<P, T, N>
where
    Simd<P, N>: Mul<Output = Simd<P, N>> + Sub<Output = Simd<P, N>> + SimdFloat<Scalar = P>,
    T: Default,
    P: SimdElement + Real + TotalOrder + Debug + Default,
{
    pub fn new_empty() -> Self {
        Self {
            points: Vec::new(),
            data: Vec::new(),
            nodes: Vec::new(),
            root: None,
        }
    }
    fn traverse(
        &self,
        target_position: Vector<P, N>,
        depth: usize,
        start_id: usize,
        stack: &mut Vec<(usize, usize, Side)>,
    ) {
        let mut depth = depth;
        let mut id = start_id;
        loop {
            let node = &self.nodes[id];
            let axis = depth % N;
            let (side, side_id);
            if target_position[axis] < self.points[id].inner[axis] {
                side = Side::Left;
                side_id = node.left.unwrap_or(usize::MAX);
            } else {
                side = Side::Right;
                side_id = node.right.unwrap_or(usize::MAX);
            }
            stack.push((id, depth, side));
            id = side_id;
            if id == usize::MAX {
                break;
            }
            depth += 1;
        }
    }
    fn nearest_neighbour_id(
        &self,
        target_position: Vector<P, N>,
        k: usize,
        max_radius: Option<P>,
    ) -> Vec<(usize, P)> {
        let mut stack = Vec::new();
        let mut other_stack = Vec::new();
        self.traverse(
            target_position,
            0,
            self.root
                .expect("Root could not be found. Is the tree empty?"),
            &mut stack,
        );
        let mut best: Vec<(usize, P)> = Vec::new();
        let insert = |best: &mut Vec<(usize, P)>, id: usize, dist: P| {
            if let Some(r) = max_radius {
                let r2 = r * r;
                if dist > r2 {
                    return;
                }
            }
            if best.len() < k {
                best.push((id, dist));
            } else {
                let (worst_idx, worst_dist) = best
                    .iter()
                    .enumerate()
                    .max_by(|a, b| a.1 .1.partial_cmp(&b.1 .1).unwrap())
                    .unwrap();
                if dist < worst_dist.1 {
                    best[worst_idx] = (id, dist);
                }
            }
            best.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
        };
        for (id, _, _) in stack.iter() {
            let node_position = &self.points[*id];
            let dist = target_position.distance_squared(node_position);
            insert(&mut best, *id, dist);
        }
        while let Some(item) = stack.pop() {
            let (id, depth, side) = item;
            let axis = depth % N;
            let node_position = &self.points[id];
            let distance_to_hyperplane = (target_position[axis] - node_position.inner[axis]).abs();
            let worst = best.last().map(|(_, d)| *d).unwrap_or(P::max_value());
            let radius_limit = max_radius.map(|r| r * r).unwrap_or(P::max_value());
            let prune_threshold = worst.min(radius_limit);
            if distance_to_hyperplane.powi(2) >= prune_threshold {
                continue;
            }
            let node = &self.nodes[id];
            let other_child_id = match side {
                Side::Left => node.right,
                Side::Right => node.left,
                _ => unreachable!(),
            };
            if let Some(other_id) = other_child_id {
                other_stack.clear();
                self.traverse(target_position, depth + 1, other_id, &mut other_stack);
                for (nid, _, _) in &other_stack {
                    let node_position = &self.points[*nid];
                    let dist = target_position.distance_squared(node_position);
                    insert(&mut best, *nid, dist);
                }
                stack.append(&mut other_stack);
            }
        }
        best
    }
    pub fn nearest_neighbour(
        &self,
        target_position: Vector<P, N>,
        k: usize,
    ) -> Vec<(Vector<P, N>, &T)> {
        let ids = self.nearest_neighbour_id(target_position, k, None);
        ids.iter()
            .map(|id| (self.points[id.0], &self.data[id.0]))
            .collect()
    }
    pub fn build(&mut self, mut data: Vec<(Vector<P, N>, T)>) {
        let mut task_queue = Vec::with_capacity(data.len());
        task_queue.push((0..data.len(), Depth(0), Parent(None), Side::None));
        while let Some(task) = task_queue.pop() {
            let (range, depth, parent, side) = task;
            if range.is_empty() {
                continue;
            }
            let axis = depth.0 as usize % N;
            let mid_rel = range.len() / 2;
            let mid_abs = range.start + mid_rel;
            data[range.clone()]
                .select_nth_unstable_by(mid_rel, |a, b| a.0[axis].total_cmp(&b.0[axis]));
            let node_id = self.nodes.len();
            let (mid_point, _) = &data[mid_abs];
            self.nodes.push(KDNode {
                axis,
                id: node_id,
                left: None,
                right: None,
            });
            if let Parent(Some(parent_id)) = parent {
                match side {
                    Side::Left => self.nodes[parent_id].left = Some(node_id),
                    Side::Right => self.nodes[parent_id].right = Some(node_id),
                    Side::None => self.root = Some(node_id),
                }
            } else {
                self.root = Some(node_id)
            }
            task_queue.push((
                range.start..mid_abs,
                Depth(depth.0 + 1),
                Parent(Some(node_id)),
                Side::Left,
            ));
            task_queue.push((
                (mid_abs + 1)..range.end,
                Depth(depth.0 + 1),
                Parent(Some(node_id)),
                Side::Right,
            ));
            self.points.push(*mid_point);
            let (_, mid_element) = std::mem::take(&mut data[mid_abs]);
            self.data.push(mid_element);
        }
    }
    pub fn is_valid(&self, node_idx: Option<usize>, depth: usize) -> bool {
        let idx = match node_idx {
            Some(i) => i,
            None => return true,
        };
        let node = &self.nodes[idx];
        let axis = depth % N;
        let parent_val = self.points[node.id][axis];
        if let Some(left_idx) = node.left {
            let left_point = self.points[self.nodes[left_idx].id];
            if left_point[axis] > parent_val {
                return false;
            }
            if !self.is_valid(Some(left_idx), depth + 1) {
                return false;
            }
        }
        if let Some(right_idx) = node.right {
            let right_point = self.points[self.nodes[right_idx].id];
            if right_point[axis] < parent_val {
                return false;
            }
            if !self.is_valid(Some(right_idx), depth + 1) {
                return false;
            }
        }
        true
    }
    pub fn debug_print(&self, node_idx: Option<usize>, depth: usize) {
        if let Some(idx) = node_idx {
            let node = &self.nodes[idx];
            println!(
                "{:indent$}Node {}: Point {}, Axis {}",
                "",
                idx,
                self.points[node.id],
                node.axis,
                indent = depth * 2
            );
            self.debug_print(node.left, depth + 1);
            self.debug_print(node.right, depth + 1);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_kdtree_invariants() {
        let points = vec![
            (Vector::from([5., 5., 5.]), "A"),
            (Vector::from([2., 2., 2.]), "B"),
            (Vector::from([8., 8., 8.]), "C"),
            (Vector::from([1., 9., 1.]), "D"),
        ];
        let mut tree = KDTree {
            points: Vec::new(),
            data: Vec::new(),
            nodes: Vec::new(),
            root: None,
        };
        tree.build(points);
        assert_eq!(tree.nodes.len(), 4);
        assert!(tree.root.is_some());
        assert!(
            tree.is_valid(tree.root, 0),
            "Tree invariants were violated!"
        );
        tree.debug_print(Some(1), 0);
    }
}
