use std::{collections::VecDeque, fmt::Display, simd::SimdElement};

use crate::maths::la::vector::Vector;

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
impl<P: SimdElement + PartialOrd + Display, T: Clone, const N: usize> KDTree<P, T, N> {
    pub fn new_empty() -> Self {
        Self {
            points: Vec::new(),
            data: Vec::new(),
            nodes: Vec::new(),
            root: None,
        }
    }
    pub fn build(&mut self, data: Vec<(Vector<P, N>, T)>) {
        let mut data = data.clone();

        let mut task_queue = VecDeque::new();
        task_queue.push_front((0..data.len(), Depth(0), Parent(None), Side::None));
        while let Some(task) = task_queue.pop_back() {
            let (range, depth, parent, side) = task;
            if range.is_empty() {
                continue;
            }
            let axis = depth.0 as usize % N;
            let mid_rel = range.len() / 2;
            let mid_abs = range.start + mid_rel;
            data[range.clone()].select_nth_unstable_by(mid_rel, |a, b| {
                a.0.inner[axis]
                    .partial_cmp(&b.0.inner[axis])
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
            let node_id = self.nodes.len();
            let (mid_point, mid_element) = &data[mid_abs];
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
            task_queue.push_front((
                range.start..mid_abs,
                Depth(depth.0 + 1),
                Parent(Some(node_id)),
                Side::Left,
            ));
            task_queue.push_front((
                (mid_abs + 1)..range.end,
                Depth(depth.0 + 1),
                Parent(Some(node_id)),
                Side::Right,
            ));
            self.points.push(*mid_point);
            self.data.push(mid_element.clone());
        }
    }
    pub fn is_valid(&self, node_idx: Option<usize>, depth: usize) -> bool {
        let idx = match node_idx {
            Some(i) => i,
            None => return true,
        };
        let node = &self.nodes[idx];
        let axis = depth % N;
        let parent_val = self.points[node.id].inner[axis];

        if let Some(left_idx) = node.left {
            let left_point = self.points[self.nodes[left_idx].id];
            if left_point.inner[axis] > parent_val {
                return false;
            }
            if !self.is_valid(Some(left_idx), depth + 1) {
                return false;
            }
        }

        if let Some(right_idx) = node.right {
            let right_point = self.points[self.nodes[right_idx].id];
            if right_point.inner[axis] < parent_val {
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
            (Vector::from([5, 5, 5]), "A"),
            (Vector::from([2, 2, 2]), "B"),
            (Vector::from([8, 8, 8]), "C"),
            (Vector::from([1, 9, 1]), "D"),
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
