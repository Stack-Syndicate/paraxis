use std::collections::VecDeque;

use crate::maths::la::vector::Vector;

enum Side {
    Left,
    Right,
    None,
}

struct Depth(u32);

struct Parent(Option<usize>);

struct TraverseResult {
    stack: VecDeque<(usize, usize, Side)>,
    shortest_distance: f32,
    closest_node_id: usize,
}

pub struct KDNode {
    pub axis: usize,
    pub id: usize,
    pub left: Option<usize>,
    pub right: Option<usize>,
}

pub struct KDTree<T, const N: usize> {
    pub points: Vec<Vector<f32, N>>,
    pub data: Vec<T>,
    pub nodes: Vec<KDNode>,
    pub root: Option<usize>,
}
impl<T: Clone, const N: usize> KDTree<T, N> {
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
        target_position: Vector<f32, N>,
        depth: usize,
        start_id: usize,
    ) -> TraverseResult {
        let mut depth = depth;
        let mut stack = VecDeque::new();
        let mut id = start_id;
        let mut shortest_distance = f32::MAX;
        let mut closest_node_id = usize::MAX;
        // Traverse down to leaf
        loop {
            let node = &self.nodes[id];
            let axis = depth % N;
            let distance = target_position.distance_squared(&self.points[id]);
            // Check if current distance is shortest so far
            if distance < shortest_distance {
                shortest_distance = distance;
                closest_node_id = id;
            }
            // Decide if we should go left or right
            let (side, side_id);
            if target_position.inner[axis] < self.points[id].inner[axis] {
                side = Side::Left;
                side_id = node.left.unwrap_or(usize::MAX);
            } else {
                side = Side::Right;
                side_id = node.right.unwrap_or(usize::MAX);
            }
            // Add current node to the stack
            stack.push_front((id, depth, side));
            // Go left or right
            id = side_id;
            if id == usize::MAX {
                break;
            }
            // Book keeping
            depth += 1;
        }
        return TraverseResult {
            stack,
            shortest_distance,
            closest_node_id,
        };
    }
    pub fn nearest_neighbour_id(&self, target_position: Vector<f32, N>) -> usize {
        let mut traversal = self.traverse(
            target_position,
            0,
            self.root
                .expect("Root could not be found. Is the tree empty?"),
        );
        while let Some(item) = traversal.stack.pop_back() {
            let (id, depth, side) = item;
            let axis = depth % N;
            let node_position = &self.points[id];
            let distance_to_hyperplane =
                (target_position.inner[axis] - node_position.inner[axis]).abs();
            if distance_to_hyperplane.powi(2) >= traversal.shortest_distance {
                continue;
            }
            let node = &self.nodes[id];
            let other_child_id = match side {
                Side::Left => node.right,
                Side::Right => node.left,
                _ => unreachable!(),
            };
            if let Some(other_id) = other_child_id {
                let other_traversal = self.traverse(target_position, depth + 1, other_id);
                // Merge results
                traversal.stack = other_traversal
                    .stack
                    .into_iter()
                    .chain(traversal.stack)
                    .collect();
                // Update closest point if a nearer one was found
                if other_traversal.shortest_distance < traversal.shortest_distance {
                    traversal.shortest_distance = other_traversal.shortest_distance;
                    traversal.closest_node_id = other_traversal.closest_node_id;
                }
            }
        }
        traversal.closest_node_id
    }
    pub fn build(&mut self, data: Vec<(Vector<f32, N>, T)>) {
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
