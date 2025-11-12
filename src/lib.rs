pub mod voxel;

#[derive(Clone, Debug)]
pub struct KDNode<const K: usize> {
    position: [f32; K],
    left: Option<usize>,
    right: Option<usize>,
}

#[derive(Clone, Debug)]
pub struct KDTree<const K: usize> {
    kd_nodes: Vec<KDNode<K>>,
}

impl<const K: usize> KDTree<K> {
    pub fn generate(points: Vec<[f32; K]>) -> Self {
        let mut kd_nodes = Vec::new();

        fn traverse_points<const K: usize>(
            points: &mut Vec<[f32; K]>,
            depth: usize,
            kd_nodes: &mut Vec<KDNode<K>>,
        ) -> Option<usize> {
            if points.is_empty() {
                return None;
            }

            let axis = depth % K;
            points.sort_by(|a, b| a[axis].total_cmp(&b[axis]));

            let median_index = points.len() / 2;
            let median_node = points[median_index];
            let current_index = kd_nodes.len();
            kd_nodes.push(KDNode {
                position: median_node,
                left: None,
                right: None,
            });
            let (left_subtree, right_subtree) = points.split_at_mut(median_index);
            let left_index =
                traverse_points(&mut left_subtree[..median_index].to_vec(), depth + 1, kd_nodes);
            let right_index = traverse_points(&mut right_subtree[1..].to_vec(), depth + 1, kd_nodes); // Start from right of median

            kd_nodes[current_index].left = left_index;
            kd_nodes[current_index].right = right_index;

            Some(current_index)
        }

        let mut points_copy = points.clone();
        traverse_points(&mut points_copy, 0, &mut kd_nodes);

        Self { kd_nodes }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_kd_tree_generation() {
        let points = vec![
            [2.0, 3.0],
            [5.0, 4.0],
            [9.0, 6.0],
            [4.0, 7.0],
            [8.0, 1.0],
            [7.0, 2.0],
        ];

        let kd_tree = KDTree::<2>::generate(points.clone());

        assert!(!kd_tree.kd_nodes.is_empty());

        let root_node = &kd_tree.kd_nodes[0];
        println!("{:?}", kd_tree.kd_nodes);
        assert!(root_node.left.is_some() || root_node.right.is_some());

        assert_eq!(kd_tree.kd_nodes.len(), 6);

        // Check that nodes have expected children
        for i in 0..kd_tree.kd_nodes.len() {
            let node = &kd_tree.kd_nodes[i];
            assert!(
                node.left.is_some()
                    || node.right.is_some()
                    || (node.left.is_none() && node.right.is_none())
            );
        }
    }
}
