use std::collections::VecDeque;

use crate::datastructures::basic::hashvec::*;

pub type NodeID = usize;

#[derive(Clone)]
pub struct Node<D: Clone> {
    data: D,
    parent: Option<NodeID>,
    children: Vec<NodeID>,
    is_leaf: bool,
}
impl<D: Clone> Node<D> {
    pub fn new(data: D, parent: Option<usize>) -> Self {
        Self {
            data,
            parent,
            children: Vec::new(),
            is_leaf: true,
        }
    }
}
pub struct HashTree<D: Clone> {
    nodes: HashVec<NodeID, Node<D>>,
    root: Option<NodeID>,
}
impl<D: Clone> HashTree<D> {
    pub fn new() -> Self {
        Self {
            root: None,
            nodes: HashVec::new(),
        }
    }
    pub fn add(&mut self, data: D, parent: Option<NodeID>) -> Option<NodeID> {
        match (self.root, parent) {
            (None, Some(_)) => return None,
            (Some(_), None) => return None,
            (Some(_), Some(p)) if !self.nodes.has(&p) => return None,
            _ => {}
        }
        let new_id = self.nodes.len();
        let node = Node::new(data, parent);
        self.nodes.insert(new_id, node);
        if let Some(parent_id) = parent {
            if let Some(parent_node) = self.nodes.get_mut(&parent_id) {
                parent_node.children.push(new_id);
            }
        } else {
            self.root = Some(new_id);
        }
        return Some(new_id);
    }
    pub fn dfs<'a>(&self) -> DFSIter<'_, D> {
        let mut stack = Vec::new();
        if let Some(root_id) = self.root {
            stack.push(root_id);
        }
        DFSIter { tree: self, stack }
    }
    pub fn bfs(&self) -> BFSIter<'_, D> {
        let mut queue = VecDeque::new();
        if let Some(root_id) = self.root {
            queue.push_front(root_id);
        }
        BFSIter { tree: self, queue }
    }
}

pub struct DFSIter<'a, D: Clone> {
    tree: &'a HashTree<D>,
    stack: Vec<NodeID>,
}
impl<'a, D: Clone> Iterator for DFSIter<'a, D> {
    type Item = (NodeID, &'a Node<D>);
    fn next(&mut self) -> Option<Self::Item> {
        let node_id = self.stack.pop()?;
        let node = self.tree.nodes.get(&node_id)?;
        for &child_id in node.children.iter().rev() {
            self.stack.push(child_id);
        }
        Some((node_id, node))
    }
}

pub struct BFSIter<'a, D: Clone> {
    tree: &'a HashTree<D>,
    queue: VecDeque<NodeID>,
}
impl<'a, D: Clone> Iterator for BFSIter<'a, D> {
    type Item = (NodeID, &'a Node<D>);
    fn next(&mut self) -> Option<Self::Item> {
        let node_id = self.queue.pop_front()?;
        let node = self.tree.nodes.get(&node_id)?;
        for &child_id in node.children.iter() {
            self.queue.push_back(child_id);
        }
        Some((node_id, node))
    }
}

#[cfg(test)]
mod hashtree_tests {
    use super::*;

    #[test]
    fn test_root_enforcement() {
        let mut tree = HashTree::new();

        // 1. Cannot add a child if no root exists
        assert!(
            tree.add("child", Some(0)).is_none(),
            "Should not allow child without root"
        );

        // 2. Add a root
        let root_id = tree.add("root", None).expect("Should allow adding a root");
        assert_eq!(tree.root, Some(root_id));

        // 3. Cannot add a second root
        assert!(
            tree.add("duplicate_root", None).is_none(),
            "Should not allow a second root"
        );
    }

    #[test]
    fn test_tree_links() {
        let mut tree = HashTree::new();
        let root = tree.add("root", None).unwrap();
        let c1 = tree.add("child1", Some(root)).unwrap();
        let c2 = tree.add("child2", Some(root)).unwrap();

        // Verify parent knows about children
        let root_node = tree.nodes.get(&root).unwrap();
        assert_eq!(root_node.children.len(), 2);
        assert!(root_node.children.contains(&c1));
        assert!(root_node.children.contains(&c2));

        // Verify children know about parent
        assert_eq!(tree.nodes.get(&c1).unwrap().parent, Some(root));
        assert_eq!(tree.nodes.get(&c2).unwrap().parent, Some(root));
    }

    #[test]
    fn test_dfs_traversal_order() {
        let mut tree = HashTree::new();
        // Structure:
        //      0
        //     / \
        //    1   2
        //   /
        //  3
        let n0 = tree.add("0", None).unwrap();
        let n1 = tree.add("1", Some(n0)).unwrap();
        let n2 = tree.add("2", Some(n0)).unwrap();
        let n3 = tree.add("3", Some(n1)).unwrap();

        // DFS (Stack-based with rev() children)
        // Order: 0 -> 1 -> 3 -> 2
        let visited: Vec<NodeID> = tree.dfs().map(|(id, _)| id).collect();
        assert_eq!(visited, vec![n0, n1, n3, n2]);
    }

    #[test]
    fn test_bfs_traversal_order() {
        let mut tree = HashTree::new();
        // Structure:
        //      A
        //     / \
        //    B   C
        //   / \
        //  D   E
        let na = tree.add("A", None).unwrap();
        let nb = tree.add("B", Some(na)).unwrap();
        let nc = tree.add("C", Some(na)).unwrap();
        let nd = tree.add("D", Some(nb)).unwrap();
        let ne = tree.add("E", Some(nb)).unwrap();

        // BFS (Queue-based)
        // Order: A -> B -> C -> D -> E
        let visited: Vec<NodeID> = tree.bfs().map(|(id, _)| id).collect();
        assert_eq!(visited, vec![na, nb, nc, nd, ne]);
    }

    #[test]
    fn test_invalid_parent_id() {
        let mut tree = HashTree::new();
        tree.add("root", None).unwrap();

        // Attempting to add a child to a non-existent parent ID
        // Note: Your current code doesn't strictly return None for this,
        // it just fails the get_mut check. This test documents that behavior.
        let result = tree.add("orphan", Some(9999));

        // Check if the orphan is unreachable via DFS
        let reachable = tree.dfs().any(|(id, _)| result == Some(id));
        assert!(
            !reachable,
            "Node with invalid parent should not be in the tree hierarchy"
        );
    }
}
