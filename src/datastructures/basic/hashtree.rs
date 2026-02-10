use std::collections::{VecDeque, binary_heap::Iter};

use crate::datastructures::basic::hashvec::HashVec;

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
        let invalid =
            (self.root.is_none() && parent.is_some()) || (self.root.is_some() && parent.is_none());
        if invalid {
            return None;
        };
        let mut new_id = 0;
        while self.nodes.has(&new_id) {
            new_id += 1;
        }
        if self.root.is_none() && parent.is_none() {
            self.root = Some(new_id);
            self.nodes.insert(new_id, Node::new(data, None));
        } else {
            let parent_id = parent.unwrap();
            self.nodes.insert(new_id, Node::new(data, parent));
            if let Some(parent) = self.nodes.get_mut(&parent_id) {
                parent.children.push(new_id);
                parent.is_leaf = false;
            }
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
    pub fn bfs(&self) {}
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
