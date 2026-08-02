use std::sync::{RwLock, RwLockReadGuard, RwLockWriteGuard};

#[derive(Debug, Clone)]
pub struct NodeData<D> {
    pub inner: Option<D>,
}

#[derive(Debug)]
pub struct Node<P, D> {
    pub position: P,
    pub next: Option<usize>,
    pub prev: Option<usize>,
    inner: RwLock<NodeData<D>>,
}
impl<P: Clone, D: Clone> Clone for Node<P, D> {
    fn clone(&self) -> Self {
        let position = self.position.clone();
        let inner = self.inner.read().unwrap().inner.clone();
        Self {
            position,
            inner: RwLock::new(NodeData { inner }),
            prev: self.prev,
            next: self.next,
        }
    }
}
impl<P, D> Node<P, D> {
    pub fn new(position: P, data: Option<D>) -> Self {
        Self {
            position,
            inner: RwLock::new(NodeData { inner: data }),
            next: None,
            prev: None,
        }
    }
    pub fn write(&self) -> RwLockWriteGuard<'_, NodeData<D>> {
        self.inner.write().unwrap()
    }
    pub fn read(&self) -> RwLockReadGuard<'_, NodeData<D>> {
        self.inner.read().unwrap()
    }
}
