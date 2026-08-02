use std::sync::{RwLock, RwLockWriteGuard};

#[derive(Debug, Clone)]
pub struct NodeMut<D> {
    pub inner: Option<D>,
    pub next: Option<usize>,
    pub prev: Option<usize>,
}

#[derive(Debug)]
pub struct Node<P, D> {
    pub position: P,
    inner: RwLock<NodeMut<D>>,
}
impl<P: Clone, D: Clone> Clone for Node<P, D> {
    fn clone(&self) -> Self {
        let inner_lock = self.inner.read().unwrap();
        let position = self.position.clone();
        let inner = inner_lock.inner.clone();
        let next = inner_lock.next;
        let prev = inner_lock.prev;
        Self {
            position,
            inner: RwLock::new(NodeMut { inner, next, prev }),
        }
    }
}
impl<P, D> Node<P, D> {
    pub fn new(position: P, data: Option<D>) -> Self {
        Self {
            position,
            inner: RwLock::new(NodeMut {
                inner: data,
                next: None,
                prev: None,
            }),
        }
    }
    pub fn write(&self) -> RwLockWriteGuard<'_, NodeMut<D>> {
        self.inner.write().unwrap()
    }
    pub fn read(&self) -> RwLockWriteGuard<'_, NodeMut<D>> {
        self.inner.write().unwrap()
    }
}
