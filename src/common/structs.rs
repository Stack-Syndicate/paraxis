use std::sync::{RwLock, RwLockReadGuard, RwLockWriteGuard};

#[derive(Debug, Clone)]
pub struct NodeData<D> {
    pub inner: Option<D>,
}

#[derive(Debug)]
pub struct Node<P, D> {
    pub position: P,
    pub bounds: Option<(P, P)>,
    pub next: usize,
    pub prev: usize,
    inner: RwLock<NodeData<D>>,
}
impl<P: Clone, D: Clone> Clone for Node<P, D> {
    fn clone(&self) -> Self {
        let inner = self.inner.read().unwrap().inner.clone();
        Self {
            position: self.position.clone(),
            bounds: self.bounds.clone(),
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
            bounds: None,
            inner: RwLock::new(NodeData { inner: data }),
            next: usize::MAX,
            prev: usize::MAX,
        }
    }
    pub fn new_bounded(position: P, bounds: (P, P), data: Option<D>) -> Self {
        Self {
            position,
            bounds: Some(bounds),
            inner: RwLock::new(NodeData { inner: data }),
            next: usize::MAX,
            prev: usize::MAX,
        }
    }
    pub fn write(&self) -> RwLockWriteGuard<'_, NodeData<D>> {
        self.inner.write().unwrap()
    }
    pub fn read(&self) -> RwLockReadGuard<'_, NodeData<D>> {
        self.inner.read().unwrap()
    }
}

pub struct Ray<const N: usize> {
    pub origin: [f32; N],
    pub direction: [f32; N],
    pub inv_direction: [f32; N],
}
impl<const N: usize> Ray<N> {
    pub fn new(origin: [f32; N], direction: [f32; N]) -> Self {
        let mut inv_direction = [0.0; N];
        for i in 0..N {
            inv_direction[i] = 1.0 / direction[i];
        }
        Self {
            origin,
            direction,
            inv_direction,
        }
    }
}
