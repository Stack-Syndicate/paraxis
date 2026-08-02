#[derive(Debug, Clone)]
pub struct Node<P, D> {
    pub position: P,
    pub inner: Option<D>,
    pub next: Option<usize>,
    pub prev: Option<usize>,
}
