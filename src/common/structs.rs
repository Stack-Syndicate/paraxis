#[derive(Clone, Copy)]
pub struct Node<T, P> {
    pub position: P,
    pub inner: Option<T>,
}
