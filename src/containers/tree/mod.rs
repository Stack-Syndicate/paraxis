use kdtree::KdTree;
use num_traits::Float;
use rstar::{Point, RTree, RTreeNum, RTreeObject, primitives::GeomWithData};

pub trait Tree<P, D> {
    fn new() -> Self;
    fn add(&mut self, position: P, data: D);
}

pub struct RSTree<T, D, const N: usize>
where
    T: RTreeNum,
    [T; N]: Point<Scalar = T>,
    GeomWithData<[T; N], D>: RTreeObject,
{
    inner: RTree<GeomWithData<[T; N], D>>,
}
impl<T, D, const N: usize> Tree<[T; N], D> for RSTree<T, D, N>
where
    T: RTreeNum,
    [T; N]: Point<Scalar = T>,
    GeomWithData<[T; N], D>: RTreeObject,
{
    fn new() -> Self {
        Self {
            inner: RTree::new(),
        }
    }
    fn add(&mut self, position: [T; N], data: D) {
        self.inner.insert(GeomWithData::new(position, data));
    }
}

pub struct KDTree<T: Float, D, const N: usize> {
    inner: KdTree<T, D, [T; N]>,
}
impl<T: Float, D, const N: usize> Tree<[T; N], D> for KDTree<T, D, N> {
    fn new() -> Self {
        let kdtree = KdTree::new(N);
        Self { inner: kdtree }
    }
    fn add(&mut self, position: [T; N], data: D) {
        self.inner.add(position, data).unwrap();
    }
}
