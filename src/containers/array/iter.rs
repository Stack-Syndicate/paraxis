use crate::containers::array::Array;

impl<D> IntoIterator for Array<D> {
    type Item = D;
    type IntoIter = std::vec::IntoIter<D>;
    fn into_iter(self) -> Self::IntoIter {
        self.data.into_iter()
    }
}
impl<'a, D> IntoIterator for &'a Array<D> {
    type Item = &'a D;
    type IntoIter = std::slice::Iter<'a, D>;

    fn into_iter(self) -> Self::IntoIter {
        self.data.iter()
    }
}
