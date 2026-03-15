use std::{hash::Hash, ops::Add};

use ahash::AHashMap;
use glam::{IVec2, IVec3, IVec4};

pub trait Adjacency {
    fn offsets() -> &'static [Self]
    where
        Self: Sized;
}
impl Adjacency for IVec2 {
    fn offsets() -> &'static [Self] {
        &[IVec2::X, IVec2::NEG_X, IVec2::Y, IVec2::NEG_Y]
    }
}
impl Adjacency for IVec3 {
    fn offsets() -> &'static [Self] {
        &[
            IVec3::X,
            IVec3::NEG_X,
            IVec3::Y,
            IVec3::NEG_Y,
            IVec3::Z,
            IVec3::NEG_Z,
        ]
    }
}
impl Adjacency for IVec4 {
    fn offsets() -> &'static [Self] {
        &[
            IVec4::X,
            IVec4::NEG_X,
            IVec4::Y,
            IVec4::NEG_Y,
            IVec4::Z,
            IVec4::NEG_Z,
            IVec4::W,
            IVec4::NEG_W,
        ]
    }
}

pub trait Grid<P: Eq + Hash, E> {
    fn new() -> Self;
    fn inner(&self) -> &AHashMap<P, E>;
    fn inner_mut(&mut self) -> &mut AHashMap<P, E>;
    fn insert(&mut self, position: P, element: E) {
        self.inner_mut().insert(position, element);
    }
    fn remove(&mut self, position: P) {
        self.inner_mut().remove(&position);
    }
}
macro_rules! impl_grid {
    ($name:ident, $pos:ty) => {
        pub struct $name<E> {
            inner: AHashMap<$pos, E>,
        }
        impl<E> Grid<$pos, E> for $name<E> {
            fn new() -> Self {
                Self {
                    inner: AHashMap::new(),
                }
            }
            fn inner(&self) -> &AHashMap<$pos, E> {
                &self.inner
            }
            fn inner_mut(&mut self) -> &mut AHashMap<$pos, E> {
                &mut self.inner
            }
        }
    };
}
impl_grid!(Grid2, IVec2);
impl_grid!(Grid3, IVec3);
impl_grid!(Grid4, IVec4);

pub trait Neighbour<P, E> {
    fn neighbours(&self, pos: P) -> Vec<&E>;

    fn neighbours_mut(&mut self, pos: P) -> Vec<&mut E>;
}
impl<'a, T, P, E> Neighbour<P, E> for T
where
    T: Grid<P, E>,
    P: Adjacency + Add<Output = P> + Eq + Hash + Copy + 'static,
{
    fn neighbours(&self, pos: P) -> Vec<&E> {
        P::offsets()
            .iter()
            .filter_map(|&offset| self.inner().get(&(pos + offset)))
            .collect()
    }

    fn neighbours_mut(&mut self, pos: P) -> Vec<&mut E> {
        let map = self.inner_mut();
        P::offsets()
            .iter()
            .filter_map(|&offset| {
                let target = pos + offset;
                unsafe {
                    let ptr = map.get_mut(&target)? as *mut E;
                    Some(&mut *ptr)
                }
            })
            .collect()
    }
}

#[test]
fn test() {
    let mut g = Grid2::new();
    g.insert(IVec2::new(-1, 0), "w");
    g.insert(IVec2::new(0, -1), "s");
    g.insert(IVec2::new(1, 0), "d");
    g.insert(IVec2::new(0, -2), "g");
    println!("{:?}", g.neighbours(IVec2::new(0, 0)));
}
