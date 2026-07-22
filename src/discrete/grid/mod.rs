use std::collections::HashMap;

#[derive(Clone, Copy)]
pub struct Node<T, P> {
    position: P,
    inner: Option<T>,
}

pub trait Grid<T: Clone, P> {
    fn new(size: P) -> impl Grid<T, P>;
    fn insert(&mut self, data: T, position: &P);
    fn remove(&mut self, position: &P) -> Node<T, P>;
    fn get(&self, position: &P) -> Option<&Node<T, P>>;
    fn get_mut(&mut self, position: &P) -> Option<&mut Node<T, P>>;
    fn in_grid_bounds(&self, position: &P) -> bool;
}

pub struct Grid2D<T: Clone> {
    data: Vec<Node<T, [i32; 2]>>,
    size: [i32; 2],
}
impl<T: Clone> Grid<T, [i32; 2]> for Grid2D<T> {
    fn new(size: [i32; 2]) -> impl Grid<T, [i32; 2]> {
        let mut data = Vec::with_capacity((size[0] * size[1]) as usize);
        for y in 0..size[1] {
            for x in 0..size[0] {
                data.push(Node::<T, [i32; 2]> {
                    position: [x, y],
                    inner: None,
                });
            }
        }
        Self { data, size }
    }
    fn insert(&mut self, data: T, position: &[i32; 2]) {
        if !self.in_grid_bounds(position) {
            panic!("Insertion error: position out of range.")
        }
        let grid_id = position[0] + position[1] * self.size[0];
        self.data[grid_id as usize].inner = Some(data);
    }
    fn remove(&mut self, position: &[i32; 2]) -> Node<T, [i32; 2]> {
        if !self.in_grid_bounds(position) {
            panic!("Removal error: position out of range.")
        }
        let grid_id = position[0] + position[1] * self.size[0];
        let old_value = self.data[grid_id as usize].clone();
        self.data[grid_id as usize].inner = None;
        old_value
    }
    fn get(&self, position: &[i32; 2]) -> Option<&Node<T, [i32; 2]>> {
        if !self.in_grid_bounds(position) {
            panic!("Get error: position out of range.")
        }
        let grid_id = position[0] + position[1] * self.size[0];
        self.data.get(grid_id as usize)
    }
    fn get_mut(&mut self, position: &[i32; 2]) -> Option<&mut Node<T, [i32; 2]>> {
        if !self.in_grid_bounds(position) {
            panic!("Get error: position out of range.")
        }
        let grid_id = position[0] + position[1] * self.size[0];
        self.data.get_mut(grid_id as usize)
    }
    fn in_grid_bounds(&self, position: &[i32; 2]) -> bool {
        0 <= position[0]
            && position[0] < self.size[0]
            && 0 <= position[1]
            && position[1] < self.size[1]
    }
}

pub struct Grid3D<T: Clone> {
    data: Vec<Node<T, [i32; 3]>>,
    size: [i32; 3],
}
impl<T: Clone> Grid<T, [i32; 3]> for Grid3D<T> {
    fn new(size: [i32; 3]) -> impl Grid<T, [i32; 3]> {
        let mut data = Vec::with_capacity((size[0] * size[1] * size[2]) as usize);
        for z in 0..size[2] {
            for y in 0..size[1] {
                for x in 0..size[0] {
                    data.push(Node::<T, [i32; 3]> {
                        position: [x, y, z],
                        inner: None,
                    });
                }
            }
        }
        Self { data, size }
    }
    fn insert(&mut self, data: T, position: &[i32; 3]) {
        if !self.in_grid_bounds(position) {
            panic!("Insertion error: position out of range.")
        }
        let grid_id = position[0] + position[1] * self.size[0] + position[2] * self.size[1];
        self.data[grid_id as usize].inner = Some(data);
    }
    fn remove(&mut self, position: &[i32; 3]) -> Node<T, [i32; 3]> {
        if !self.in_grid_bounds(position) {
            panic!("Removal error: position out of range.")
        }
        let grid_id = position[0] + position[1] * self.size[0] + position[2] * self.size[1];
        let old_value = self.data[grid_id as usize].clone();
        self.data[grid_id as usize].inner = None;
        old_value
    }
    fn get(&self, position: &[i32; 3]) -> Option<&Node<T, [i32; 3]>> {
        if !self.in_grid_bounds(position) {
            panic!("Get error: position out of range.")
        }
        let grid_id = position[0] + position[1] * self.size[0] + position[2] * self.size[1];
        self.data.get(grid_id as usize)
    }
    fn get_mut(&mut self, position: &[i32; 3]) -> Option<&mut Node<T, [i32; 3]>> {
        if !self.in_grid_bounds(position) {
            panic!("Get error: position out of range.")
        }
        let grid_id = position[0] + position[1] * self.size[0] + position[2] * self.size[1];
        self.data.get_mut(grid_id as usize)
    }
    fn in_grid_bounds(&self, position: &[i32; 3]) -> bool {
        0 <= position[0]
            && position[0] < self.size[0]
            && 0 <= position[1]
            && position[1] < self.size[1]
            && 0 <= position[2]
            && position[2] < self.size[2]
    }
}

pub struct SparseGrid<T, const N: usize> {
    data: HashMap<[i32; N], Node<T, [i32; N]>>,
    size: [i32; N],
}
impl<T: Clone, const N: usize> Grid<T, [i32; N]> for SparseGrid<T, N> {
    fn new(size: [i32; N]) -> impl Grid<T, [i32; N]> {
        let data = HashMap::new();
        Self { data, size }
    }
    fn insert(&mut self, data: T, position: &[i32; N]) {
        if !self.in_grid_bounds(position) {
            panic!("Insertion error: position out of range.")
        }
        self.data
            .entry(*position)
            .or_insert_with(|| Node {
                position: *position,
                inner: None,
            })
            .inner = Some(data)
    }
    fn remove(&mut self, position: &[i32; N]) -> Node<T, [i32; N]> {
        if !self.in_grid_bounds(position) {
            panic!("Removal error: position out of range.")
        }
        self.data.remove(position).unwrap()
    }
    fn get(&self, position: &[i32; N]) -> Option<&Node<T, [i32; N]>> {
        if !self.in_grid_bounds(position) {
            panic!("Get error: position out of range.")
        }
        self.data.get(position)
    }
    fn get_mut(&mut self, position: &[i32; N]) -> Option<&mut Node<T, [i32; N]>> {
        if !self.in_grid_bounds(position) {
            panic!("Get error: position out of range.")
        }
        self.data.get_mut(position)
    }
    fn in_grid_bounds(&self, position: &[i32; N]) -> bool {
        0 <= position[0]
            && position[0] < self.size[0]
            && 0 <= position[1]
            && position[1] < self.size[1]
    }
}
