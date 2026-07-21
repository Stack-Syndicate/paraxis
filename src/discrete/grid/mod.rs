use nalgebra::SVector;

#[derive(Clone, Copy)]
pub struct Cell<T, P> {
    position: P,
    inner: Option<T>,
}

pub trait Grid<T: Clone, P> {
    fn new(size: P) -> impl Grid<T, P>;
    fn insert(&mut self, data: T, position: P);
    fn remove(&mut self, position: P) -> Cell<T, P>;
    fn get_ref(&self, position: P) -> &Cell<T, P>;
    fn get_mut(&mut self, position: P) -> &mut Cell<T, P>;
    fn get_cube_ref(&self, corner_1: P, corner_2: P) -> Vec<&Cell<T, P>>;
    fn get_cube_mut(&mut self, corner_1: P, corner_2: P) -> Vec<&mut Cell<T, P>>;
    fn in_grid_bounds(&self, position: &P) -> bool;
}

pub struct Grid2D<T: Clone> {
    data: Vec<Cell<T, SVector<i32, 2>>>,
    size: SVector<i32, 2>,
}
impl<T: Clone> Grid<T, SVector<i32, 2>> for Grid2D<T> {
    fn new(size: SVector<i32, 2>) -> impl Grid<T, SVector<i32, 2>> {
        let mut data = Vec::with_capacity((size[0] * size[1]) as usize);
        for y in 0..size[1] {
            for x in 0..size[0] {
                data.push(Cell::<T, SVector<i32, 2>> {
                    position: SVector::from([x, y]),
                    inner: None,
                });
            }
        }
        Self { data, size }
    }
    fn insert(&mut self, data: T, position: SVector<i32, 2>) {
        if !self.in_grid_bounds(&position) {
            panic!("Insertion error: position out of range.")
        }
        let grid_id = position[0] + position[1] * self.size[0];
        self.data[grid_id as usize].inner = Some(data);
    }
    fn remove(&mut self, position: SVector<i32, 2>) -> Cell<T, SVector<i32, 2>> {
        if !self.in_grid_bounds(&position) {
            panic!("Removal error: position out of range.")
        }
        let grid_id = position[0] + position[1] * self.size[0];
        let old_value = self.data[grid_id as usize].clone();
        self.data[grid_id as usize].inner = None;
        old_value
    }
    fn get_ref(&self, position: SVector<i32, 2>) -> &Cell<T, SVector<i32, 2>> {
        if !self.in_grid_bounds(&position) {
            panic!("Get error: position out of range.")
        }
        let grid_id = position[0] + position[1] * self.size[0];
        &self.data[grid_id as usize]
    }
    fn get_mut(&mut self, position: SVector<i32, 2>) -> &mut Cell<T, SVector<i32, 2>> {
        if !self.in_grid_bounds(&position) {
            panic!("Get error: position out of range.")
        }
        let grid_id = position[0] + position[1] * self.size[0];
        &mut self.data[grid_id as usize]
    }
    fn get_cube_ref(
        &self,
        corner_1: SVector<i32, 2>,
        corner_2: SVector<i32, 2>,
    ) -> Vec<&Cell<T, SVector<i32, 2>>> {
        if !self.in_grid_bounds(&corner_1) || !self.in_grid_bounds(&corner_2) {
            panic!("Get Cube Mut error: corners out of range.")
        }
        let mut results = Vec::new();
        results.extend((corner_1[1]..corner_2[1]).flat_map(|y| {
            let y_offset = (y * self.size[0]) as usize;
            (corner_1[0]..corner_2[0]).map(move |x| &self.data[y_offset + x as usize])
        }));
        results
    }
    fn get_cube_mut(
        &mut self,
        corner_1: SVector<i32, 2>,
        corner_2: SVector<i32, 2>,
    ) -> Vec<&mut Cell<T, SVector<i32, 2>>> {
        if !self.in_grid_bounds(&corner_1) || !self.in_grid_bounds(&corner_2) {
            panic!("Get Cube Mut error: corners out of range.")
        }
        let x_range = (corner_1[0] as usize)..(corner_2[0] as usize);
        let mut results = Vec::with_capacity(self.size.product() as usize);

        let start_id = (corner_1[1] * self.size[0]) as usize;
        let end_id = (corner_2[1] * self.size[0]) as usize;
        let relevant_data = &mut self.data[start_id..end_id];
        for row in relevant_data.chunks_exact_mut(self.size[0] as usize) {
            results.extend(row[x_range.clone()].iter_mut());
        }
        results
    }
    fn in_grid_bounds(&self, position: &SVector<i32, 2>) -> bool {
        0 <= position[0]
            && position[0] < self.size[0]
            && 0 <= position[1]
            && position[1] < self.size[1]
    }
}

pub struct Grid3D<T: Clone> {
    data: Vec<Cell<T, SVector<i32, 3>>>,
    size: SVector<i32, 3>,
}
impl<T: Clone> Grid<T, SVector<i32, 3>> for Grid3D<T> {
    fn new(size: SVector<i32, 3>) -> impl Grid<T, SVector<i32, 3>> {
        let mut data = Vec::with_capacity((size[0] * size[1] * size[2]) as usize);
        for z in 0..size[2] {
            for y in 0..size[1] {
                for x in 0..size[0] {
                    data.push(Cell::<T, SVector<i32, 3>> {
                        position: SVector::from([x, y, z]),
                        inner: None,
                    });
                }
            }
        }
        Self { data, size }
    }
    fn insert(&mut self, data: T, position: SVector<i32, 3>) {
        if !self.in_grid_bounds(&position) {
            panic!("Insertion error: position out of range.")
        }
        let grid_id = position[0] + position[1] * self.size[0] + position[2] * self.size[1];
        self.data[grid_id as usize].inner = Some(data);
    }
    fn remove(&mut self, position: SVector<i32, 3>) -> Cell<T, SVector<i32, 3>> {
        if !self.in_grid_bounds(&position) {
            panic!("Removal error: position out of range.")
        }
        let grid_id = position[0] + position[1] * self.size[0] + position[2] * self.size[1];
        let old_value = self.data[grid_id as usize].clone();
        self.data[grid_id as usize].inner = None;
        old_value
    }
    fn get_ref(&self, position: SVector<i32, 3>) -> &Cell<T, SVector<i32, 3>> {
        if !self.in_grid_bounds(&position) {
            panic!("Get error: position out of range.")
        }
        let grid_id = position[0] + position[1] * self.size[0] + position[2] * self.size[1];
        &self.data[grid_id as usize]
    }
    fn get_mut(&mut self, position: SVector<i32, 3>) -> &mut Cell<T, SVector<i32, 3>> {
        if !self.in_grid_bounds(&position) {
            panic!("Get error: position out of range.")
        }
        let grid_id = position[0] + position[1] * self.size[0] + position[2] * self.size[1];
        &mut self.data[grid_id as usize]
    }
    fn get_cube_ref(
        &self,
        corner_1: SVector<i32, 3>,
        corner_2: SVector<i32, 3>,
    ) -> Vec<&Cell<T, SVector<i32, 3>>> {
        if !self.in_grid_bounds(&corner_1) || !self.in_grid_bounds(&corner_2) {
            panic!("Get Cube Mut error: corners out of range.")
        }
        let mut results = Vec::new();
        results.extend((corner_1[1]..corner_2[1]).flat_map(|y| {
            let y_offset = (y * self.size[0]) as usize;
            (corner_1[0]..corner_2[0]).map(move |x| &self.data[y_offset + x as usize])
        }));
        results
    }
    fn get_cube_mut(
        &mut self,
        corner_1: SVector<i32, 3>,
        corner_2: SVector<i32, 3>,
    ) -> Vec<&mut Cell<T, SVector<i32, 3>>> {
        if !self.in_grid_bounds(&corner_1) || !self.in_grid_bounds(&corner_2) {
            panic!("Get Cube Mut error: corners out of range.")
        }
        let x_range = (corner_1[0] as usize)..(corner_2[0] as usize);
        let mut results = Vec::with_capacity(self.size.product() as usize);

        let start_id = (corner_1[1] * self.size[0]) as usize;
        let end_id = (corner_2[1] * self.size[0]) as usize;
        let relevant_data = &mut self.data[start_id..end_id];
        for row in relevant_data.chunks_exact_mut(self.size[0] as usize) {
            results.extend(row[x_range.clone()].iter_mut());
        }
        results
    }
    fn in_grid_bounds(&self, position: &SVector<i32, 3>) -> bool {
        0 <= position[0]
            && position[0] < self.size[0]
            && 0 <= position[1]
            && position[1] < self.size[1]
            && 0 <= position[2]
            && position[2] < self.size[2]
    }
}

pub struct SparseGrid2D {}
impl SparseGrid2D {}

pub struct SparseGrid3D {}
impl SparseGrid3D {}
