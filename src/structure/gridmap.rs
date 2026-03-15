use std::{cmp::Ordering, ops::Index};

use indexmap::{IndexMap, map::Iter};
use nalgebra::SVector;

pub struct GridMap<T, const D: usize> {
    inner: IndexMap<SVector<i32, D>, T>,
}
impl<T, const D: usize> GridMap<T, D> {
    pub fn new() -> Self {
        Self {
            inner: IndexMap::new(),
        }
    }
    pub fn get(&self, position: SVector<i32, D>) -> Option<&T> {
        self.inner.get(&position)
    }
    pub fn get_mut(&mut self, position: SVector<i32, D>) -> Option<&mut T> {
        self.inner.get_mut(&position)
    }
    pub fn sort_by<F: FnMut(&SVector<i32, D>, &T, &SVector<i32, D>, &T) -> Ordering>(
        &mut self,
        cmp: F,
    ) {
        self.inner.sort_by(cmp);
    }
    pub fn insert(&mut self, position: SVector<i32, D>, entry: T) {
        self.inner.insert(position, entry);
    }
    pub fn remove(&mut self, position: &SVector<i32, D>) {
        self.inner.swap_remove(position);
    }
    pub fn shift_remove(&mut self, position: &SVector<i32, D>) {
        self.inner.shift_remove(position);
    }
    pub fn orthogonal_neighbours(
        &self,
        position: SVector<i32, D>,
    ) -> Vec<(SVector<i32, D>, Option<&T>)> {
        let mut neighbours = Vec::with_capacity(2 * D);
        for axis in 0..D {
            for direction in [-1, 1] {
                let mut neighbour_offset = SVector::<i32, D>::zeros();
                neighbour_offset[axis] = direction;
                let neighbour_position = position + neighbour_offset;
                let neighbour = self.get(neighbour_position);
                neighbours.push((neighbour_position, neighbour));
            }
        }
        neighbours
    }
    pub fn moore_neighbours(
        &self,
        position: SVector<i32, D>,
    ) -> Vec<(SVector<i32, D>, Option<&T>)> {
        let total = 3_usize.pow(D as u32);
        let mut neighbours = Vec::with_capacity(total - 1);
        for i in 0..total {
            let mut offset = SVector::<i32, D>::zeros();
            let mut temp = i;
            for axis in 0..D {
                offset[axis] = (temp % 3) as i32 - 1;
                temp /= 3;
            }
            if offset.iter().any(|&v| v != 0) {
                neighbours.push((offset, self.get(position + offset)));
            }
        }
        neighbours
    }
    pub fn raycast(
        &self,
        origin: SVector<f32, D>,
        direction: SVector<f32, D>,
        max_distance: f32,
    ) -> Vec<(SVector<i32, D>, &T)> {
        let mut hit_cells = Vec::new();
        let dir = direction.normalize();
        let mut current_cell = origin.map(|x| x.floor() as i32);
        let delta_dist = dir.map(|short_dir| (1.0 / short_dir).abs());
        let mut step = SVector::<i32, D>::zeros();
        let mut side_dist = SVector::<f32, D>::zeros();
        for i in 0..D {
            if dir[i] < 0.0 {
                step[i] = -1;
                side_dist[i] = (origin[i] - current_cell[i] as f32) * delta_dist[i];
            } else {
                step[i] = 1;
                side_dist[i] = (current_cell[i] as f32 + 1.0 - origin[i]) * delta_dist[i];
            }
        }
        let mut distance = 0.0;
        while distance < max_distance {
            if let Some(value) = self.get(current_cell) {
                hit_cells.push((current_cell, value));
            }
            let mut increment_axis = 0;
            for i in 1..D {
                if side_dist[i] < side_dist[increment_axis] {
                    increment_axis = i;
                }
            }
            distance = side_dist[increment_axis];
            side_dist[increment_axis] += delta_dist[increment_axis];
            current_cell[increment_axis] += step[increment_axis];
        }
        hit_cells
    }
    pub fn raycast_first(
        &self,
        origin: SVector<f32, D>,
        direction: SVector<f32, D>,
        max_distance: f32,
    ) -> Option<(SVector<i32, D>, &T, SVector<i32, D>)> {
        let dir = direction.normalize();
        let mut current_cell = origin.map(|x| x.floor() as i32);
        let delta_dist = dir.map(|d| (1.0 / d).abs());
        let mut step = SVector::<i32, D>::zeros();
        let mut side_dist = SVector::<f32, D>::zeros();
        for i in 0..D {
            if dir[i] < 0.0 {
                step[i] = -1;
                side_dist[i] = (origin[i] - current_cell[i] as f32) * delta_dist[i];
            } else {
                step[i] = 1;
                side_dist[i] = (current_cell[i] as f32 + 1.0 - origin[i]) * delta_dist[i];
            }
        }
        let mut last_axis = 0;
        let mut distance = 0.0;
        while distance < max_distance {
            if let Some(value) = self.get(current_cell) {
                let mut normal = SVector::<i32, D>::zeros();
                normal[last_axis] = -step[last_axis];
                return Some((current_cell, value, normal));
            }
            let mut increment_axis = 0;
            for i in 1..D {
                if side_dist[i] < side_dist[increment_axis] {
                    increment_axis = i;
                }
            }
            distance = side_dist[increment_axis];
            side_dist[increment_axis] += delta_dist[increment_axis];
            current_cell[increment_axis] += step[increment_axis];
            last_axis = increment_axis;
        }
        None
    }
}
impl<'a, T, const D: usize> IntoIterator for &'a GridMap<T, D> {
    type Item = (&'a SVector<i32, D>, &'a T);
    type IntoIter = Iter<'a, SVector<i32, D>, T>;
    fn into_iter(self) -> Self::IntoIter {
        self.inner.iter()
    }
}
impl<const D: usize, T> Index<SVector<i32, D>> for GridMap<T, D> {
    type Output = T;
    fn index(&self, index: SVector<i32, D>) -> &Self::Output {
        self.get(index)
            .expect("No element at that position in the grid.")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Vector2, Vector3};

    type GridMap2D = GridMap<String, 2>;
    type GridMap3D = GridMap<i32, 3>;

    #[test]
    fn insert_get_and_mut() {
        let mut grid = GridMap2D::new();
        let pos = Vector2::new(1, 2);
        grid.insert(pos, "Hello".to_string());
        assert_eq!(grid.get(pos), Some(&"Hello".to_string()));
        assert_eq!(grid.get(Vector2::new(0, 0)), None);
        if let Some(val) = grid.get_mut(pos) {
            *val = "World".to_string();
        }
        assert_eq!(grid.get(pos), Some(&"World".to_string()));
    }

    #[test]
    fn index_trait() {
        let mut grid = GridMap2D::new();
        let pos = Vector2::new(5, 5);
        grid.insert(pos, "Target".to_string());
        assert_eq!(grid[pos], "Target");
    }

    #[test]
    fn removal() {
        let mut grid = GridMap2D::new();
        grid.insert(Vector2::new(0, 0), "A".to_string());
        grid.insert(Vector2::new(1, 1), "B".to_string());
        grid.insert(Vector2::new(2, 2), "C".to_string());
        grid.remove(&Vector2::new(0, 0));
        assert_eq!(grid.get(Vector2::new(0, 0)), None);
        grid.shift_remove(&Vector2::new(1, 1));
        assert_eq!(grid.get(Vector2::new(1, 1)), None);
        assert_eq!(grid.get(Vector2::new(2, 2)), Some(&"C".to_string()));
    }

    #[test]
    fn sorting_and_iteration() {
        let mut grid = GridMap2D::new();
        grid.insert(Vector2::new(2, 0), "C".to_string());
        grid.insert(Vector2::new(0, 0), "A".to_string());
        grid.insert(Vector2::new(1, 0), "B".to_string());
        grid.sort_by(|k1, _, k2, _| k1[0].cmp(&k2[0]));
        let mut iter = grid.into_iter();
        assert_eq!(iter.next().unwrap().1, "A");
        assert_eq!(iter.next().unwrap().1, "B");
        assert_eq!(iter.next().unwrap().1, "C");
        assert!(iter.next().is_none());
    }

    #[test]
    fn orthogonal_neighbours() {
        let mut grid = GridMap2D::new();
        let center = Vector2::new(0, 0);
        grid.insert(Vector2::new(1, 0), "Right".to_string());
        let neighbours = grid.orthogonal_neighbours(center);
        assert_eq!(neighbours.len(), 4);
        let found = neighbours
            .iter()
            .filter(|a| a.1.is_some())
            .collect::<Vec<_>>()
            .len();
        assert_eq!(found, 1);
    }

    #[test]
    fn moore_neighbours_3d() {
        let mut grid = GridMap3D::new();
        let center = Vector3::new(0, 0, 0);
        grid.insert(Vector3::new(1, 1, 1), 42);
        let neighbours = grid.moore_neighbours(center);
        assert_eq!(neighbours.len(), 26);
        let found = neighbours
            .iter()
            .filter(|a| a.1.is_some())
            .collect::<Vec<_>>()
            .len();
        assert_eq!(found, 1);
    }
    #[test]
    fn raycast_multiple_hits() {
        let mut grid = GridMap2D::new();
        grid.insert(Vector2::new(5, 1), "Wall 1".to_string());
        grid.insert(Vector2::new(6, 0), "Wall 2".to_string());
        grid.insert(Vector2::new(7, 0), "Wall 3".to_string());
        let origin = Vector2::new(3.0, 0.0);
        let dir = Vector2::new(1.0, 0.0);
        let hits = grid.raycast(origin, dir, 10.0);
        assert_eq!(hits.len(), 2);
        assert_eq!(hits[0].0, Vector2::new(6, 0));
        assert_eq!(hits[1].0, Vector2::new(7, 0));
    }

    #[test]
    fn raycast_first_with_normal() {
        let mut grid = GridMap2D::new();
        grid.insert(Vector2::new(2, 0), "Target".to_string());
        let origin = Vector2::new(0.5, 0.5);
        let dir = Vector2::new(1.0, 0.0);
        let hit = grid.raycast_first(origin, dir, 5.0);
        assert!(hit.is_some());
        let (pos, val, normal) = hit.unwrap();
        assert_eq!(pos, Vector2::new(2, 0));
        assert_eq!(val, "Target");
        assert_eq!(normal, Vector2::new(-1, 0));
    }

    #[test]
    fn raycast_miss() {
        let mut grid = GridMap2D::new();
        grid.insert(Vector2::new(0, 2), "Top".to_string());

        let hit = grid.raycast_first(Vector2::new(0.5, 0.5), Vector2::new(1.0, 0.0), 5.0);
        assert!(hit.is_none());
    }
}
