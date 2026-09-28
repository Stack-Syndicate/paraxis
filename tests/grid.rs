use paraxis::common::{errors::ParaxisError, structs::Node, traits::Grid};
use paraxis::containers::grid::{ContinuousGrid, DenseGrid, SparseGrid};

fn node_inner<P, D: Clone>(node: &Node<P, D>) -> Option<D> {
    node.write().inner.clone()
}

#[test]
fn continuous_new_rejects_negative_size() {
    let result = ContinuousGrid::<[f32; 2], i32>::new(&[-1.0, 2.0]);
    assert!(matches!(result, Err(ParaxisError::NegativeSize)));
}

#[test]
fn continuous_new_accepts_zero_size() {
    let result = ContinuousGrid::<[f32; 2], i32>::new(&[0.0, 2.0]);
    assert!(result.is_ok());
}

#[test]
fn continuous_bounds_are_half_open() {
    let grid = ContinuousGrid::<[f32; 2], i32>::new(&[2.0, 3.0]).unwrap();
    assert!(grid.in_grid_bounds(&[0.0, 0.0]));
    assert!(grid.in_grid_bounds(&[1.999, 2.999]));
    assert!(!grid.in_grid_bounds(&[-0.001, 0.0]));
    assert!(!grid.in_grid_bounds(&[2.0, 0.0]));
    assert!(!grid.in_grid_bounds(&[0.0, 3.0]));
}

#[test]
fn continuous_insert_and_get() {
    let mut grid = ContinuousGrid::<[f32; 2], i32>::new(&[4.0, 4.0]).unwrap();
    let position = [1.25, 2.5];
    grid.insert(42, &position).unwrap();
    let node = grid.get(&position).unwrap();
    assert_eq!(node_inner(node), Some(42));
}

#[test]
fn continuous_get_mut_changes_value() {
    let mut grid = ContinuousGrid::<[f32; 2], i32>::new(&[4.0, 4.0]).unwrap();
    let position = [1.25, 2.5];
    grid.insert(42, &position).unwrap();
    let node = grid.get_mut(&position).unwrap();
    node.write().inner = Some(99);
    assert_eq!(node_inner(grid.get(&position).unwrap()), Some(99));
}

#[test]
fn continuous_remove_returns_value_and_clears_node() {
    let mut grid = ContinuousGrid::<[f32; 2], i32>::new(&[4.0, 4.0]).unwrap();
    let position = [1.25, 2.5];
    grid.insert(42, &position).unwrap();
    let removed = grid.remove(&position).unwrap();
    assert_eq!(node_inner(&removed), Some(42));
    assert_eq!(node_inner(grid.get(&position).unwrap()), None);
}

#[test]
fn continuous_missing_node_is_an_error() {
    let grid = ContinuousGrid::<[f32; 2], i32>::new(&[4.0, 4.0]).unwrap();
    let position = [1.25, 2.5];
    assert!(matches!(grid.get(&position), Err(ParaxisError::UnintNode)));
}

#[test]
fn continuous_out_of_bounds_operations_are_rejected() {
    let mut grid = ContinuousGrid::<[f32; 2], i32>::new(&[4.0, 4.0]).unwrap();
    let position = [4.0, 1.0];
    assert!(matches!(
        grid.insert(42, &position),
        Err(ParaxisError::OutOfBounds)
    ));
    assert!(matches!(
        grid.get(&position),
        Err(ParaxisError::OutOfBounds)
    ));
    assert!(matches!(
        grid.get_mut(&position),
        Err(ParaxisError::OutOfBounds)
    ));
    assert!(matches!(
        grid.remove(&position),
        Err(ParaxisError::OutOfBounds)
    ));
}

#[test]
fn continuous_distinguishes_different_coordinates() {
    let mut grid = ContinuousGrid::<[f32; 2], i32>::new(&[4.0, 4.0]).unwrap();
    let a = [1.0, 2.0];
    let b = [1.0, 2.001];
    grid.insert(10, &a).unwrap();
    grid.insert(20, &b).unwrap();
    assert_eq!(node_inner(grid.get(&a).unwrap()), Some(10));
    assert_eq!(node_inner(grid.get(&b).unwrap()), Some(20));
}

#[test]
fn continuous_insert_does_not_overwrite_existing_node_current_behavior() {
    let mut grid = ContinuousGrid::<[f32; 2], i32>::new(&[4.0, 4.0]).unwrap();
    let position = [1.0, 2.0];
    grid.insert(10, &position).unwrap();
    grid.insert(20, &position).unwrap();
    assert_eq!(node_inner(grid.get(&position).unwrap()), Some(10));
}

#[test]
fn dense_new_creates_every_grid_position() {
    let grid = DenseGrid::<[i32; 2], i32>::new(&[2, 3]).unwrap();
    for x in 0..2 {
        for y in 0..3 {
            assert!(grid.get(&[x, y]).is_ok());
        }
    }
    assert!(matches!(grid.get(&[2, 0]), Err(ParaxisError::OutOfBounds)));
}

#[test]
fn dense_new_rejects_negative_size() {
    let result = DenseGrid::<[i32; 2], i32>::new(&[-1, 2]);
    assert!(matches!(result, Err(ParaxisError::NegativeSize)));
}

#[test]
fn dense_bounds_are_half_open() {
    let grid = DenseGrid::<[i32; 2], i32>::new(&[2, 3]).unwrap();
    assert!(grid.in_grid_bounds(&[0, 0]));
    assert!(grid.in_grid_bounds(&[1, 2]));
    assert!(!grid.in_grid_bounds(&[-1, 0]));
    assert!(!grid.in_grid_bounds(&[2, 0]));
    assert!(!grid.in_grid_bounds(&[0, 3]));
}

#[test]
fn dense_cells_start_empty() {
    let grid = DenseGrid::<[i32; 2], i32>::new(&[2, 2]).unwrap();
    for position in [[0, 0], [0, 1], [1, 0], [1, 1]] {
        assert_eq!(node_inner(grid.get(&position).unwrap()), None);
    }
}

#[test]
fn dense_insert_and_get() {
    let mut grid = DenseGrid::<[i32; 2], i32>::new(&[4, 4]).unwrap();
    let position = [1, 2];
    grid.insert(42, &position).unwrap();
    assert_eq!(node_inner(grid.get(&position).unwrap()), Some(42));
}

#[test]
fn dense_insert_replaces_existing_value() {
    let mut grid = DenseGrid::<[i32; 2], i32>::new(&[4, 4]).unwrap();
    let position = [1, 2];
    grid.insert(10, &position).unwrap();
    grid.insert(20, &position).unwrap();
    assert_eq!(node_inner(grid.get(&position).unwrap()), Some(20));
}

#[test]
fn dense_get_mut_changes_value() {
    let mut grid = DenseGrid::<[i32; 2], i32>::new(&[4, 4]).unwrap();
    let position = [1, 2];
    grid.insert(42, &position).unwrap();
    let node = grid.get_mut(&position).unwrap();
    node.write().inner = Some(99);
    assert_eq!(node_inner(grid.get(&position).unwrap()), Some(99));
}

#[test]
fn dense_remove_returns_value_and_clears_cell() {
    let mut grid = DenseGrid::<[i32; 2], i32>::new(&[4, 4]).unwrap();
    let position = [1, 2];
    grid.insert(42, &position).unwrap();
    let removed = grid.remove(&position).unwrap();
    assert_eq!(node_inner(&removed), Some(42));
    assert_eq!(node_inner(grid.get(&position).unwrap()), None);
}

#[test]
fn dense_out_of_bounds_operations_are_rejected() {
    let mut grid = DenseGrid::<[i32; 2], i32>::new(&[4, 4]).unwrap();
    let position = [4, 1];
    assert!(matches!(
        grid.insert(42, &position),
        Err(ParaxisError::OutOfBounds)
    ));
    assert!(matches!(
        grid.get(&position),
        Err(ParaxisError::OutOfBounds)
    ));
    assert!(matches!(
        grid.get_mut(&position),
        Err(ParaxisError::OutOfBounds)
    ));
    assert!(matches!(
        grid.remove(&position),
        Err(ParaxisError::OutOfBounds)
    ));
}

#[test]
fn dense_uses_requested_coordinate_type() {
    let grid = DenseGrid::<[u8; 3], i32>::new(&[2, 3, 4]).unwrap();
    assert!(grid.get(&[0, 0, 0]).is_ok());
    assert!(grid.get(&[1, 2, 3]).is_ok());
}

#[test]
fn sparse_new_starts_empty() {
    let grid = SparseGrid::<[i32; 2], i32>::new(&[4, 4]).unwrap();
    let position = [1, 2];
    assert!(matches!(grid.get(&position), Err(ParaxisError::UnintNode)));
}

#[test]
fn sparse_new_rejects_negative_size() {
    let result = SparseGrid::<[i32; 2], i32>::new(&[-1, 2]);
    assert!(matches!(result, Err(ParaxisError::NegativeSize)));
}

#[test]
fn sparse_bounds_are_half_open() {
    let grid = SparseGrid::<[i32; 2], i32>::new(&[2, 3]).unwrap();
    assert!(grid.in_grid_bounds(&[0, 0]));
    assert!(grid.in_grid_bounds(&[1, 2]));
    assert!(!grid.in_grid_bounds(&[-1, 0]));
    assert!(!grid.in_grid_bounds(&[2, 0]));
    assert!(!grid.in_grid_bounds(&[0, 3]));
}

#[test]
fn sparse_insert_and_get() {
    let mut grid = SparseGrid::<[i32; 2], i32>::new(&[4, 4]).unwrap();
    let position = [1, 2];
    grid.insert(42, &position).unwrap();
    assert_eq!(node_inner(grid.get(&position).unwrap()), Some(42));
}

#[test]
fn sparse_insert_replaces_existing_value() {
    let mut grid = SparseGrid::<[i32; 2], i32>::new(&[4, 4]).unwrap();
    let position = [1, 2];
    grid.insert(10, &position).unwrap();
    grid.insert(20, &position).unwrap();
    assert_eq!(node_inner(grid.get(&position).unwrap()), Some(20));
}

#[test]
fn sparse_get_mut_changes_value() {
    let mut grid = SparseGrid::<[i32; 2], i32>::new(&[4, 4]).unwrap();
    let position = [1, 2];
    grid.insert(42, &position).unwrap();
    let node = grid.get_mut(&position).unwrap();
    node.write().inner = Some(99);
    assert_eq!(node_inner(grid.get(&position).unwrap()), Some(99));
}

#[test]
fn sparse_remove_returns_value_and_clears_node() {
    let mut grid = SparseGrid::<[i32; 2], i32>::new(&[4, 4]).unwrap();
    let position = [1, 2];
    grid.insert(42, &position).unwrap();
    let removed = grid.remove(&position).unwrap();
    assert_eq!(node_inner(&removed), Some(42));
    assert_eq!(node_inner(grid.get(&position).unwrap()), None);
}

#[test]
fn sparse_out_of_bounds_operations_are_rejected() {
    let mut grid = SparseGrid::<[i32; 2], i32>::new(&[4, 4]).unwrap();
    let position = [4, 1];
    assert!(matches!(
        grid.insert(42, &position),
        Err(ParaxisError::OutOfBounds)
    ));
    assert!(matches!(
        grid.get(&position),
        Err(ParaxisError::OutOfBounds)
    ));
    assert!(matches!(
        grid.get_mut(&position),
        Err(ParaxisError::OutOfBounds)
    ));
    assert!(matches!(
        grid.remove(&position),
        Err(ParaxisError::OutOfBounds)
    ));
}

#[test]
fn sparse_keeps_distinct_positions_separate() {
    let mut grid = SparseGrid::<[i32; 2], i32>::new(&[4, 4]).unwrap();
    let a = [1, 2];
    let b = [1, 3];
    grid.insert(10, &a).unwrap();
    grid.insert(20, &b).unwrap();
    assert_eq!(node_inner(grid.get(&a).unwrap()), Some(10));
    assert_eq!(node_inner(grid.get(&b).unwrap()), Some(20));
}
