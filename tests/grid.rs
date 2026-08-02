use paraxis::{common::traits::Grid, containers::grid::DenseGrid};

#[test]
fn initialization() {
    assert!(DenseGrid::<[i32; 2], i32>::new(&[10, 10]).is_ok());
    assert!(DenseGrid::<[i32; 2], i32>::new(&[10, 10]).is_ok());
}

#[test]
fn insert_get() {
    let mut grid = DenseGrid::new(&[50, 50]).unwrap();
    let pos = [12, 40];
    assert!(grid.insert(42, &pos).is_ok());
    let node = grid.get(&pos).unwrap();
    assert_eq!(node.position, pos);
    assert_eq!(node.inner, Some(42));
    let empty_node = grid.get(&[10, 10]).unwrap();
    assert!(empty_node.inner.is_none());
}

#[test]
fn insert_remove() {
    let mut grid = DenseGrid::new(&[50, 50]).unwrap();
    let pos = [12, 40];
    assert!(grid.insert(10, &pos).is_ok());
    assert!(grid.remove(&pos).is_ok());
    assert!(grid.get(&pos).unwrap().inner.is_none());
}

#[test]
fn bounds_checking() {
    let mut grid = DenseGrid::new(&[10, 10]).unwrap();
    let edge_pos = [9, 9];
    assert!(grid.insert(1, &edge_pos).is_ok());
    assert!(grid.get(&edge_pos).is_ok());
    let oob_pos = [10, 5];
    assert!(grid.insert(2, &oob_pos).is_err());
    assert!(grid.get(&oob_pos).is_err());
    let neg_pos = [-1, 5];
    assert!(grid.insert(3, &neg_pos).is_err());
}

#[test]
fn multidimensional_3d_and_4d() {
    let mut grid3d = DenseGrid::new(&[5, 5, 5]).unwrap();
    let pos3d = [2, 3, 4];
    assert!(grid3d.insert("cube", &pos3d).is_ok());
    assert_eq!(grid3d.get(&pos3d).unwrap().inner, Some("cube"));
    let mut grid4d = DenseGrid::new(&[3, 3, 3, 3]).unwrap();
    let pos4d = [1, 2, 0, 2];
    assert!(grid4d.insert(99, &pos4d).is_ok());
    assert_eq!(grid4d.get(&pos4d).unwrap().inner, Some(99));
}

#[test]
fn insert_overwrite() {
    let mut grid = DenseGrid::new(&[5, 5]).unwrap();
    let pos = [2, 2];
    assert!(grid.insert(100, &pos).is_ok());
    assert_eq!(grid.get(&pos).unwrap().inner, Some(100));
    assert!(grid.insert(200, &pos).is_ok());
    assert_eq!(grid.get(&pos).unwrap().inner, Some(200));
}

#[test]
fn zero_size_dimension() {
    let grid = DenseGrid::<[i32; 2], i32>::new(&[0, 10]);
    assert!(grid.is_ok());
    if let Ok(g) = grid {
        assert!(g.get(&[0, 0]).is_err());
    }
}
