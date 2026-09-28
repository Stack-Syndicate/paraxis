use paraxis::{
    common::traits::Tree,
    containers::tree::{BIHierarchy, KDTree},
};

#[test]
fn kd_tree_empty() {
    let tree = KDTree::<[f64; 2], usize>::new(Vec::new());
    assert!(
        tree.k_nearest_neighbours(&[0.0, 0.0], 1)
            .unwrap()
            .is_empty()
    );
}

#[test]
fn kd_tree_single_point() {
    let tree = KDTree::<[f64; 2], usize>::new(vec![([1.0, 2.0], 42)]);
    let result = tree.k_nearest_neighbours(&[1.0, 2.0], 1).unwrap();
    assert_eq!(result.len(), 1);
    assert_eq!(result[0].position, [1.0, 2.0]);
}

#[test]
fn kd_tree_contains_all_points() {
    let input = vec![
        ([0.0, 0.0], 0),
        ([1.0, 0.0], 1),
        ([0.0, 1.0], 2),
        ([1.0, 1.0], 3),
        ([2.0, 2.0], 4),
        ([-1.0, -1.0], 5),
        ([3.0, 0.0], 6),
        ([0.0, 3.0], 7),
    ];
    let tree = KDTree::<[f64; 2], usize>::new(input.clone());
    let result = tree.k_nearest_neighbours(&[0.0, 0.0], input.len()).unwrap();
    assert_eq!(result.len(), input.len());
    for (position, _) in input {
        assert!(result.iter().any(|node| node.position == position));
    }
}

#[test]
fn kd_tree_k_zero() {
    let tree = KDTree::<[f64; 2], usize>::new(vec![([0.0, 0.0], 0), ([1.0, 1.0], 1)]);
    assert!(
        tree.k_nearest_neighbours(&[0.0, 0.0], 0)
            .unwrap()
            .is_empty()
    );
}

#[test]
fn kd_tree_k_larger_than_tree() {
    let tree =
        KDTree::<[f64; 2], usize>::new(vec![([0.0, 0.0], 0), ([1.0, 1.0], 1), ([2.0, 2.0], 2)]);
    let result = tree.k_nearest_neighbours(&[0.0, 0.0], 100).unwrap();
    assert_eq!(result.len(), 3);
}

#[test]
fn kd_tree_nearest_neighbour() {
    let tree = KDTree::<[f64; 2], usize>::new(vec![
        ([0.0, 0.0], 0),
        ([1.0, 0.0], 1),
        ([0.0, 1.0], 2),
        ([10.0, 10.0], 3),
    ]);
    let result = tree.k_nearest_neighbours(&[0.9, 0.1], 1).unwrap();
    assert_eq!(result.len(), 1);
    assert_eq!(result[0].position, [1.0, 0.0]);
}

#[test]
fn kd_tree_nearest_neighbours_are_sorted() {
    let tree = KDTree::<[f64; 2], usize>::new(vec![
        ([0.0, 0.0], 0),
        ([1.0, 0.0], 1),
        ([2.0, 0.0], 2),
        ([3.0, 0.0], 3),
    ]);
    let result = tree.k_nearest_neighbours(&[0.0, 0.0], 4).unwrap();
    let distances: Vec<f64> = result
        .iter()
        .map(|node| node.position[0] * node.position[0] + node.position[1] * node.position[1])
        .collect();
    for pair in distances.windows(2) {
        assert!(pair[0] <= pair[1]);
    }
}

#[test]
fn kd_tree_duplicate_positions() {
    let tree =
        KDTree::<[f64; 2], usize>::new(vec![([1.0, 1.0], 0), ([1.0, 1.0], 1), ([1.0, 1.0], 2)]);
    let result = tree.k_nearest_neighbours(&[1.0, 1.0], 3).unwrap();
    assert_eq!(result.len(), 3);
    for node in result {
        assert_eq!(node.position, [1.0, 1.0]);
    }
}

#[test]
fn kd_tree_insertion() {
    let mut tree = KDTree::<[f64; 2], usize>::new(vec![([0.0, 0.0], 0), ([10.0, 10.0], 1)]);
    tree.add(&[1.0, 1.0], 2);
    tree.add(&[-1.0, -1.0], 3);
    let result = tree.k_nearest_neighbours(&[1.0, 1.0], 4).unwrap();
    assert_eq!(result.len(), 4);
    assert_eq!(result[0].position, [1.0, 1.0]);
}

#[test]
fn kd_tree_rebalances_after_insertions() {
    let mut tree = KDTree::<[f64; 2], usize>::new(vec![([0.0, 0.0], 0)]);
    for i in 1..100 {
        let x = i as f64;
        tree.add(&[x, x], i);
    }
    let result = tree.k_nearest_neighbours(&[50.0, 50.0], 1).unwrap();
    assert_eq!(result.len(), 1);
    assert_eq!(result[0].position, [50.0, 50.0]);
}

#[test]
fn bi_hierarchy_empty() {
    let tree = BIHierarchy::<[f64; 2], usize>::new(Vec::new());
    assert!(
        tree.k_nearest_neighbours(&[0.0, 0.0], 1)
            .unwrap()
            .is_empty()
    );
}

#[test]
fn bi_hierarchy_single_point() {
    let tree = BIHierarchy::<[f64; 2], usize>::new(vec![([1.0, 2.0], 42)]);
    let result = tree.k_nearest_neighbours(&[1.0, 2.0], 1).unwrap();
    assert_eq!(result.len(), 1);
    assert_eq!(result[0].position, [1.0, 2.0]);
}

#[test]
fn bi_hierarchy_contains_all_points() {
    let input = vec![
        ([0.0, 0.0], 0),
        ([1.0, 0.0], 1),
        ([0.0, 1.0], 2),
        ([1.0, 1.0], 3),
        ([2.0, 2.0], 4),
        ([-1.0, -1.0], 5),
        ([3.0, 0.0], 6),
        ([0.0, 3.0], 7),
    ];
    let tree = BIHierarchy::<[f64; 2], usize>::new(input.clone());
    let result = tree.k_nearest_neighbours(&[0.0, 0.0], input.len()).unwrap();
    assert_eq!(result.len(), input.len());
    for (position, _) in input {
        assert!(result.iter().any(|node| node.position == position));
    }
}

#[test]
fn bi_hierarchy_k_zero() {
    let tree = BIHierarchy::<[f64; 2], usize>::new(vec![([0.0, 0.0], 0), ([1.0, 1.0], 1)]);
    assert!(
        tree.k_nearest_neighbours(&[0.0, 0.0], 0)
            .unwrap()
            .is_empty()
    );
}

#[test]
fn bi_hierarchy_k_larger_than_tree() {
    let tree = BIHierarchy::<[f64; 2], usize>::new(vec![
        ([0.0, 0.0], 0),
        ([1.0, 1.0], 1),
        ([2.0, 2.0], 2),
    ]);
    let result = tree.k_nearest_neighbours(&[0.0, 0.0], 100).unwrap();
    assert_eq!(result.len(), 3);
}

#[test]
fn bi_hierarchy_nearest_neighbour() {
    let tree = BIHierarchy::<[f64; 2], usize>::new(vec![
        ([0.0, 0.0], 0),
        ([1.0, 0.0], 1),
        ([0.0, 1.0], 2),
        ([10.0, 10.0], 3),
    ]);
    let result = tree.k_nearest_neighbours(&[0.9, 0.1], 1).unwrap();
    assert_eq!(result.len(), 1);
    assert_eq!(result[0].position, [1.0, 0.0]);
}

#[test]
fn bi_hierarchy_nearest_neighbours_are_sorted() {
    let tree = BIHierarchy::<[f64; 2], usize>::new(vec![
        ([0.0, 0.0], 0),
        ([1.0, 0.0], 1),
        ([2.0, 0.0], 2),
        ([3.0, 0.0], 3),
    ]);
    let result = tree.k_nearest_neighbours(&[0.0, 0.0], 4).unwrap();
    let distances: Vec<f64> = result
        .iter()
        .map(|node| node.position[0] * node.position[0] + node.position[1] * node.position[1])
        .collect();
    for pair in distances.windows(2) {
        assert!(pair[0] <= pair[1]);
    }
}

#[test]
fn bi_hierarchy_duplicate_positions() {
    let tree = BIHierarchy::<[f64; 2], usize>::new(vec![
        ([1.0, 1.0], 0),
        ([1.0, 1.0], 1),
        ([1.0, 1.0], 2),
    ]);
    let result = tree.k_nearest_neighbours(&[1.0, 1.0], 3).unwrap();
    assert_eq!(result.len(), 3);
    for node in result {
        assert_eq!(node.position, [1.0, 1.0]);
    }
}

#[test]
fn bi_hierarchy_all_points_on_one_side_of_split() {
    let tree = BIHierarchy::<[f64; 2], usize>::new(vec![
        ([0.0, 0.0], 0),
        ([1.0, 0.0], 1),
        ([2.0, 0.0], 2),
        ([3.0, 0.0], 3),
    ]);
    let result = tree.k_nearest_neighbours(&[1.5, 0.0], 4).unwrap();
    assert_eq!(result.len(), 4);
}

#[test]
fn bi_hierarchy_many_points_on_split_plane() {
    let tree = BIHierarchy::<[f64; 2], usize>::new(vec![
        ([0.0, 0.0], 0),
        ([2.0, 0.0], 1),
        ([1.0, 0.0], 2),
        ([1.0, 1.0], 3),
        ([1.0, -1.0], 4),
        ([1.0, 2.0], 5),
        ([1.0, -2.0], 6),
    ]);
    let result = tree.k_nearest_neighbours(&[1.0, 0.0], 7).unwrap();
    assert_eq!(result.len(), 7);
}

#[test]
fn bi_hierarchy_unbalanced_coordinate_ranges() {
    let tree = BIHierarchy::<[f64; 3], usize>::new(vec![
        ([0.0, 0.0, 0.0], 0),
        ([100.0, 1.0, 1.0], 1),
        ([200.0, 2.0, 2.0], 2),
        ([300.0, 3.0, 3.0], 3),
        ([400.0, 4.0, 4.0], 4),
        ([500.0, 5.0, 5.0], 5),
    ]);
    let result = tree.k_nearest_neighbours(&[250.0, 2.5, 2.5], 6).unwrap();
    assert_eq!(result.len(), 6);
    for pair in result.windows(2) {
        let a = pair[0].position;
        let b = pair[1].position;
        let da = a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
        let db = b[0] * b[0] + b[1] * b[1] + b[2] * b[2];
        assert!(da.is_finite());
        assert!(db.is_finite());
    }
}

#[test]
fn bi_hierarchy_trace_ray_hits_nearest_voxel() {
    let tree = BIHierarchy::<[f64; 2], usize>::new(vec![
        ([2.0, 0.0], 0),
        ([5.0, 0.0], 1),
        ([8.0, 0.0], 2),
    ]);
    let result = tree.trace_ray([0.0, 0.0], [1.0, 0.0], 0.0, 20.0, 1.0);
    assert!(result.is_some());
    let (distance, node) = result.unwrap();
    assert_eq!(node.position, [2.0, 0.0]);
    assert!(distance >= 0.0);
    assert!(distance < 20.0);
}

#[test]
fn bi_hierarchy_trace_ray_misses() {
    let tree = BIHierarchy::<[f64; 2], usize>::new(vec![
        ([2.0, 0.0], 0),
        ([5.0, 0.0], 1),
        ([8.0, 0.0], 2),
    ]);
    let result = tree.trace_ray([0.0, 10.0], [1.0, 0.0], 0.0, 20.0, 1.0);
    assert!(result.is_none());
}
