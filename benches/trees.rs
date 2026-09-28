use criterion::{Criterion, criterion_group, criterion_main};
use paraxis::prelude::*;
use rand::RngExt;

fn kd_tree(c: &mut Criterion) {
    let mut rng = rand::rng();
    let mut points = Vec::new();
    for _ in 0..1_000_000 {
        points.push((
            [rng.random_range(0.0..=100.0), rng.random_range(0.0..=100.0)],
            (),
        ));
    }
    let tree = KDTree::new(points);
    c.bench_function("kd_tree/nearest_neighbour", |b| {
        b.iter(|| tree.k_nearest_neighbours(&[50.0, 50.0], 1));
    });
    c.bench_function("kd_tree/5_nearest_neighbours", |b| {
        b.iter(|| tree.k_nearest_neighbours(&[50.0, 50.0], 5));
    });
    c.bench_function("kd_tree/100_nearest_neighbours", |b| {
        b.iter(|| tree.k_nearest_neighbours(&[50.0, 50.0], 100));
    });
}

fn bi_hierarchy(c: &mut Criterion) {
    let mut rng = rand::rng();
    let mut points = Vec::new();
    for _ in 0..1_000_000 {
        points.push((
            [rng.random_range(0.0..=100.0), rng.random_range(0.0..=100.0)],
            (),
        ));
    }
    let tree = BIHierarchy::new(points);
    c.bench_function("bi_hierarchy/nearest_neighbour", |b| {
        b.iter(|| tree.k_nearest_neighbours(&[50.0, 50.0], 1));
    });
    c.bench_function("bi_hierarchy/5_nearest_neighbours", |b| {
        b.iter(|| tree.k_nearest_neighbours(&[50.0, 50.0], 5));
    });
    c.bench_function("bi_hierarchy/100_nearest_neighbours", |b| {
        b.iter(|| tree.k_nearest_neighbours(&[50.0, 50.0], 100));
    });
}

criterion_group!(benches, kd_tree, bi_hierarchy);
criterion_main!(benches);
