use std::hint::black_box;
use std::time::Duration;

use criterion::{BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use paraxis::containers::array::*;
use rand::prelude::*;
use rand::rngs::StdRng;

const SIZES: [usize; 4] = [8, 16, 32, 64];

fn random_rect(rows: usize, cols: usize, seed: u64) -> Array<f64> {
    let mut rng = StdRng::seed_from_u64(seed);
    let data: Vec<f64> = (0..rows * cols)
        .map(|_| rng.random_range(-10.0..10.0))
        .collect();
    Array::from_vec_shape(data, &[rows, cols])
}

fn random_square(n: usize, seed: u64) -> Array<f64> {
    random_rect(n, n, seed)
}

fn random_spd_matrix(n: usize, seed: u64) -> Array<f64> {
    let a = random_square(n, seed);
    let at = a.clone().transpose();
    let ata = at.contract(&a);
    let mut data = ata.to_cloned_vec();
    for i in 0..n {
        data[i * n + i] += 10.0;
    }
    Array::from_vec_shape(data, &[n, n])
}

fn random_clustered_spd_matrix(n: usize, seed: u64) -> Array<f64> {
    let mut rng = StdRng::seed_from_u64(seed);
    let mut diag_data = vec![0.0; n * n];
    for i in 0..n {
        let cluster_noise = rng.random_range(-1e-3..1e-3);
        diag_data[i * n + i] = 10.0 + cluster_noise;
    }
    if n >= 2 {
        diag_data[0] = 50.0;
    }
    let diag = Array::from_vec_shape(diag_data, &[n, n]);

    let q = random_square(n, seed.wrapping_add(1)).qr().q;
    let qt = q.clone().transpose();
    q.contract(&diag).contract(&qt)
}

fn bench_qr(c: &mut Criterion) {
    let mut group = c.benchmark_group("qr");
    for &n in &SIZES {
        let m = random_square(n, 42);
        group.throughput(Throughput::Elements((n * n) as u64));
        group.bench_with_input(BenchmarkId::from_parameter(n), &m, |b, m| {
            b.iter(|| black_box(m).qr())
        });
    }
    group.finish();
}

fn bench_lu(c: &mut Criterion) {
    let mut group = c.benchmark_group("lu");
    for &n in &SIZES {
        let m = random_square(n, 43);
        group.throughput(Throughput::Elements((n * n) as u64));
        group.bench_with_input(BenchmarkId::from_parameter(n), &m, |b, m| {
            b.iter(|| black_box(m).lu())
        });
    }
    group.finish();
}

fn bench_eigen(c: &mut Criterion) {
    let mut group = c.benchmark_group("eigen");
    for &n in &SIZES {
        let m = random_spd_matrix(n, 44);
        group.throughput(Throughput::Elements((n * n) as u64));
        group.bench_with_input(BenchmarkId::from_parameter(n), &m, |b, m| {
            b.iter(|| black_box(m).eigen())
        });
    }
    group.finish();
}

fn bench_eigen_clustered(c: &mut Criterion) {
    let mut group = c.benchmark_group("eigen_clustered");
    for &n in &[8usize, 16, 32, 64] {
        let m = random_clustered_spd_matrix(n, 60);
        group.throughput(Throughput::Elements((n * n) as u64));
        group.bench_with_input(BenchmarkId::from_parameter(n), &m, |b, m| {
            b.iter(|| black_box(m).eigen())
        });
    }
    group.finish();
}

fn bench_solve(c: &mut Criterion) {
    let mut group = c.benchmark_group("solve");
    for &n in &SIZES {
        let m = random_square(n, 48);
        let b_vec = Array::from_vec((0..n).map(|i| i as f64 + 1.0).collect());
        group.throughput(Throughput::Elements(n as u64));
        group.bench_with_input(
            BenchmarkId::from_parameter(n),
            &(m, b_vec),
            |bch, (m, b_vec)| bch.iter(|| black_box(m).solve(black_box(b_vec))),
        );
    }
    group.finish();
}

fn bench_det(c: &mut Criterion) {
    let mut group = c.benchmark_group("det");
    for &n in &SIZES {
        let m = random_square(n, 49);
        group.throughput(Throughput::Elements((n * n) as u64));
        group.bench_with_input(BenchmarkId::from_parameter(n), &m, |b, m| {
            b.iter(|| black_box(m).det())
        });
    }
    group.finish();
}

fn bench_inverse(c: &mut Criterion) {
    let mut group = c.benchmark_group("inverse");
    for &n in &SIZES {
        let m = random_square(n, 50);
        group.throughput(Throughput::Elements((n * n) as u64));
        group.bench_with_input(BenchmarkId::from_parameter(n), &m, |b, m| {
            b.iter(|| black_box(m).inverse())
        });
    }
    group.finish();
}

fn bench_rank(c: &mut Criterion) {
    let mut group = c.benchmark_group("rank");
    for &n in &SIZES {
        let m = random_square(n, 51);
        group.throughput(Throughput::Elements((n * n) as u64));
        group.bench_with_input(BenchmarkId::from_parameter(n), &m, |b, m| {
            b.iter(|| black_box(m).rank())
        });
    }
    group.finish();
}

fn bench_contract(c: &mut Criterion) {
    let mut group = c.benchmark_group("contract");
    for &n in &SIZES {
        let a = random_square(n, 52);
        let b = random_square(n, 53);
        group.throughput(Throughput::Elements((n * n * n) as u64));
        group.bench_with_input(BenchmarkId::from_parameter(n), &(a, b), |bch, (a, b)| {
            bch.iter(|| black_box(a).contract(black_box(b)))
        });
    }
    group.finish();
}

fn bench_svd_internals(c: &mut Criterion) {
    let mut group = c.benchmark_group("svd_internals");
    for &n in &[16usize, 32, 64] {
        let m = random_square(n, 54);
        group.bench_with_input(BenchmarkId::new("column_norms", n), &m, |b, m| {
            b.iter(|| black_box(m).column_norms())
        });
        group.bench_with_input(BenchmarkId::new("normalize_columns", n), &m, |b, m| {
            b.iter(|| black_box(m).normalize_columns(black_box(1e-14)))
        });
    }
    group.finish();
}

criterion_group! {
    name = benches;
    config = Criterion::default().measurement_time(Duration::from_secs(10));
    targets =
        bench_qr,
        bench_lu,
        bench_eigen,
        bench_eigen_clustered,
        bench_solve,
        bench_det,
        bench_inverse,
        bench_rank,
        bench_contract,
        bench_svd_internals,
}
criterion_main!(benches);
