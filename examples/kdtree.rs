use ::rand::random_range;
use macroquad::prelude::*;
use paraxis::{dsa::tree::skd::SKDTree, maths::vec::Vector};
use rayon::iter::IndexedParallelIterator;
use rayon::iter::ParallelIterator;
use rayon::slice::ParallelSliceMut;

#[macroquad::main("KDTree Visualizer")]
async fn main() {
    let mut raw_points: Vec<(Vector<f32, 2>, ())> = Vec::new();
    // let mut tree = SKDTree::new(&raw_points, 1); // split_count=1
    loop {
        let width = screen_width() as u32;
        let height = screen_height() as u32;
        let step = 32;
        let grid_w = width / step;
        let grid_h = height / step;
        let mut buffer = vec![(0u32, 0u32, 0.0f32); (grid_w * grid_h) as usize];
        clear_background(BLACK);
        let (x, y) = (
            random_range(0..width) as f32,
            random_range(0..height) as f32,
        );
        raw_points.push((Vector { inner: [x, y] }, ()));
        let tree = SKDTree::new(&raw_points, 1);
        if !tree.data.is_empty() {
            buffer
                .par_chunks_mut(32)
                .enumerate()
                .for_each(|(chunk_idx, chunk)| {
                    let start = chunk_idx * 32;
                    for (i, elem) in chunk.iter_mut().enumerate() {
                        let idx = start + i;
                        if idx >= (grid_w * grid_h) as usize {
                            break;
                        }
                        let x = (idx as u32 % grid_w) * step;
                        let y = (idx as u32 / grid_w) * step;
                        let pos = Vector::new([x as f32, y as f32]);
                        let dist = tree.nearest_neighbour_euclidean(&pos).0.dist_euclidean(pos);
                        let alpha = dist / width as f32 * 2.0;
                        *elem = (x, y, alpha);
                    }
                });
            for &(x, y, a) in &buffer {
                draw_rectangle(
                    x as f32,
                    y as f32,
                    step as f32,
                    step as f32,
                    Color {
                        r: 1.0,
                        g: 0.0,
                        b: 0.0,
                        a,
                    },
                );
            }
        }
        for (p, _) in &tree.data {
            draw_circle(p.inner[0], p.inner[1], 1.0, WHITE);
        }
        let mouse_position = mouse_position();
        if !tree.data.is_empty() {
            let nearest_neighbour = tree
                .nearest_neighbour_euclidean(&Vector::new([mouse_position.0, mouse_position.1]))
                .0;
            draw_line(
                mouse_position.0,
                mouse_position.1,
                nearest_neighbour.inner[0],
                nearest_neighbour.inner[1],
                3.0,
                GREEN,
            );
        }
        draw_text("Click to add points", 20.0, 30.0, 20.0, WHITE);
        draw_text(
            &format!("Total Points: {}", tree.data.len()),
            20.0,
            50.0,
            20.0,
            GREEN,
        );
        draw_fps();
        next_frame().await;
    }
}
