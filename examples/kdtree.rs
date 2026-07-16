use ::rand::random_range;
use macroquad::prelude::*;
use nalgebra::SVector;
use paraxis::tree::kd::KDTree;
use rayon::iter::IndexedParallelIterator;
use rayon::iter::ParallelIterator;
use rayon::slice::ParallelSliceMut;

#[macroquad::main("KDTree Visualizer")]
async fn main() {
    let mut raw_points: Vec<(SVector<f32, 2>, ())> = Vec::new();
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
        raw_points.push((SVector::from([x, y]), ()));
        let tree = KDTree::new(raw_points.clone());
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
                        let pos = SVector::from([x as f32, y as f32]);
                        let dist = tree.nearest_neighbour(pos).unwrap().0.metric_distance(&pos);
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
            draw_circle(p[0], p[1], 1.0, WHITE);
        }
        let mouse_position = mouse_position();
        if !tree.data.is_empty() {
            let nearest_neighbour = tree
                .nearest_neighbour(SVector::from([mouse_position.0, mouse_position.1]))
                .unwrap()
                .0;
            draw_line(
                mouse_position.0,
                mouse_position.1,
                nearest_neighbour[0],
                nearest_neighbour[1],
                3.0,
                GREEN,
            );
        }
        draw_text("Click to add points", 20.0, 30.0, 20.0, WHITE);
        draw_text(
            format!("Total Points: {}", tree.data.len()),
            20.0,
            50.0,
            20.0,
            GREEN,
        );
        draw_fps();
        next_frame().await;
    }
}
