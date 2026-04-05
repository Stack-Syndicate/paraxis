use macroquad::prelude::*;
use paraxis::{dsa::tree::kd::KDTree, maths::vec::Vector};

fn draw_tree_bounds(
    data: &[(Vector<f32, 2>, ())],
    depth: usize,
    min_x: f32,
    max_x: f32,
    min_y: f32,
    max_y: f32,
) {
    if data.is_empty() {
        return;
    }
    let mid_idx = data.len() / 2;
    let (point, _) = &data[mid_idx];
    let axis = depth % 2;
    if axis == 0 {
        draw_line(point.inner[0], min_y, point.inner[0], max_y, 1.0, BLUE);
        draw_tree_bounds(
            &data[..mid_idx],
            depth + 1,
            min_x,
            point.inner[0],
            min_y,
            max_y,
        );
        draw_tree_bounds(
            &data[mid_idx + 1..],
            depth + 1,
            point.inner[0],
            max_x,
            min_y,
            max_y,
        );
    } else {
        draw_line(min_x, point.inner[1], max_x, point.inner[1], 1.0, RED);
        draw_tree_bounds(
            &data[..mid_idx],
            depth + 1,
            min_x,
            max_x,
            min_y,
            point.inner[1],
        );
        draw_tree_bounds(
            &data[mid_idx + 1..],
            depth + 1,
            min_x,
            max_x,
            point.inner[1],
            max_y,
        );
    }
}
#[macroquad::main("KDTree Visualizer")]
async fn main() {
    let mut raw_points: Vec<(Vector<f32, 2>, ())> = Vec::new();
    let mut tree = KDTree::new(&raw_points);
    loop {
        clear_background(BLACK);
        if is_mouse_button_down(MouseButton::Left) {
            let (x, y) = mouse_position();
            raw_points.push((Vector { inner: [x, y] }, ()));
            tree = KDTree::new(&raw_points);
        }
        draw_tree_bounds(&tree.data, 0, 0.0, screen_width(), 0.0, screen_height());
        for (p, _) in &tree.data {
            draw_circle(p.inner[0], p.inner[1], 3.0, WHITE);
        }
        draw_text("Click to add points", 20.0, 30.0, 20.0, WHITE);
        draw_text(
            &format!("Total Points: {}", tree.data.len()),
            20.0,
            50.0,
            20.0,
            GREEN,
        );
        let mouse_position = mouse_position();
        if !tree.data.is_empty() {
            let nearest_neighbour = tree
                .nearest_neighbour_euclidean(Vector::new([mouse_position.0, mouse_position.1]))
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
        draw_fps();
        next_frame().await
    }
}
