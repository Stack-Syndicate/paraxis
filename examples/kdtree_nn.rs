use paraxis::maths::la::vector::Vector;
use macroquad::prelude::*;
use macroquad::time;
use paraxis::dstruct::kdtree::KDTree;

#[macroquad::main("KD-Tree Nearest Neighbor")]
async fn main() {
    rand::srand(time::get_time() as _);
    let mut points = Vec::new();
    for _ in 0..10000 {
        let px = rand::gen_range(50.0, screen_width() - 50.0);
        let py = rand::gen_range(50.0, screen_height() * 1.5 - 50.0);
        points.push((Vector::from([px, py]), ()));
    }
    let mut tree = KDTree::new_empty();
    tree.build(points);
    let mut last_click: Option<(f32, f32)> = None;
    loop {
        let mut nearest_pts: Vec<Option<(f32, f32)>> = Vec::new();
        clear_background(BLACK);
        for pt in &tree.points {
            draw_circle(pt.inner[0], pt.inner[1], 4.0, GREEN);
        }
        if is_mouse_button_down(MouseButton::Left) {
            let (mx, my) = mouse_position();
            last_click = Some((mx, my));
            let query = Vector::from([mx, my]);
            let pts = tree.nearest_neighbour(query, 100);
            for pt in pts {
                nearest_pts.push(Some((pt.0.inner[0], pt.0.inner[1])));
            }
        }
        if let Some((mx, my)) = last_click {
            for pt in nearest_pts.iter() {
                if let Some((nx, ny)) = pt {
                    draw_line(mx, my, *nx, *ny, 2.0, RED.with_alpha(0.5));
                    draw_circle(*nx, *ny, 3.0, RED);
                }
            }
        }
        draw_fps();
        next_frame().await;
    }
}
