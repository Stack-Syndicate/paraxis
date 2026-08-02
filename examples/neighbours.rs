use raylib::prelude::*;

use paraxis::common::traits::Tree;
use paraxis::containers::tree::KDTree;

fn main() {
    let screen_width = 800;
    let screen_height = 600;
    let (mut rl, thread) = raylib::init()
        .size(screen_width, screen_height)
        .title("KDTree Nearest Neighbor Search")
        .build();
    let num_points = 10000;
    let mut raw_data = Vec::with_capacity(num_points);
    let mut render_points = Vec::with_capacity(num_points);
    for _ in 0..num_points {
        let x = rl.get_random_value::<i32>(20..=(screen_width - 20)) as f32;
        let y = rl.get_random_value::<i32>(20..=(screen_height - 20)) as f32;
        raw_data.push(([x, y], ()));
        render_points.push([x, y]);
    }
    let tree = KDTree::new(raw_data);
    rl.set_target_fps(144);
    while !rl.window_should_close() {
        let mut d = rl.begin_drawing(&thread);
        d.clear_background(Color::BLACK);
        for pt in &render_points {
            d.draw_circle(pt[0] as i32, pt[1] as i32, 2.0, Color::DARKBLUE);
        }
        let mouse_pos = [d.get_mouse_x(), d.get_mouse_y()];
        if let Ok(nearest) = tree.k_nearest_neighbours(&mouse_pos.map(|v| v as f32), 10) {
            for target in nearest {
                let target_position = target.position;
                d.draw_line(
                    mouse_pos[0],
                    mouse_pos[1],
                    target_position[0] as i32,
                    target_position[1] as i32,
                    Color::RED,
                );
                d.draw_circle(
                    target_position[0] as i32,
                    target_position[1] as i32,
                    4.0,
                    Color::RED,
                );
            }
        }
        d.draw_circle(mouse_pos[0], mouse_pos[1], 4.0, Color::DARKGRAY);
        d.draw_fps(12, 12);
    }
}
