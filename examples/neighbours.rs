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
        let x = rl.get_random_value::<i32>(20..=(screen_width - 20));
        let y = rl.get_random_value::<i32>(20..=(screen_height - 20));
        raw_data.push(([x, y], ()));
        render_points.push([x, y]);
    }
    let tree: KDTree<[i32; 2], ()> = KDTree::new(raw_data);
    rl.set_target_fps(144);
    while !rl.window_should_close() {
        let mut d = rl.begin_drawing(&thread);
        d.clear_background(Color::BLACK);
        for pt in &render_points {
            d.draw_circle(pt[0], pt[1], 2.0, Color::DARKBLUE);
        }
        let mouse_pos = [d.get_mouse_x(), d.get_mouse_y()];
        if let Ok(nearest) = tree.nearest_neighbour(&mouse_pos) {
            let target = nearest.position;
            d.draw_line(mouse_pos[0], mouse_pos[1], target[0], target[1], Color::RED);
            d.draw_circle(target[0], target[1], 4.0, Color::RED);
        }
        d.draw_circle(mouse_pos[0], mouse_pos[1], 4.0, Color::DARKGRAY);
        d.draw_fps(12, 12);
    }
}
