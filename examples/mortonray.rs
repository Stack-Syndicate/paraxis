use std::f32::consts::PI;

use macroquad::prelude::*;
use paraxis::dsa::tree::mt::MortonTree;
use paraxis::maths::vec::Vector;
use rayon::iter::IntoParallelRefIterator;
use rayon::iter::ParallelIterator;

#[macroquad::main("MortonTree Raycasting Visualizer")]
async fn main() {
    let maze = [
        "1111111111",
        "1000010001",
        "1011010101",
        "1001000101",
        "1101001101",
        "1000000001",
        "1111111111",
    ];

    let tile_size = 32.0;
    let mut raw_points: Vec<(Vector<f32, 2>, ())> = Vec::new();
    for (row_index, row) in maze.iter().enumerate() {
        for (col_index, char) in row.chars().enumerate() {
            if char == '1' {
                let x_offset = col_index as f32 * tile_size;
                let y_offset = row_index as f32 * tile_size;
                for i in (0..=tile_size as i32).step_by(8) {
                    let offset = i as f32;
                    raw_points.push((Vector::new([x_offset + offset, y_offset]), ()));
                    raw_points.push((Vector::new([x_offset + offset, y_offset + tile_size]), ()));
                    raw_points.push((Vector::new([x_offset, y_offset + offset]), ()));
                    raw_points.push((Vector::new([x_offset + tile_size, y_offset + offset]), ()));
                }
            }
        }
    }
    let tree = MortonTree::new(raw_points.clone(), Vector::new([-10000.0, -10000.0]));
    let render_target = render_target(1920, 1080);
    let raycast_depth = 3;
    let mut tree_needs_update = true;
    let raycast_coarseness = 16;
    let mut fov = PI / 2.0;
    let mut player_position = Vector::new([64.0, 64.0]);
    let mut player_angle = 0.0f32;
    let move_speed = 0.5;
    let rotation_speed = 0.01;
    loop {
        let forward = Vector::new([player_angle.cos(), player_angle.sin()]);
        let right = Vector::new([-player_angle.sin(), player_angle.cos()]);
        clear_background(BLACK);
        if is_key_down(KeyCode::W) {
            player_position += forward * move_speed;
        }
        if is_key_down(KeyCode::S) {
            player_position -= forward * move_speed;
        }
        if is_key_down(KeyCode::A) {
            player_position -= right * move_speed;
        }
        if is_key_down(KeyCode::D) {
            player_position += right * move_speed;
        }
        if is_key_down(KeyCode::Q) {
            player_angle -= rotation_speed;
        }
        if is_key_down(KeyCode::E) {
            player_angle += rotation_speed;
        }
        if is_key_down(KeyCode::Z) {
            fov -= PI / 128.0;
        }
        if is_key_down(KeyCode::X) {
            fov += PI / 128.0;
        }
        if tree_needs_update {
            set_camera(&Camera2D {
                render_target: Some(render_target.clone()),
                zoom: vec2(2.0 / 1920.0, 2.0 / 1080.0),
                target: vec2(1920.0 / 2.0, 1080.0 / 2.0),
                ..Default::default()
            });
            clear_background(BLANK);
            for (mc, _) in &tree.data {
                draw_rectangle(
                    mc.position.inner[0],
                    mc.position.inner[1],
                    8.0,
                    8.0,
                    DARKGRAY,
                );
            }
            set_default_camera();
            tree_needs_update = false;
        }

        let mut ray_dirs = Vec::new();
        let angle_step = fov / (1920 / raycast_coarseness) as f32;
        let num_rays = 1920 / raycast_coarseness;
        let start_angle = player_angle - fov / 2.0;
        for i in 0..num_rays {
            let current_angle = start_angle + (i as f32 * angle_step);
            ray_dirs.push(Vector::new([current_angle.cos(), current_angle.sin()]));
        }
        let all_hit_results: Vec<_> = ray_dirs
            .par_iter()
            .map(|ray_dir| {
                let max_ray_dist = 500.0;
                let hit_buckets =
                    tree.raycast(player_position, *ray_dir, max_ray_dist, raycast_depth);
                (ray_dir, hit_buckets)
            })
            .collect();
        let mut first_hits = Vec::new();
        for (_, hit_buckets) in all_hit_results.iter() {
            for (i, (dist, bucket)) in hit_buckets.iter().enumerate() {
                if i == 0 {
                    first_hits.push(dist);
                }
            }
        }
        let total_rays = 1920 / raycast_coarseness;
        let middle_index = total_rays as f32 / 2.0;
        for (i, (x, dist)) in (0..1920)
            .step_by((1920.0 / num_rays as f32) as usize)
            .zip(first_hits)
            .enumerate()
        {
            let relative_column = i as f32 - middle_index;
            let angle_diff = relative_column * angle_step;
            let corrected_dist = *dist * angle_diff.cos();
            let mut wall_height = 1080.0 / corrected_dist * 32.0;
            let y_pos = (1080.0 - wall_height) / 2.0;
            if *dist == -1.0 {
                wall_height = 0.0;
            }
            draw_rectangle(
                x as f32,
                y_pos,
                1920.0 / total_rays as f32,
                wall_height,
                WHITE.with_alpha(wall_height / 1080.0),
            );
        }

        draw_rectangle(0.0, 0.0, 350.0, 250.0, BLACK);
        draw_texture(&render_target.texture, 0.0, 0.0, WHITE);
        draw_circle(
            player_position.inner[0],
            player_position.inner[1],
            8.0,
            GREEN,
        );
        draw_line(
            player_position.inner[0],
            player_position.inner[1],
            player_position.inner[0] + f32::cos(player_angle + fov / 2.0) * 128.0,
            player_position.inner[1] + f32::sin(player_angle + fov / 2.0) * 128.0,
            2.0,
            GREEN,
        );
        draw_line(
            player_position.inner[0],
            player_position.inner[1],
            player_position.inner[0] + f32::cos(player_angle - fov / 2.0) * 128.0,
            player_position.inner[1] + f32::sin(player_angle - fov / 2.0) * 128.0,
            2.0,
            GREEN,
        );
        draw_fps();
        next_frame().await;
    }
}
