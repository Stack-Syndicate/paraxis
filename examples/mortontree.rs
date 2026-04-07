use macroquad::prelude::*;
use paraxis::dsa::tree::mt::MortonTree;
use paraxis::maths::vec::Vector;

#[macroquad::main("MortonTree Bucketing Visualizer")]
async fn main() {
    let mut raw_points: Vec<(Vector<f32, 2>, ())> = Vec::new();
    let bucket_scale = 1. / 64.;
    let cell_size = 1.0 / bucket_scale;
    loop {
        let width = screen_width();
        let height = screen_height();
        clear_background(BLACK);
        let new_point = Vector::new([rand::gen_range(0.0, width), rand::gen_range(0.0, height)]);
        raw_points.push((new_point, ()));
        let tree = MortonTree::new(
            raw_points.clone(),
            Vector::new([0.0, 0.0]),
            bucket_scale,
            64,
        );
        for x in (0..width as i32).step_by(cell_size as usize) {
            draw_line(
                x as f32 + cell_size / 2.,
                0.0,
                x as f32 + cell_size / 2.,
                height,
                1.0,
                Color::from_rgba(40, 40, 40, 255),
            );
        }
        for y in (0..height as i32).step_by(cell_size as usize) {
            draw_line(
                0.0,
                y as f32 + cell_size / 2.,
                width,
                y as f32 + cell_size / 2.,
                1.0,
                Color::from_rgba(40, 40, 40, 255),
            );
        }

        let m = mouse_position();
        let mouse_vec = Vector::new([m.0, m.1]);
        let bucket = tree.search_bucket(&mouse_vec);
        let cell_x = ((m.0 + cell_size / 2.) / cell_size).floor() * cell_size;
        let cell_y = ((m.1 + cell_size / 2.) / cell_size).floor() * cell_size;
        draw_rectangle(
            cell_x - cell_size / 2.,
            cell_y - cell_size / 2.,
            cell_size,
            cell_size,
            Color::from_rgba(0, 255, 0, 50),
        );
        for (_, pos, _) in bucket {
            draw_line(
                m.0,
                m.1,
                pos.inner[0],
                pos.inner[1],
                1.5,
                GREEN.with_alpha(0.25),
            );
            draw_circle(pos.inner[0], pos.inner[1], 4.0, GREEN);
        }
        for (_, p, _) in &tree.data {
            draw_circle(p.inner[0], p.inner[1], 2.0, WHITE);
        }
        draw_text(
            "Morton Bucketing (Linear Search per Cell)",
            20.0,
            30.0,
            25.0,
            WHITE,
        );
        draw_text(
            &format!("Scale: {} (Cell Size: {}px)", bucket_scale, cell_size),
            20.0,
            60.0,
            20.0,
            GRAY,
        );
        draw_text(
            &format!("Points in current bucket: {}", bucket.len()),
            20.0,
            85.0,
            20.0,
            GREEN,
        );
        draw_text(
            &format!("Total Points: {}", tree.data.len()),
            20.0,
            110.0,
            20.0,
            BLUE,
        );
        draw_fps();
        next_frame().await;
    }
}
