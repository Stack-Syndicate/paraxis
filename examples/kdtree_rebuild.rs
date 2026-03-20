use macroquad::prelude::*;
use paraxis::dstruct::kdtree::KDTree;
use paraxis::maths::la::vector::Vector;

struct Entity {
    pos: Vector<f32, 2>,
    vel: Vector<f32, 2>,
    id: usize,
}

#[macroquad::main("KD-Tree Build Stress Test")]
async fn main() {
    let mut entities: Vec<Entity> = Vec::new();
    let mut tree = KDTree::<f32, usize, 2>::new_empty();
    loop {
        clear_background(BLACK);
        if is_mouse_button_down(MouseButton::Left) {
            for _ in 0..1000 {
                let (x, y) = mouse_position();
                entities.push(Entity {
                    pos: Vector::from([x, y]),
                    vel: Vector::from([rand::gen_range(-2.0, 2.0), rand::gen_range(-2.0, 2.0)]),
                    id: entities.len(),
                });
            }
        }
        for e in entities.iter_mut() {
            e.pos.inner[0] += e.vel.inner[0];
            e.pos.inner[1] += e.vel.inner[1];
            if e.pos.inner[0] < 0.0 || e.pos.inner[0] > screen_width() { e.vel.inner[0] *= -1.0; }
            if e.pos.inner[1] < 0.0 || e.pos.inner[1] > screen_height() { e.vel.inner[1] *= -1.0; }
        }
        if !entities.is_empty() {
            tree.points.clear();
            tree.data.clear();
            tree.nodes.clear();
            tree.root = None;
            let build_data: Vec<(Vector<f32, 2>, usize)> = entities
                .iter()
                .map(|e| (e.pos, e.id))
                .collect();
            let start = get_time();
            tree.build(build_data);
            let elapsed = get_time() - start;
            if let Some(root_idx) = tree.root {
                draw_node(&tree, root_idx, [0.0, 0.0], [screen_width(), screen_height()]);
            }
            draw_text(&format!("Points: {}", entities.len()), 20.0, 30.0, 30.0, WHITE);
            draw_text(&format!("Build Time: {:.2}ms", elapsed * 1000.0), 20.0, 60.0, 30.0, GREEN);
        }
        draw_text("Hold Left Click to spawn points", 20.0, screen_height() - 20.0, 20.0, GRAY);
        draw_fps();
        next_frame().await
    }
}

fn draw_node(tree: &KDTree<f32, usize, 2>, node_idx: usize, min: [f32; 2], max: [f32; 2]) {
    let node = &tree.nodes[node_idx];
    let pos = tree.points[node_idx].inner;
    let (color, line_start, line_end) = if node.axis == 0 {
        (Color::new(0.0, 0.5, 1.0, 0.5), vec2(pos[0], min[1]), vec2(pos[0], max[1]))
    } else {
        (Color::new(1.0, 0.2, 0.2, 0.5), vec2(min[0], pos[1]), vec2(max[0], pos[1]))
    };
    draw_line(line_start.x, line_start.y, line_end.x, line_end.y, 1.0, color.with_alpha(0.5));
    // draw_circle(pos[0], pos[1], 2.0, YELLOW.with_alpha(0.75));
    if let Some(left) = node.left {
        let new_max = if node.axis == 0 { [pos[0], max[1]] } else { [max[0], pos[1]] };
        draw_node(tree, left, min, new_max);
    }
    if let Some(right) = node.right {
        let new_min = if node.axis == 0 { [pos[0], min[1]] } else { [min[0], pos[1]] };
        draw_node(tree, right, new_min, max);
    }
}
