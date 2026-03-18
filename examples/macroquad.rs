use macroquad::prelude::*;
use paraxis::dstruct::kdtree::KDTree;
use paraxis::maths::la::vector::Vector;

#[macroquad::main("KD-Tree Visualization")]
async fn main() {
    let mut raw_data: Vec<(Vector<f32, 2>, usize)> = Vec::new();
    loop {
        clear_background(BLACK);
        draw_text("Click to slice the world", 20.0, 30.0, 25.0, RED);

        if is_mouse_button_down(MouseButton::Left) {
            let (x, y) = mouse_position();
            let id = raw_data.len();
            raw_data.push((Vector::from([x, y]), id));
        }
        if !raw_data.is_empty() {
            let mut tree = KDTree::new_empty();
            tree.build(raw_data.clone());
            if let Some(root_idx) = tree.root {
                draw_node(&tree, root_idx, [0.0, 0.0], [screen_width(), screen_height()]);
            }
        }
        next_frame().await
    }
}


fn draw_node(tree: &KDTree<f32, usize, 2>, node_idx: usize, min: [f32; 2], max: [f32; 2]) {
    let node = &tree.nodes[node_idx];
    let pos = tree.points[node.id].inner;
    let (color, line_start, line_end) = if node.axis == 0 {
        (BLUE, vec2(pos[0], min[1]), vec2(pos[0], max[1]))
    } else {
        (RED, vec2(min[0], pos[1]), vec2(max[0], pos[1]))
    };
    draw_line(line_start.x, line_start.y, line_end.x, line_end.y, 2.0, color);
    draw_circle(pos[0], pos[1], 4.0, YELLOW);

    if let Some(left) = node.left {
        let new_max = if node.axis == 0 { [pos[0], max[1]] } else { [max[0], pos[1]] };
        draw_node(tree, left, min, new_max);
    }
    if let Some(right) = node.right {
        let new_min = if node.axis == 0 { [pos[0], min[1]] } else { [min[0], pos[1]] };
        draw_node(tree, right, new_min, max);
    }
}
