use paraxis::containers::tree::BIHierarchy;
use paraxis::prelude::Tree;
use raylib::prelude::*;

fn main() {
    let (mut rl, thread) = raylib::init()
        .size(1280, 720)
        .title("BIH Raytracing Demo")
        .build();
    rl.set_target_fps(60);
    let mut camera = Camera3D::perspective(
        Vector3::new(12.0, 12.0, 12.0),
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        45.0,
    );
    let mut raw_voxels = Vec::new();
    let radius = 6;
    for x in -radius..=radius {
        for y in -radius..=radius {
            for z in -radius..=radius {
                let dist_sq = x * x + y * y + z * z;
                if dist_sq <= radius * radius {
                    let color = if dist_sq < (radius - 1) * (radius - 1) {
                        Color::BROWN
                    } else {
                        Color::DARKGREEN
                    };
                    raw_voxels.push(([x as f32, y as f32, z as f32], color));
                }
            }
        }
    }
    let bih = BIHierarchy::new(raw_voxels);
    let voxel_size = 1.0;
    rl.disable_cursor();
    while !rl.window_should_close() {
        camera.update_camera(CameraMode::CAMERA_FIRST_PERSON);
        let origin = [camera.position.x, camera.position.y, camera.position.z];
        let direction = (camera.target - camera.position).normalize();
        let ray_hit = bih.trace_ray(
            origin,
            [direction.x, direction.y, direction.z],
            0.0,
            50.0,
            voxel_size,
        );
        let mut d = rl.begin_drawing(&thread);
        d.clear_background(Color::BLACK);
        {
            let mut d3d = d.begin_mode3D(camera);
            for node in &bih.data {
                if node.bounds.is_some() {
                    continue;
                }
                if let Some(color) = &node.read().inner {
                    let pos = Vector3::new(node.position[0], node.position[1], node.position[2]);
                    d3d.draw_cube(pos, voxel_size, voxel_size, voxel_size, *color);
                    d3d.draw_cube_wires(pos, voxel_size, voxel_size, voxel_size, Color::BLACK);
                }
            }
            if let Some((_, hit_node)) = ray_hit {
                let pos = Vector3::new(
                    hit_node.position[0],
                    hit_node.position[1],
                    hit_node.position[2],
                );
                d3d.draw_cube(
                    pos,
                    voxel_size + 0.01,
                    voxel_size + 0.01,
                    voxel_size + 0.01,
                    Color::RED,
                );
            }
        }
        d.draw_fps(10, 10);
        d.draw_text(
            "W, A, S, D to move, Mouse to look.",
            10,
            30,
            20,
            Color::DARKGRAY,
        );
    }
}
