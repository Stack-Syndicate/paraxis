use ::glam::Vec3;
use macroquad::{prelude::*, rand::rand};

use paraxis::voxels::SparseVoxelOctree; // replace with your crate path

#[macroquad::main("Octree Raycast Test")]
async fn main() {
    let screen_width = 800;
    let screen_height = 800;
	let fov = 90.0;
	let aspect_ratio = screen_width / screen_height;

    let mut octree = SparseVoxelOctree::empty(32, 0, 0, 0);

	for x in 0..32 {
		for y in 0..32 {
			for z in 0..32 {
				if rand() < u32::MAX/100 {
					octree.insert(x, y, z, 0);
				}
			}
		}
	}

    let camera_position = Vec3::new(16.0, 8.0, -16.0);
	
    loop {
        let mut rgba_data = vec![0u8; (screen_width * screen_height * 4) as usize];
        for i in 0..screen_width {
            for j in 0..screen_height {
                let ray_origin = camera_position;

				let ndc_x = (2.0 * i as f32 / screen_width as f32 - 1.0) * aspect_ratio as f32 * f32::tan(fov / 2.0);
				let ndc_y = (1.0 - 2.0 * j as f32 / screen_height as f32) * f32::tan(fov / 2.0);

				let ray_direction = Vec3::new(ndc_x, ndc_y, 1.0).normalize();

                let color = if let Some((_voxel, _hit_position)) =
                    octree.raycast(ray_origin, ray_direction)
                {
                    (255, 0, 0, 255)
                } else {
                    (0, 0, 0, 255)
                };
                let idx = ((j * screen_width + i) * 4) as usize;
                rgba_data[idx] = color.0;
                rgba_data[idx + 1] = color.1;
                rgba_data[idx + 2] = color.2;
                rgba_data[idx + 3] = color.3;
            }
        }
        let texture = Texture2D::from_rgba8(screen_width as u16, screen_height as u16, &rgba_data);
        texture.set_filter(FilterMode::Nearest);
        clear_background(BLACK);
        draw_texture_ex(
            &texture,
            0.0,
            0.0,
            WHITE,
            DrawTextureParams {
                dest_size: Some(vec2(screen_width as f32, screen_height as f32)),
                ..Default::default()
            },
        );
        next_frame().await
    }
}
