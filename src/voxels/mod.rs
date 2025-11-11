pub mod morton;
use core::f32;

use glam::{UVec3, Vec3};

use crate::voxels::morton::{Morton, MortonCode};

fn ray_aabb_intersection(
    ray_origin: Vec3,
    ray_direction: Vec3,
    min_aabb: Vec3,
    max_aabb: Vec3,
) -> Option<(f32, Vec3)> {
	let inverse_direction = Vec3::new(1.0 / ray_direction.x, 1.0 / ray_direction.y, 1.0 / ray_direction.z);
	
	let t1 = (min_aabb - ray_origin) * inverse_direction;
	let t2 = (max_aabb - ray_origin) * inverse_direction;

	let tmin_v = t1.min(t2);
	let tmax_v = t1.max(t2);

	let tmin = tmin_v.x.max(tmin_v.y).max(tmin_v.z);
	let tmax = tmax_v.x.min(tmax_v.y).min(tmax_v.z);

	if tmax >= tmin.max(0.0) {
		let t_hit = tmin.max(0.0);
		Some((t_hit, ray_origin + ray_direction * t_hit))	
	} else {
		None
	}
}

#[derive(Clone, Copy, Debug)]
pub struct Voxel {
    children: [Option<usize>; 8],
    material: u32,
}
impl Voxel {
    pub fn empty() -> Self {
        Self {
            children: [None; 8],
            material: 0,
        }
    }
    pub fn is_empty(&self) -> bool {
        self.children.iter().all(|c| c.is_none())
    }
}

#[derive(Debug)]
pub struct SparseVoxelOctree {
    voxels: Vec<Voxel>,
    pub size: u32,
    origin: UVec3,
}
impl SparseVoxelOctree {
    pub fn empty(size: u32, origin_x: u32, origin_y: u32, origin_z: u32) -> Self {
        Self {
            voxels: vec![Voxel {
                children: [None; 8],
                material: u32::MAX,
            }],
            size,
            origin: UVec3::new(origin_x, origin_y, origin_z),
        }
    }
    pub fn insert(&mut self, x: u32, y: u32, z: u32, material: u32) {
        let code = MortonCode::encode(x, y, z);
        let depth = self.size.trailing_zeros();
        let mut voxel_index = 0;
        for level in (0..depth).rev() {
            let child_index = ((code >> (level * 3)) & 0b111) as usize;
            if self.voxels[voxel_index].children[child_index].is_none() {
                let new_index = self.voxels.len();
                self.voxels[voxel_index].children[child_index] = Some(new_index);
                self.voxels.push(Voxel::empty());
            }
            voxel_index = self.voxels[voxel_index].children[child_index].unwrap();
        }
        self.voxels[voxel_index].material = material;
    }
    pub fn remove(&mut self, x: u32, y: u32, z: u32) {
        let code = MortonCode::encode(x, y, z);
        let depth = self.size.trailing_zeros();
        let mut voxel_index = 0;
        let mut parent_stack = Vec::with_capacity(depth as usize);
        for level in (0..depth).rev() {
            let child_index = ((code >> (level * 3)) & 0b111) as usize;
            match self.voxels[voxel_index].children[child_index] {
                Some(next) => {
                    parent_stack.push((voxel_index, child_index));
                    voxel_index = next;
                }
                None => return,
            }
        }
        self.voxels[voxel_index].material = 0;
        while let Some((parent_idx, child_idx)) = parent_stack.pop() {
            if self.voxels[voxel_index].is_empty() && self.voxels[voxel_index].material == 0 {
                self.voxels[parent_idx].children[child_idx] = None;
            } else {
                break;
            }
            voxel_index = parent_idx;
        }
    }
    pub fn raycast(&self, ray_origin: Vec3, ray_direction: Vec3) -> Option<(Voxel, Vec3)> {
        if self.voxels.is_empty() { return None; }

		let root_min = Vec3::new(self.origin.x as f32, self.origin.y as f32, self.origin.z as f32);
		let root_max = root_min + Vec3::splat(self.size as f32);
		
		let (entry_distance, _) = ray_aabb_intersection(ray_origin, ray_direction, root_min, root_max)?;
		
		let mut stack = vec![(0usize, root_min, self.size as f32, entry_distance)];
		let mut child_hits = Vec::with_capacity(8);
		while let Some((voxel_index, voxel_min, voxel_size, voxel_entry)) = stack.pop() {
			let voxel = &self.voxels[voxel_index];
			if voxel.is_empty() {
				let hit_position = ray_origin + ray_direction * voxel_entry;
				return Some((*voxel, hit_position));
			}
			let half = voxel_size / 2.0;
			child_hits.clear();
			for child in 0..8 {
				if let Some(voxel_child_index) = voxel.children[child] {
					let bx = (child & 1) as f32;
					let by = ((child >> 1) & 1) as f32;
					let bz = ((child >> 2) & 1) as f32;
					let child_min = voxel_min + Vec3::new(bx * half, by * half, bz * half);
					let child_max = child_min + Vec3::splat(half);
					if let Some((voxel_child_entry_distance, _)) = ray_aabb_intersection(ray_origin, ray_direction, child_min, child_max) {
						if voxel_child_entry_distance <= f32::MAX {
							child_hits.push((voxel_child_index, child_min, half, voxel_child_entry_distance));
						}
					}
				}
			}
			child_hits.sort_by(|a, b| {
				a.3.partial_cmp(&b.3).unwrap()
			});
			stack.extend(child_hits.iter().rev().cloned());
		}

        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stress_insert_remove() {
        let size = 64;
        let mut svo = SparseVoxelOctree::empty(size, 0, 0, 0);
        for x in 0..size {
            for y in 0..size {
                for z in 0..size {
                    svo.insert(x, y, z, 1);
                }
            }
        }

        let total_voxels = svo.voxels.len();
        assert!(total_voxels > 1, "Tree should have grown after insertions");
        for x in 0..size {
            for y in 0..size {
                for z in 0..size {
                    svo.remove(x, y, z);
                }
            }
        }
        assert_eq!(
            svo.voxels.len(),
            total_voxels,
            "Vec should not shrink, but structure should be pruned"
        );
        assert!(
            svo.voxels[0].is_empty(),
            "Root should be empty after removals"
        );
    }
}
