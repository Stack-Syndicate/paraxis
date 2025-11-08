use crossbeam::queue::SegQueue;
use glam::U16Vec3;
use std::collections::BTreeSet;
use std::fmt::Debug;
use std::collections::BTreeMap;

#[derive(Debug, Clone, Copy)]
enum MutationOp<T> {
    Insert { position: U16Vec3, value: T },
    Remove { position: U16Vec3 },
}

pub struct VoxelData<T: Send + Sync + Clone + Copy + Debug + PartialEq> {
	position: U16Vec3,
	compression: u8,
	payload: T
}

#[derive(Debug)]
pub struct ZVoxelOctree<T: Send + Sync + Clone + Copy + Debug + PartialEq> {
    data: BTreeMap<u64, T>,
    mutation_queue: SegQueue<MutationOp<T>>
}
impl<T: Send + Sync + Clone + Copy + Debug + PartialEq> ZVoxelOctree<T> {
    pub fn new() -> Self {
        Self {
            data: BTreeMap::new(),
            mutation_queue: SegQueue::new()
        }
    }
    pub fn encode_position(&self, coords: U16Vec3) -> u64 {
        let coords = coords.to_array();
        let mut morton_code: u64 = 0;
        for b in 0..16 {
            for (i, d) in coords.iter().enumerate() {
                let coord_bit: u64 = ((d >> b) & 1) as u64;
                morton_code |= (coord_bit << (b * 3 + i + 16)) as u64;
            }
        }
        return morton_code;
    }
    pub fn decode_position(&self, code: u64) -> U16Vec3 {
        let mut coords = U16Vec3::new(0, 0, 0);
        for b in 0..16 {
            for i in 0..3 {
                let bit = ((code >> (b * 3 + i) + 16) & 1) as u16;
                coords[i] |= bit << b;
            }
        }
        coords
    }
    pub fn encode_compression(&self, coords: U16Vec3, compression: u8) -> u64 {
		assert!(compression < 16);
		let mut code = self.encode_position(coords);
		code &= !0b1111;
		code |= compression as u64;
		code
	}
	pub fn decode_compression(&self, code: u64) -> u8 {
		(code & 0b1111) as u8
	}
	fn prefix_at_depth(&self, code: u64, depth: usize) -> u64 {
        let spatial_bits = 48;
        let prefix_bits_to_keep = spatial_bits - depth * 3;
        (code >> 16) >> (spatial_bits - prefix_bits_to_keep)
    }
	pub fn insert(&self, entity: T, coords: U16Vec3) {
        self.mutation_queue.push(MutationOp::Insert {
            position: coords,
            value: entity,
        })
    }
    pub fn get(&self, coords: U16Vec3) -> Option<VoxelData<T>> {
		let coords_key = self.encode_position(coords);
		let spatial_key = coords_key & !0xFFFFu64;
		let key_val_opt = self.data.range(spatial_key..=spatial_key | 0xFFFF)
			.next();
		if let Some((key, val)) = key_val_opt {
			let payload = val.clone();
			let position = coords;
			let compression = self.decode_compression(*key);
			return Some(VoxelData { position, compression, payload })
		}
		return None;
    }
    pub fn get_neighbours_cross(&self, coords: U16Vec3) -> Vec<Option<VoxelData<T>>> {
        let mut neighbours = Vec::with_capacity(6);
        for &delta in &[
            (0, 1, 0),  // up
            (0, -1, 0), // down
            (-1, 0, 0), // left
			(1, 0, 0),  // right
            (0, 0, 1),  // forward
            (0, 0, -1), // back
        ] {
            let mut n_coords = coords;
            n_coords[0] = (n_coords[0] as i32 + delta.0) as u16;
            n_coords[1] = (n_coords[1] as i32 + delta.1) as u16;
            n_coords[2] = (n_coords[2] as i32 + delta.2) as u16;
            neighbours.push(self.get(n_coords));
        }
        neighbours
    }
    pub fn get_neighbours_area(&self, coords: U16Vec3, radius: usize) -> Vec<VoxelData<T>> {
        let depth = radius.next_power_of_two().ilog2() as usize;
        let prefix_neighbours = self.get_neighbours_prefix(coords, depth);
        let neighbours = prefix_neighbours
            .into_iter()
            .filter_map(|voxel| {
                if coords.chebyshev_distance(voxel.position) <= radius as u16 {
                    Some(voxel)
                } else {
                    None
                }
            })
            .collect();
        return neighbours;
    }
    pub fn get_neighbours_prefix(&self, coords: U16Vec3, depth: usize) -> Vec<VoxelData<T>> {
        let code = self.encode_position(coords);
        let prefix = code >> (16 + depth * 3);
        let start_code = prefix << (16 + depth * 3);
        let end_code = ((prefix + 1) << (16 + depth * 3)) - 1;
        let mut result = Vec::new();
        for (&key, &value) in self.data.range(start_code..=end_code) {
            result.push(VoxelData { 
				position: self.decode_position(key), 
				compression: self.decode_compression(key), 
				payload: value });
        }
        return result;
    }
    fn compress_level(&mut self, depth: u8) {
        let mut seen_prefixes = BTreeSet::<u64>::new();
        let keys: Vec<u64> = self.data.keys().copied().collect();

        for code in keys {
            let prefix = self.prefix_at_depth(code, depth as usize);
            if seen_prefixes.contains(&prefix) {
                continue;
            }

            let pos = self.decode_position(code);
            let neighbours = self.get_neighbours_prefix(pos, depth as usize);
            if neighbours.len() < 2_usize.pow(3*depth as u32) {
                continue;
            }
            let first_value = neighbours[0].payload;
            if neighbours.iter().all(|voxel| voxel.payload == first_value) {
                for neighbour in &neighbours {
                    self.data.remove(&self.encode_position(neighbour.position));
                }

                let compressed_code = (prefix << (64 - (48 - depth as usize * 3))) | (depth as u64 & 0xF);
                self.data.insert(compressed_code, first_value);
            }
            seen_prefixes.insert(prefix);
        }
    }
	pub fn compress(&mut self, depth: u8) {
		assert!(depth < 16);
		for i in 1..=depth {
			self.apply_mutations();
			self.compress_level(i);
			self.apply_mutations();
		}
	}
    pub fn apply_mutations(&mut self) {
		while let Some(op) = self.mutation_queue.pop() {
			match op {
				MutationOp::Insert { position, value } => {
					self.data.insert(self.encode_position(position), value);
				}
				MutationOp::Remove { position } => {
					let removed = self.data.remove(&self.encode_position(position));
					if removed.is_some() { continue; }
					for depth in 1..16 {
						let prefix = self.get_neighbours_prefix(position, depth);
						if prefix.is_empty() { continue; }
						if prefix.len() == 1 {
							let compressed_voxel = &prefix[0];
							let code = self.encode_compression(compressed_voxel.position, compressed_voxel.compression);
							self.data.remove(&code);
						}
					}
				}
			}
		}
    }
    pub fn remove(&self, coords: U16Vec3) {
        self.mutation_queue.push(MutationOp::Remove {
            position: coords,
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{sync::{Arc, RwLock}, thread};

    #[test]
    fn insert_and_get() {
        let mut octree = ZVoxelOctree::new();
        octree.insert("hello", [1, 2, 3].into());
        octree.apply_mutations();
        let value1 = octree.get([1, 2, 3].into());
        let value2 = octree.get([0, 0, 0].into());
        assert_eq!(value1.unwrap().payload, "hello");
        assert!(value2.is_none());
    }

    #[test]
    fn remove() {
        let mut octree = ZVoxelOctree::new();
        octree.insert(42, [1, 1, 1].into());
        octree.apply_mutations();
        assert_eq!(octree.get([1, 1, 1].into()).unwrap().payload, 42);
        octree.remove([1, 1, 1].into());
        octree.apply_mutations();
        assert!(octree.get([1, 1, 1].into()).is_none());
    }

    #[test]
    fn prefix() {
        let mut octree = ZVoxelOctree::new();
        octree.insert(true, [0, 0, 0].into());
        octree.insert(true, [2, 0, 0].into());
        octree.apply_mutations();
        assert!(octree.get_neighbours_prefix([0, 0, 0].into(), 1).len() == 1);
        assert!(octree.get_neighbours_prefix([0, 0, 0].into(), 2).len() == 2);
    }

    #[test]
    fn concurrent_remove() {
        let octree = Arc::new(ZVoxelOctree::<u32>::new());
        for i in 0..50 {
            let coords = U16Vec3::new(i % 5, (i / 5) % 5, i / 25);
            octree.insert(i.into(), coords);
        }
        let octree_ref = octree.clone();
        let handle = thread::spawn(move || {
            for i in 0..50 {
                let coords = U16Vec3::new(i % 5, (i / 5) % 5, i / 25);
                octree_ref.remove(coords);
            }
        });
        handle.join().unwrap();
        let mut octree_owned = Arc::try_unwrap(octree).expect("Only one strong reference left");
        octree_owned.apply_mutations();
		
        for i in 0..50 {
            let coords = U16Vec3::new(i % 5, (i / 5) % 5, i / 25);
            assert!(octree_owned.get(coords).is_none());
        }
    }

	#[test]
    fn concurrent_insertions() {
        let octree = Arc::new(RwLock::new(ZVoxelOctree::<u32>::new()));
        let mut handles = vec![];
        for i in 0..100 {
            let octree_clone = octree.clone();
            let coords = U16Vec3::new((i % 5) as u16, (i / 5 % 5) as u16, (i / 25) as u16);
            let thread_handle = thread::spawn(move || {
                let octree = octree_clone.write().expect("Could not get write lock");
                octree.insert(i, coords);
            });
            handles.push(thread_handle);
        }
        for handle in handles {
            handle.join().unwrap();
        }
		let mut octree_write_lock = octree.write().expect("Could not get write lock");
		octree_write_lock.apply_mutations();
		drop(octree_write_lock);
		let octree_read_lock = octree.read().expect("Read lock error.");
        // Check that all values have been inserted correctly
        for i in 0..100 {
            let coords = U16Vec3::new((i % 5) as u16, (i / 5 % 5) as u16, (i / 25) as u16);
            let result = octree_read_lock.get(coords);
            assert!(result.is_some(), "Expected value for coords {:?} to be present", coords);
            assert_eq!(result.unwrap().payload, i, "Value mismatch for coords {:?}", coords);
        }
    }

	#[test]
	fn compression() {
		let mut octree = ZVoxelOctree::new();
		octree.insert(true, [0, 0, 1].into());
		octree.insert(true, [0, 1, 0].into());
		octree.insert(true, [0, 1, 1].into());
		octree.insert(true, [1, 0, 0].into());
		octree.insert(true, [1, 0, 1].into());
		octree.insert(true, [1, 1, 0].into());
		octree.insert(true, [1, 1, 1].into());
		octree.insert(true, [0, 0, 0].into());
		octree.apply_mutations();
		octree.compress(2);
		for (&code, value) in &octree.data {
			let coords = octree.decode_position(code);
			let level = octree.decode_compression(code);
			println!("coords: {:?}, value: {}, level: {}, binary: {:#06b}", coords, value, level, code);
		}
	}
}
