use dashmap::{DashMap, mapref::one::{Ref, RefMut}};
use rayon::prelude::*;

pub struct ZOctree<T: Send + Sync> {
	dimensions: u32,
	data: DashMap<u128, T>,
}
impl<T: Send + Sync> ZOctree<T> {
	pub fn new(dimensions: u32) -> Self {
		Self {
			dimensions,
			data: DashMap::new()
		}
	}
	pub fn encode(&self, coords: &[u32]) -> u128 {
		let mut morton_code: u128 = 0;
		let dim = self.dimensions as usize;
		// For every bit index
		for b in 0..32 {
			// Enumerate through the coordinates
			for (i, d) in coords.iter().enumerate() {
				// Shift the coordinate binary right by b, then retrieve the right-most bit
				let coord_bit: u128 = ((d >> b) & 1) as u128;
				// Insert the coordinate's bit into the morton code, offset by the coordinate index
				morton_code |= (coord_bit << (b * dim + i)) as u128;
			}
		}
		return morton_code;
	}
	pub fn decode(&self, code: u128) -> Vec<u32> {
		let dim = self.dimensions as usize;
		let mut coords = vec![0u32; dim];
		for b in 0..32 {
			for i in 0..dim {
				let bit = ((code >> (b * dim + i)) & 1) as u32;
				coords[i] |= bit << b;
			}
		}
		coords
	}
	pub fn insert(&self, entity: T, coords: &[u32]) {
		self.data.insert(self.encode(coords), entity);
	}
	pub fn get(&self, coords: &[u32]) -> Option<Ref<'_, u128, T>> {
		self.data.get(&self.encode(coords))
	}
	pub fn get_mut(&self, coords: &[u32]) -> Option<RefMut<'_, u128, T>> {
		self.data.get_mut(&self.encode(coords))
	}
	pub fn get_at_depth(&self, coords: &[u32], level: u8) -> Vec<(u128, Ref<'_, u128, T>)> {
		let full_code = self.encode(coords);
		let shift = self.dimensions * (32 - level as u32);
		let prefix = full_code >> shift;
		let keys: Vec<u128> = self.data.iter().map(|entry| *entry.key()).collect();
		keys.par_iter()
			.filter_map(|&code| {
				if (code >> shift) == prefix {
					self.data.get(&code).map(|v| (code, v))
				} else {
					None
				}
			})
			.collect()
	}
	pub fn get_mut_at_depth(&self, coords: &[u32], level: u8) -> Vec<(u128, RefMut<'_, u128, T>)> {
		let full_code = self.encode(coords);
		let shift = self.dimensions * (32 - level as u32);
		let prefix = full_code >> shift;
		let keys: Vec<u128> = self.data.iter().map(|entry| *entry.key()).collect();
		keys.par_iter()
			.filter_map(|&code| {
				if (code >> shift) == prefix {
					self.data.get_mut(&code).map(|v| (code, v))
				} else {
					None
				}
			})
			.collect()
	}
	pub fn get_radius_box(&self, center: &[u32], radius: u32) -> Vec<(u128, Ref<'_, u128, T>)> {
		let min: Vec<u32> = center.iter().map(|&c| c.saturating_sub(radius)).collect();
		let max: Vec<u32> = center.iter().map(|&c| c + radius).collect();
		let candidate_codes: Vec<u128> = self.data
			.par_iter()
			.filter_map(|entry| {
				let code = *entry.key();
				let coords = self.decode(code);
				if coords.iter().zip(&min).all(|(&c, &mi)| c >= mi) &&
				coords.iter().zip(&max).all(|(&c, &ma)| c <= ma) {
					Some(code)
				} else {
					None
				}
			})
			.collect();
		candidate_codes.into_iter()
			.filter_map(|code| {
				self.data.get(&code).map(|v| (code, v))
			})
			.collect()
	}
	pub fn get_mut_radius_box(&self, center: &[u32], radius: u32) -> Vec<(u128, RefMut<'_, u128, T>)> {
		let min: Vec<u32> = center.iter().map(|&c| c.saturating_sub(radius)).collect();
		let max: Vec<u32> = center.iter().map(|&c| c + radius).collect();
		let candidate_codes: Vec<u128> = self.data
			.iter()
			.filter_map(|entry| {
				let code = *entry.key();
				let coords = self.decode(code);
				if coords.iter().zip(&min).all(|(&c, &mi)| c >= mi) &&
				coords.iter().zip(&max).all(|(&c, &ma)| c <= ma) {
					Some(code)
				} else {
					None
				}
			})
			.collect();
		candidate_codes.into_iter()
			.filter_map(|code| {
				self.data.get_mut(&code).map(|v| (code, v))
			})
			.collect()
	}
	pub fn remove(&self, coords: &[u32]) {
		self.data.remove(&self.encode(coords));
	}
}

#[cfg(test)]
mod tests {
    use super::*;  // imports the MortonOctree

    #[test]
    fn insert_and_get() {
        let octree = ZOctree::new(3);
        octree.insert("hello", &[1, 2, 3]);
		let value1 = octree.get(&[1, 2, 3]);
		let value2 = octree.get(&[0, 0, 0]);
        assert_eq!(value1.unwrap().value(), &"hello");
        assert!(value2.is_none());
    }

    #[test]
    fn remove_works() {
        let octree = ZOctree::new(3);
        octree.insert(42, &[1, 1, 1]);
		assert_eq!(octree.get(&[1, 1, 1]).unwrap().value(), &42);
        octree.remove(&[1, 1, 1]);
        assert!(octree.get(&[1, 1, 1]).is_none());
    }

	#[test]
    fn test_get_radius_box() {
        let octree = ZOctree::new(3);

        // Insert points in 3D space
        octree.insert("a", &[0, 0, 0]);
        octree.insert("b", &[1, 1, 1]);
        octree.insert("c", &[2, 2, 2]);
        octree.insert("d", &[5, 5, 5]);
        octree.insert("e", &[10, 10, 10]);

        // Test radius 1 around [1,1,1]
        let results = octree.get_radius_box(&[1, 1, 1], 1);
        let mut found: Vec<&str> = results.iter().map(|(_, r)| *r.value()).collect();
        found.sort();
        assert_eq!(found, vec!["a", "b", "c"]); // 0,0,0; 1,1,1; 2,2,2

        // Test radius 0 (only center)
        let results = octree.get_radius_box(&[1, 1, 1], 0);
        let found: Vec<&str> = results.iter().map(|(_, r)| *r.value()).collect();
        assert_eq!(found, vec!["b"]);

        // Test radius covering no points
        let results = octree.get_radius_box(&[20, 20, 20], 2);
        assert!(results.is_empty());

        // Test radius covering all inserted points
        let results = octree.get_radius_box(&[5, 5, 5], 10);
        let mut found: Vec<&str> = results.iter().map(|(_, r)| *r.value()).collect();
        found.sort();
        assert_eq!(found, vec!["a", "b", "c", "d", "e"]);
    }
}
