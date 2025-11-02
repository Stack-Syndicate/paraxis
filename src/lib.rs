use std::collections::HashMap;
use rayon::prelude::*;

pub struct MortonOctree<T: Send + Sync> {
	dimensions: u128,
	bitdepth: u128,
	data: HashMap<u128, T>
}
impl<T: Send + Sync> MortonOctree<T> {
	pub fn new(dimensions: u128, bitdepth: u128) -> Self {
		Self {
			dimensions,
			bitdepth,
			data: HashMap::new()
		}
	}
	pub fn generate_morton_encoding(&self, coords: &[u128]) -> u128 {
		let mut morton_code: u128 = 0;
		for b in 0..self.bitdepth {
			for (i, d) in coords.iter().enumerate() {
				let coord_bit = (d >> b) & 1;
				morton_code |= coord_bit << (b * self.dimensions + i as u128);
			}
		}
		return morton_code;
	}
	pub fn insert(&mut self, entity: T, coords: &[u128]) {
		self.data.insert(self.generate_morton_encoding(coords), entity);
	}
	pub fn get(&self, coords: &[u128]) -> Option<&T> {
		self.data.get(&self.generate_morton_encoding(coords))
	}
	pub fn get_prefix(&self, coords: &[u128], level: u128) -> Vec<&T> {
		let full_code = self.generate_morton_encoding(coords);
		let shift = self.dimensions * (self.bitdepth - level);
		let prefix = full_code >> shift;
		self.data.par_iter()
			.filter_map(|(&code, value)| {
				if (code >> shift) == prefix { Some(value) } else { None }
			})
			.collect()
	}
	pub fn remove(&mut self, coords: &[u128]) {
		self.data.remove(&self.generate_morton_encoding(coords));
	}
}

#[cfg(test)]
mod tests {
    use super::*;  // imports the MortonOctree

    #[test]
    fn insert_and_get() {
        let mut octree = MortonOctree::new(3, 4);
        octree.insert("hello", &[1, 2, 3]);
        assert_eq!(octree.get(&[1, 2, 3]), Some(&"hello"));
        assert_eq!(octree.get(&[0, 0, 0]), None);
    }

    #[test]
    fn remove_works() {
        let mut octree = MortonOctree::new(3, 4);
        octree.insert(42, &[1, 1, 1]);
		assert_eq!(octree.get(&[1, 1, 1]), Some(&42));
        octree.remove(&[1, 1, 1]);
        assert_eq!(octree.get(&[1, 1, 1]), None);
    }
}
