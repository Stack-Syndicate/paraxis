use std::collections::HashMap;

pub struct MortonOctree<T> {
	dimensions: u64,
	bitdepth: u64,
	data: HashMap<u64, T>
}
impl<T> MortonOctree<T> {
	pub fn new(dimensions: u64, bitdepth: u64) -> Self {
		Self {
			dimensions,
			bitdepth,
			data: HashMap::new()
		}
	}
	pub fn generate_morton_encoding(&self, coords: &[u64]) -> u64 {
		let mut morton_code: u64 = 0;
		for b in 0..self.bitdepth {
			for (i, d) in coords.iter().enumerate() {
				let coord_bit = (d >> b) & 1;
				morton_code |= coord_bit << (b * self.dimensions + i as u64);
			}
		}
		return morton_code;
	}
	pub fn insert(&mut self, entity: T, coords: &[u64]) {
		self.data.insert(self.generate_morton_encoding(coords), entity);
	}
	pub fn get(&self, coords: &[u64]) -> Option<&T> {
		self.data.get(&self.generate_morton_encoding(coords))
	}
	pub fn get_prefix(&self, coords: &[u64], level: u64) -> Vec<&T> {
		let full_code = self.generate_morton_encoding(coords);
		let shift = self.dimensions * (self.bitdepth - level);
		let prefix = full_code >> shift;
		self.data.iter()
			.filter_map(|(&code, value)| {
				if (code >> shift) == prefix { Some(value) } else { None }
			})
			.collect()
	}
	pub fn remove(&mut self, coords: &[u64]) {
		self.data.remove(&self.generate_morton_encoding(coords));
	}
}