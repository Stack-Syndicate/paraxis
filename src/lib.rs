use std::{
    collections::BTreeMap,
    sync::{Arc, Mutex},
};

pub struct ZOctree<T: Send + Sync + Clone + Copy> {
    dimensions: u16,
    data: Arc<Mutex<BTreeMap<u64, T>>>,
}
impl<T: Send + Sync + Clone + Copy> ZOctree<T> {
    pub fn new(dimensions: u16) -> Self {
        Self {
            dimensions,
            data: Arc::new(Mutex::new(BTreeMap::new())),
        }
    }
    pub fn encode(&self, coords: &[u16]) -> u64 {
        let mut morton_code: u64 = 0;
        let dim = self.dimensions as usize;
        // For every bit index
        for b in 0..16 {
            // Enumerate through the coordinates
            for (i, d) in coords.iter().enumerate() {
                // Shift the coordinate binary right by b, then retrieve the right-most bit
                let coord_bit: u64 = ((d >> b) & 1) as u64;
                // Insert the coordinate's bit into the morton code, offset by the coordinate index
                morton_code |= (coord_bit << (b * dim + i)) as u64;
            }
        }
        return morton_code;
    }
    pub fn decode(&self, code: u64) -> Vec<u16> {
        let dim = self.dimensions as usize;
        let mut coords = vec![0u16; dim];
        for b in 0..16 {
            for i in 0..dim {
                let bit = ((code >> (b * dim + i)) & 1) as u16;
                coords[i] |= bit << b;
            }
        }
        coords
    }
    pub fn insert(&self, entity: T, coords: &[u16]) {
		let mut data_lock = self.data.lock().expect("ZOctree data lock failed in insert.");
        data_lock.insert(self.encode(coords), entity);
    }
    pub fn get(&self, coords: &[u16]) -> Option<T> {
        let data_lock = self.data.lock().expect("ZOctree data lock failed in get.");
        data_lock.get(&self.encode(coords)).cloned()
    }
    pub fn set(&self, coords: &[u16], value: T) {
        let mut map = self.data.lock().expect("ZOctree data lock failed in set.");
        map.insert(self.encode(coords), value);
    }
    pub fn get_at_depth(&self, coords: &[u16], depth: u8) -> Vec<(u64, T)> {
        let full_code = self.encode(coords);
        let shift = self.dimensions * (16 - depth as u16);
        let prefix = full_code >> shift;
        let data_lock = self
            .data
            .lock()
            .expect("ZOctree data lock failed in get depth.");
        data_lock
            .iter()
            .filter_map(|(&code, value)| {
                if code >> shift == prefix {
                    Some((code, value.clone()))
                } else {
                    None
                }
            })
            .collect()
    }
    pub fn set_at_depth<F>(&mut self, coords: &[u16], depth: u8, mut f: F)
    where
        F: FnMut(&mut T),
    {
        let full_code = self.encode(coords);
        let shift = self.dimensions * (16 - depth as u16);
        let prefix = full_code >> shift;
        let mut data_lock = self
            .data
            .lock()
            .expect("ZOctree data lock failed in set depth.");
        for (&code, value) in data_lock.iter_mut() {
            if code >> shift == prefix {
                f(value)
            }
        }
    }
    pub fn get_radius_box(&self, center: &[u16], radius: u16) -> Vec<(u64, T)> {
        let min: Vec<u16> = center.iter().map(|&c| c.saturating_sub(radius)).collect();
        let max: Vec<u16> = center.iter().map(|&c| c + radius).collect();
        let data_lock = self
            .data
            .lock()
            .expect("ZOctree data lock failed in get radius box");
        data_lock
            .iter()
            .filter_map(|(&code, value)| {
                let coords = self.decode(code);
                if coords.iter().zip(&min).all(|(&c, &mi)| c >= mi)
                    && coords.iter().zip(&max).all(|(&c, &ma)| c <= ma)
                {
                    Some((code, value.clone()))
                } else {
                    None
                }
            })
            .collect()
    }
    pub fn set_radius_box<F>(&self, center: &[u16], radius: u16, mut f: F)
    where
        F: FnMut(&mut T),
    {
        let min: Vec<u16> = center.iter().map(|&c| c.saturating_sub(radius)).collect();
        let max: Vec<u16> = center.iter().map(|&c| c + radius).collect();
		let mut data_lock = self.data.lock().expect("ZOctree data lock failed in set radius box.");

        for (code, value) in data_lock.iter_mut() {
            let coords = self.decode(*code);
            if coords.iter().zip(&min).all(|(&c, &mi)| c >= mi)
                && coords.iter().zip(&max).all(|(&c, &ma)| c <= ma)
            {
                f(value);
            }
        }
    }
    pub fn remove(&self, coords: &[u16]) {
		let mut data_lock = self.data.lock().expect("ZOctree data lock failed in remove");
        data_lock.remove(&self.encode(coords));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn insert_and_get() {
        let octree = ZOctree::new(3);
        octree.insert("hello", &[1, 2, 3]);
        let value1 = octree.get(&[1, 2, 3]);
        let value2 = octree.get(&[0, 0, 0]);
        assert_eq!(value1.unwrap(), "hello");
        assert!(value2.is_none());
    }

    #[test]
    fn remove_works() {
        let octree = ZOctree::new(3);
        octree.insert(42, &[1, 1, 1]);
        assert_eq!(octree.get(&[1, 1, 1]).unwrap(), 42);
        octree.remove(&[1, 1, 1]);
        assert!(octree.get(&[1, 1, 1]).is_none());
    }

    #[test]
    fn test_get_radius_box() {
        let octree = ZOctree::new(3);

        octree.insert("a", &[0, 0, 0]);
        octree.insert("b", &[1, 1, 1]);
        octree.insert("c", &[2, 2, 2]);
        octree.insert("d", &[5, 5, 5]);
        octree.insert("e", &[10, 10, 10]);

        // Radius 1 around [1,1,1]
        let results = octree.get_radius_box(&[1, 1, 1], 1);
        let mut found: Vec<&str> = results.iter().map(|(_, r)| *r).collect();
        found.sort();
        assert_eq!(found, vec!["a", "b", "c"]);

        // Radius 0 (only center)
        let results = octree.get_radius_box(&[1, 1, 1], 0);
        let found: Vec<&str> = results.iter().map(|(_, r)| *r).collect();
        assert_eq!(found, vec!["b"]);

        // Radius covering no points
        let results = octree.get_radius_box(&[20, 20, 20], 2);
        assert!(results.is_empty());

        // Radius covering all inserted points
        let results = octree.get_radius_box(&[5, 5, 5], 10);
        let mut found: Vec<&str> = results.iter().map(|(_, r)| *r).collect();
        found.sort();
        assert_eq!(found, vec!["a", "b", "c", "d", "e"]);
    }
}
