pub mod morton;
pub mod brick;

use std::collections::BTreeMap;

use crate::voxels::morton::*;
use crate::voxels::brick::*;

pub struct SparseVoxelOctree<B: Brick> {
    data: BTreeMap<MortonCode, B>,
}
impl<B: Brick + Send + Sync> SparseVoxelOctree<B> {
    pub fn new() -> Self {
        Self {
            data: BTreeMap::new(),
        }
    }
    pub fn get_voxel(&self, x: u32, y: u32, z: u32) -> Option<B> {
        let code = u64::encode(x, y, z);
        let get_opt = self.data.get(&code);
        if let Some(voxel) = get_opt {
            Some(voxel.clone())
        } else {
            None
        }
    }
    pub fn get_voxels_radius(&self, x: u32, y: u32, z: u32, r: u32) -> Vec<B> {
        let min = [
            x.saturating_sub(r),
            y.saturating_sub(r),
            z.saturating_sub(r),
        ];
        let max = [
            x.saturating_add(r),
            y.saturating_add(r),
            z.saturating_add(r),
        ];

        fn collect_parallel<B: Copy + Clone + Send + Sync>(
            map: &BTreeMap<MortonCode, B>,
            min: [u32; 3],
            max: [u32; 3],
        ) -> Vec<B> {
            // skip degenerate cube
            if min[0] > max[0] || min[1] > max[1] || min[2] > max[2] {
                return Vec::new();
            }

            let min_code = MortonCode::encode(min[0], min[1], min[2]);
            let max_code = MortonCode::encode(max[0], max[1], max[2]);

            if max_code - min_code <= 8 {
                return map
                    .range(min_code..=max_code)
                    .map(|(_, v)| v.clone())
                    .collect();
            }

            // find largest axis to split
            let dx = max[0] - min[0];
            let dy = max[1] - min[1];
            let dz = max[2] - min[2];

            if dx > dy && dx > dz {
                let mid = min[0] + dx / 2;
                let (left, right) = rayon::join(
                    || collect_parallel(map, min, [mid, max[1], max[2]]),
                    || collect_parallel(map, [mid + 1, min[1], min[2]], max),
                );
                [left, right].concat()
            } else if dy > dz {
                let mid = min[1] + dy / 2;
                let (left, right) = rayon::join(
                    || collect_parallel(map, min, [max[0], mid, max[2]]),
                    || collect_parallel(map, [min[0], mid + 1, min[2]], max),
                );
                [left, right].concat()
            } else {
                let mid = min[2] + dz / 2;
                let (left, right) = rayon::join(
                    || collect_parallel(map, min, [max[0], max[1], mid]),
                    || collect_parallel(map, [min[0], min[1], mid + 1], max),
                );
                [left, right].concat()
            }
        }

        collect_parallel(&self.data, min, max)
    }
    // pub fn set_voxel(&mut self, metadata: u64) {

	// }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    #[test]
    fn test_get_voxels_radius() {
        let mut map: BTreeMap<MortonCode, Brick64> = BTreeMap::new();

        // insert some voxels
        let voxels = vec![
            ((0, 0, 0), Brick64 { occupancy: 1 }),
            ((1, 0, 0), Brick64 { occupancy: 2 }),
            ((2, 2, 2), Brick64 { occupancy: 3 }),
            ((5, 5, 5), Brick64 { occupancy: 4 }),
        ];

        for ((x, y, z), brick) in voxels.iter() {
            let code = MortonCode::encode(*x, *y, *z);
            map.insert(code, brick.clone());
        }

        let mut octree = SparseVoxelOctree::new();
        octree.data = map;
        // search around (1,1,1) with radius 1
        let result: Vec<u64> = octree.get_voxels_radius(1, 1, 1, 1).iter().map(|&v| {
			v.occupancy
		}).collect();

        // should include bricks at (0,0,0), (1,0,0), (2,2,2)
        assert_eq!(result, vec![1, 2, 3]);
    }
}
