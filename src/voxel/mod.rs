pub mod morton;

use crate::voxel::morton::{Morton, MortonCode};

#[derive(Clone, Copy, PartialEq, Debug)]
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

#[derive(PartialEq, Clone, Debug)]
pub struct SparseVoxelOctree {
    voxels: Vec<Voxel>,
    pub size: u32,
}
impl SparseVoxelOctree {
    pub fn empty(size: u32) -> Self {
        Self {
            voxels: vec![Voxel {
                children: [None; 8],
                material: u32::MAX,
            }],
            size,
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
    pub fn get(&self, x: u32, y: u32, z: u32) -> Option<Voxel> {
        let code = MortonCode::encode(x, y, z);
        let depth = self.size.trailing_zeros();
        let mut voxel_index = 0;
        for level in (0..depth).rev() {
            let child_index = ((code >> (level * 3)) & 0b111) as usize;
            match self.voxels[voxel_index].children[child_index] {
                Some(next) => voxel_index = next,
                None => return None,
            }
        }
        Some(self.voxels[voxel_index])
    }
}