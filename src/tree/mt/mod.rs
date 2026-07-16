use crate::encoding::MortonCode;
use nalgebra::{RealField, SVector, Scalar};
use num_traits::ToPrimitive;

pub mod ray;

pub struct MortonTree<T: Scalar + RealField + Copy + ToPrimitive, P, const N: usize> {
    pub data: Vec<(MortonCode<T, N>, P)>,
    pub min: SVector<T, N>,
    pub cell_sizes: Vec<u32>,
}
impl<T: Scalar + RealField + Copy + ToPrimitive, P, const N: usize> MortonTree<T, P, N> {
    pub fn new(raw_data: Vec<(SVector<T, N>, P)>, min: SVector<T, N>) -> Self {
        let mut morton_data = Vec::new();
        for raw_datum in raw_data {
            morton_data.push((MortonCode::from_vector(raw_datum.0, &min), raw_datum.1));
        }
        morton_data.sort_unstable_by_key(|a| a.0.bits);
        let mut cell_sizes = Vec::new();
        for d in 0..32 {
            let cell_size = 1u32 << d;
            cell_sizes.push(cell_size);
        }
        Self {
            data: morton_data,
            min,
            cell_sizes,
        }
    }
    pub fn search_bucket(&self, pos: &SVector<T, N>) -> &[(MortonCode<T, N>, P)] {
        let code = MortonCode::from_vector(*pos, &self.min);
        let start = self.data.partition_point(|(c, _)| *c.bits < code.bits);
        let end = self.data.partition_point(|(c, _)| *c.bits <= code.bits);
        &self.data[start..end]
    }
    pub fn search_level(
        &self,
        pos: &SVector<T, N>,
        depth_offset: usize,
    ) -> &[(MortonCode<T, N>, P)] {
        let depth = 32 - depth_offset;
        if self.data.is_empty() {
            return &[];
        }
        let full_code = MortonCode::from_vector(*pos, &self.min);
        let bit_count = (depth * N).min(32 * N);
        if bit_count == 0 {
            return &self.data;
        }
        let target_prefix = &full_code.bits[..bit_count];
        let start = self
            .data
            .partition_point(|(c, _)| c.bits[..bit_count] < target_prefix);
        let end = self.data[start..].partition_point(|(c, _)| c.bits[..bit_count] <= target_prefix)
            + start;
        &self.data[start..end]
    }
}
