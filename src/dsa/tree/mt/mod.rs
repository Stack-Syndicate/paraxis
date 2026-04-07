use bitvec::prelude::*;
use num_traits::{Num, ToPrimitive};

use crate::maths::vec::Vector;

pub struct MortonTree<T: Num, P, const N: usize> {
    pub data: Vec<(BitVec<u8, Msb0>, Vector<T, N>, P)>,
    min: Vector<T, N>,
    scale: T,
    bits_per_axis: usize,
}
impl<T: Num + ToPrimitive + PartialOrd + Copy, P, const N: usize> MortonTree<T, P, N> {
    pub fn new(
        raw_data: Vec<(Vector<T, N>, P)>,
        min: Vector<T, N>,
        scale: T,
        bits_per_axis: usize,
    ) -> Self {
        let mut morton_data = Vec::new();
        for raw_datum in raw_data {
            morton_data.push((
                Self::calculate_morton_code(&raw_datum.0, &min, scale, bits_per_axis),
                raw_datum.0,
                raw_datum.1,
            ));
        }
        morton_data.sort_unstable_by(|a, b| a.0.cmp(&b.0));
        Self {
            data: morton_data,
            min,
            scale,
            bits_per_axis,
        }
    }
    fn calculate_morton_code(
        pos: &Vector<T, N>,
        min: &Vector<T, N>,
        scale: T,
        bits_per_axis: usize,
    ) -> BitVec<u8, Msb0> {
        let mut quantized = Vec::with_capacity(N);
        for i in 0..N {
            let val = (pos.inner[i] - min.inner[i]) * scale;
            let int_val = val.to_f64().unwrap_or(0.0).round() as u64;
            quantized.push(int_val);
        }
        let mut morton = BitVec::<u8, Msb0>::with_capacity(N * bits_per_axis);
        for bit in (0..bits_per_axis).rev() {
            for axis in 0..N {
                let bit_val = (quantized[axis] >> bit) & 1 == 1;
                morton.push(bit_val);
            }
        }
        morton
    }
    pub fn search_bucket(&self, pos: &Vector<T, N>) -> &[(BitVec<u8, Msb0>, Vector<T, N>, P)] {
        let code = Self::calculate_morton_code(pos, &self.min, self.scale, self.bits_per_axis);
        let start = self.data.partition_point(|(c, _, _)| *c < code);
        let end = self.data.partition_point(|(c, _, _)| *c <= code);
        &self.data[start..end]
    }
    pub fn search_level(
        &self,
        pos: &Vector<T, N>,
        depth: usize,
    ) -> &[(BitVec<u8, Msb0>, Vector<T, N>, P)] {
        let depth = self.bits_per_axis - depth;
        if self.data.is_empty() {
            return &[];
        }
        let full_code = Self::calculate_morton_code(pos, &self.min, self.scale, self.bits_per_axis);
        let bit_count = (depth * N).min(self.bits_per_axis * N);
        if bit_count == 0 {
            return &self.data;
        }
        let target_prefix = &full_code[..bit_count];
        let start = self.data.partition_point(|(c, _, _)| {
            let bit_len = c.len();
            if bit_len < bit_count {
                c.as_bitslice() < target_prefix
            } else {
                &c[..bit_count] < target_prefix
            }
        });
        let end = self.data.partition_point(|(c, _, _)| {
            let bit_len = c.len();
            if bit_len < bit_count {
                c.as_bitslice() <= target_prefix
            } else {
                &c[..bit_count] <= target_prefix
            }
        });
        &self.data[start..end]
    }
}
