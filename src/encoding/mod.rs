use bitvec::{array::BitArray, order::Msb0};
use num_traits::Float;

use crate::maths::vec::Vector;

pub struct MortonCode<T: Float, const N: usize> {
    pub bits: BitArray<[u32; N], Msb0>,
    pub position: Vector<T, N>,
}
impl<'a, T: Float, const N: usize> MortonCode<T, N> {
    pub fn from_vector(position: Vector<T, N>, min: &Vector<T, N>) -> Self {
        let mut quantized = [0u64; N];
        let mut bits = BitArray::<[u32; N], Msb0>::ZERO;
        for i in 0..N {
            let axis_diff = position.inner[i] - min.inner[i];
            quantized[i] = axis_diff.to_u64().unwrap_or(0u64);
        }
        let mut current_bit = 0;
        for bit in (0..32).rev() {
            for axis in 0..N {
                let is_set = (quantized[axis] >> bit) & 1 == 1;
                if is_set {
                    bits.set(current_bit, true);
                }
                current_bit += 1;
                if current_bit >= N * 32 {
                    break;
                }
            }
        }
        Self { bits, position }
    }
}
impl<T: Float> MortonCode<T, 3> {
    pub fn to_u64(&self) -> u64 {
        let mut out = 0u64;
        let len = self.bits.len().min(64);
        for i in 0..len {
            if self.bits[i] {
                out |= 1u64 << i;
            }
        }
        out
    }
}
