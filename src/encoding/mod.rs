use bitvec::{array::BitArray, order::Msb0};
use nalgebra::{RealField, SVector, Scalar};
use num_traits::ToPrimitive;

pub struct MortonCode<T: Scalar + RealField + Copy, const N: usize> {
    pub bits: BitArray<[u32; N], Msb0>,
    pub position: SVector<T, N>,
}

impl<T, const N: usize> MortonCode<T, N>
where
    T: Scalar + RealField + Copy + ToPrimitive,
{
    pub fn from_vector(position: SVector<T, N>, min: &SVector<T, N>) -> Self {
        let mut quantized = [0u32; N];
        let mut bits = BitArray::<[u32; N], Msb0>::ZERO;
        for i in 0..N {
            let axis_diff = position[i] - min[i];

            quantized[i] = axis_diff.to_u32().unwrap_or(0);
        }
        let mut current_bit = 0;
        for bit in (0..32).rev() {
            for q in quantized.iter().take(N) {
                let is_set = ((q >> bit) & 1) == 1;
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

impl<T: Scalar + RealField + Copy> MortonCode<T, 3> {
    pub fn to_u64(&self) -> u64 {
        let mut out = 0u64;
        let len = self.bits.len();
        let take = len.min(64);
        for i in 0..take {
            if self.bits[len - 1 - i] {
                out |= 1u64 << i;
            }
        }
        out
    }
}
