pub trait Morton {
	fn split_by_3(a: u32) -> Self;
	fn encode(x: u32, y: u32, z: u32) -> Self;
	fn is_child(&self) -> bool;
	fn branch(&mut self);
}
pub type MortonCode = u64;
impl Morton for MortonCode {
	fn split_by_3(a: u32) -> Self {
		let mut x: u64 = (a & 0x1fffff).into();
		x = (x | x << 32) & 0x1f00000000ffff;
		x = (x | x << 16) & 0x1f0000ff0000ff;
		x = (x | x << 8) & 0x100f00f00f00f00f;
		x = (x | x << 4) & 0x10c30c30c30c30c3;
		x = (x | x << 2) & 0x1249249249249249;
		return x;
	}
	fn encode(x: u32, y: u32, z: u32) -> Self {
		let mut a: u64 = 0;
		a |= Self::split_by_3(x) | Self::split_by_3(y) << 1 | Self::split_by_3(z) << 2;
		a << 1
	}
	fn is_child(&self) -> bool {
		self & 1 != 0
	}
	fn branch(&mut self) {
		if self.is_child() { *self &= !0b01; }
	}
}