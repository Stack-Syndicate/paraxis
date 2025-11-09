pub trait Brick: Copy + Clone{}

#[derive(Clone, Copy, PartialEq, PartialOrd)]
pub struct Brick64 {
	pub occupancy: u64
}
impl Brick for Brick64{

}