use paraxis::mathematics::vector::Vector;

fn main() {
    let rv = Vector::<32>::random_normal();
    println!("{:?}", rv);
}
