use paraxis::{common::traits::Tree, containers::tree::KDTree};

#[test]
fn instantiation() {
    let tree = KDTree::new(vec![([1.0], "a"), ([2.0], "b"), ([-1.0], "c")]);
}
