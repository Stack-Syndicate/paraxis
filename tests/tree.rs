use paraxis::{common::traits::Tree, containers::tree::KDTree};

#[test]
fn instantiation() {
    let tree = KDTree::new(vec![([1], "a"), ([2], "b"), ([-1], "c")]);
}
