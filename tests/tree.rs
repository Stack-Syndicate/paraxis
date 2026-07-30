use paraxis::containers::tree::{KDTree, Tree};

#[test]
fn tree() {
    let mut tree = KDTree::new();
    tree.add([10.0, 10.0, 10.0], "hello");
}
