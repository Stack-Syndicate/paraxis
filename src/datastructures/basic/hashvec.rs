use ahash::AHashMap;
use std::{
    hash::Hash,
    ops::{Index, IndexMut},
};

#[derive(Debug, Clone)]
pub struct Entry<K, V> {
    key: K,
    value: V,
    next: Option<usize>,
    previous: Option<usize>,
}
impl<K, V> Entry<K, V> {
    pub fn new(key: K, value: V, next: Option<usize>, previous: Option<usize>) -> Self {
        Self {
            key,
            value,
            next,
            previous,
        }
    }
}

#[derive(Clone, Debug)]
pub struct HashVec<K, V> {
    data: Vec<Entry<K, V>>,
    map: AHashMap<K, usize>,
    head: Option<usize>,
    tail: Option<usize>,
}
impl<K: Hash + Eq + Clone, V> HashVec<K, V> {
    pub fn new() -> Self {
        Self {
            data: Vec::new(),
            map: AHashMap::new(),
            head: None,
            tail: None,
        }
    }
    pub fn insert(&mut self, key: K, value: V) {
        if self.map.contains_key(&key) {
            return;
        };
        if self.data.is_empty() {
            self.data.push(Entry::new(key.clone(), value, None, None));
            self.head = Some(0);
            self.tail = Some(0);
            self.map.insert(key, 0);
        } else {
            let last_index = self.tail.unwrap();
            self.data
                .push(Entry::new(key.clone(), value, None, Some(last_index)));
            let new_index = self.data.len() - 1;
            self.data[last_index].next = Some(new_index);
            self.map.insert(key, new_index);
            self.tail = Some(new_index);
        }
    }
    pub fn remove(&mut self, key: &K) {
        if !self.map.contains_key(key) {
            return;
        }
        let index = match self.map.get(key) {
            Some(&i) => i,
            None => return,
        };
        self.map.remove(&key);
        let old_prev = self.data[index].previous;
        let old_next = self.data[index].next;
        if let Some(prev) = old_prev {
            self.data[prev].next = old_next;
        }
        if let Some(next) = old_next {
            self.data[next].previous = old_prev;
        }
        if self.head.unwrap() == index {
            self.head = old_next;
        }
        if self.tail.unwrap() == index {
            self.tail = old_prev;
        }
        let last_index = self.data.len() - 1;
        if index != last_index {
            self.data.swap_remove(index);
            if let Some(new_prev) = self.data[index].previous {
                self.data[new_prev].next = Some(index);
            }
            if let Some(new_next) = self.data[index].next {
                self.data[new_next].previous = Some(index);
            }
            if self.data[index].next.is_none() {
                self.tail = Some(index);
            }
            if self.data[index].previous.is_none() {
                self.head = Some(index);
            }
            self.map.remove(&self.data[index].key);
            self.map.insert(self.data[index].key.clone(), index);
        } else {
            self.data.pop();
        }
    }
    pub fn get(&self, key: &K) -> Option<&V> {
        let index_opt = self.map.get(key);
        if let Some(index) = index_opt {
            let entry = &self.data[*index];
            return Some(&entry.value);
        } else {
            return None;
        }
    }
    pub fn get_mut(&mut self, key: &K) -> Option<&mut V> {
        let index_opt = self.map.get_mut(key);
        if let Some(index) = index_opt {
            let entry = &mut self.data[*index];
            return Some(&mut entry.value);
        } else {
            return None;
        }
    }
    pub fn has(&self, key: &K) -> bool {
        return self.map.contains_key(key);
    }
    pub fn len(&self) -> usize {
        return self.data.len();
    }
}
impl<K, V> Index<usize> for HashVec<K, V> {
    type Output = Entry<K, V>;
    fn index(&self, index: usize) -> &Self::Output {
        return &self.data[index];
    }
}
impl<K, V> IndexMut<usize> for HashVec<K, V> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        return &mut self.data[index];
    }
}
impl<'a, K, V> HashVec<K, V> {
    pub fn iter(&'a self) -> Iter<'a, K, V> {
        Iter {
            data: &self.data,
            head: self.head,
            tail: self.tail,
        }
    }
}
impl<'a, K, V> IntoIterator for &'a HashVec<K, V> {
    type Item = &'a Entry<K, V>;
    type IntoIter = Iter<'a, K, V>;
    fn into_iter(self) -> Self::IntoIter {
        return self.iter();
    }
}
pub struct Iter<'a, K, V> {
    data: &'a [Entry<K, V>],
    head: Option<usize>,
    tail: Option<usize>,
}
impl<'a, K, V> Iterator for Iter<'a, K, V> {
    type Item = &'a Entry<K, V>;

    fn next(&mut self) -> Option<Self::Item> {
        let i = self.head?;
        let entry = &self.data[i];
        self.head = entry.next;
        Some(entry)
    }
}
impl<'a, K, V> DoubleEndedIterator for Iter<'a, K, V> {
    fn next_back(&mut self) -> Option<Self::Item> {
        let i = self.tail?;
        let entry = &self.data[i];
        self.tail = entry.previous;
        Some(entry)
    }
}
pub struct IterMut<'a, K, V> {
    ptr: *mut Entry<K, V>, // pointer to the data
    head: Option<usize>,
    tail: Option<usize>,
    _marker: std::marker::PhantomData<&'a mut Entry<K, V>>,
}
impl<'a, K, V> Iterator for IterMut<'a, K, V> {
    type Item = &'a mut Entry<K, V>;

    fn next(&mut self) -> Option<Self::Item> {
        let i = self.head?;
        unsafe {
            let entry = &mut *self.ptr.add(i);
            self.head = entry.next;
            Some(entry)
        }
    }
}
impl<'a, K, V> DoubleEndedIterator for IterMut<'a, K, V> {
    fn next_back(&mut self) -> Option<Self::Item> {
        let i = self.tail?;
        unsafe {
            let entry = &mut *self.ptr.add(i);
            self.tail = entry.previous;
            Some(entry)
        }
    }
}
impl<'a, K, V> HashVec<K, V> {
    pub fn iter_mut(&'a mut self) -> IterMut<'a, K, V> {
        IterMut {
            ptr: self.data.as_mut_ptr(),
            head: self.head,
            tail: self.tail,
            _marker: std::marker::PhantomData,
        }
    }
}
impl<'a, K, V> IntoIterator for &'a mut HashVec<K, V> {
    type Item = &'a mut Entry<K, V>;
    type IntoIter = IterMut<'a, K, V>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter_mut()
    }
}

#[cfg(test)]
mod hashvec_tests {
    use super::*; // Adjust this based on your module structure

    #[test]
    fn adding_and_removing() {
        let mut hv = HashVec::new();
        hv.insert(10, "first");
        hv.insert(200, "second");
        hv.insert(1, "third");

        // Verify initial state
        assert_eq!(hv.get(&10), Some(&"first"));
        assert_eq!(hv.get(&200), Some(&"second"));
        assert_eq!(hv.get(&1), Some(&"third"));

        // Remove the middle element
        hv.remove(&200);

        // 1. Assert lookup works correctly
        assert_eq!(hv.has(&200), false, "Key 200 should be removed from map");
        assert_eq!(hv.get(&200), None);
        assert_eq!(hv.get(&10), Some(&"first"), "Key 10 should still exist");
        assert_eq!(hv.get(&1), Some(&"third"), "Key 1 should still exist");

        // 2. Assert order is preserved (10 -> 1)
        let mut iter = hv.iter();

        let first = iter.next().expect("Should have a first element");
        assert_eq!(first.key, 10);
        assert_eq!(first.value, "first");

        let second = iter
            .next()
            .expect("Should have a second element (the moved 'third')");
        assert_eq!(second.key, 1);
        assert_eq!(second.value, "third");

        assert!(iter.next().is_none(), "Should only have 2 elements left");
    }

    #[test]
    fn test_map_consistency_after_remove() {
        let mut hv = HashVec::new();
        hv.insert("A", 1);
        hv.insert("B", 2);
        hv.insert("C", 3);

        // Initial state: data.len() = 3, map.len() = 3
        // Indices: A=0, B=1, C=2

        // Remove the first element "A".
        // This should move "C" from index 2 to index 0.
        hv.remove(&"A");

        // 1. Check if "C" is still retrievable (Logic check)
        assert_eq!(hv.get(&"C"), Some(&3), "Should still be able to find 'C'");

        // 2. Check Map Size (The "Zombie Entry" check)
        // If your code doesn't clean up the old index for 'C', the map size will be 3 instead of 2.
        // However, since you use AHashMap::insert, if the key "C" is re-inserted,
        // it overwrites the old "C" entry. BUT, did the old index 2 get cleaned up?
        assert_eq!(
            hv.map.len(),
            hv.data.len(),
            "Map and Data lengths are out of sync!"
        );
    }

    #[test]
    fn test_stale_index_leak() {
        let mut hv = HashVec::new();
        hv.insert("A", 1);
        hv.insert("B", 2);

        // Index mapping: A -> 0, B -> 1
        hv.remove(&"A");
        // B moves from index 1 to 0.

        // If we insert a NEW key, it will take index 1 (the new end of the Vec).
        hv.insert("C", 3);

        // If the map still has a "zombie" entry for 'B' pointing to index 1,
        // and we just inserted 'C' at index 1, the map is now corrupted.
        assert_eq!(hv.get(&"B"), Some(&2));
        assert_eq!(hv.get(&"C"), Some(&3));

        // Final sanity check: Total keys in map should be exactly 2 ("B" and "C")
        // If your implementation of `remove` doesn't remove the *old* key location,
        // you might have more entries in the map than in the vector.
    }
    #[test]
    fn test_complex_link_repair() {
        let mut hv = HashVec::new();
        hv.insert(1, "tail");
        hv.insert(2, "middle");
        hv.insert(3, "head");
        // Logical order: 1 <-> 2 <-> 3
        // Vec order: [1, 2, 3]

        // Remove "middle" (index 1).
        // swap_remove will move "head" (index 2) into index 1.
        hv.remove(&2);

        // Now verify the head still points to the correct thing
        // and the tail's "next" is correct.
        let items: Vec<_> = hv.iter().map(|e| e.value).collect();
        assert_eq!(items, vec!["tail", "head"]);

        // Verify reverse iteration
        let rev_items: Vec<_> = hv.iter().rev().map(|e| e.value).collect();
        assert_eq!(rev_items, vec!["head", "tail"]);
    }
}
