use std::{collections::HashMap, hash::Hash};

#[derive(Debug, Clone)]
struct Entry<K, V> {
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

#[derive(Clone)]
pub struct HashVec<K, V: Clone> {
    data: Vec<Entry<K, V>>,
    map: HashMap<K, usize>,
    head: Option<usize>,
    tail: Option<usize>,
}
impl<K: Hash + Eq + Clone, V: Clone> HashVec<K, V> {
    pub fn new() -> Self {
        Self {
            data: Vec::new(),
            map: HashMap::new(),
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
    pub fn iter(&self) -> HashVecIter<'_, K, V> {
        HashVecIter {
            data: &self.data,
            next: self.head,
        }
    }
}

pub struct HashVecIter<'a, K, V> {
    data: &'a [Entry<K, V>],
    next: Option<usize>,
}
impl<'a, K, V> Iterator for HashVecIter<'a, K, V> {
    type Item = (&'a K, &'a V);

    fn next(&mut self) -> Option<Self::Item> {
        let i = self.next?;
        let entry = &self.data[i];
        self.next = entry.next;
        Some((&entry.key, &entry.value))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn iter_empty() {
        let hv: HashVec<i32, i32> = HashVec::new();
        let mut iter = hv.iter();
        assert_eq!(iter.next(), None);
    }

    #[test]
    fn iter_single_element() {
        let mut hv = HashVec::new();
        hv.insert(1, 10);

        let collected: Vec<_> = hv.iter().map(|(k, v)| (*k, *v)).collect();
        assert_eq!(collected, vec![(1, 10)]);
    }

    #[test]
    fn iter_multiple_elements_in_insertion_order() {
        let mut hv = HashVec::new();
        hv.insert(1, 10);
        hv.insert(2, 20);
        hv.insert(3, 30);

        let collected: Vec<_> = hv.iter().map(|(k, v)| (*k, *v)).collect();
        assert_eq!(collected, vec![(1, 10), (2, 20), (3, 30)]);
    }

    #[test]
    fn iter_after_removal_middle() {
        let mut hv = HashVec::new();
        hv.insert(1, 10);
        hv.insert(2, 20);
        hv.insert(3, 30);

        hv.remove(&2);

        let collected: Vec<_> = hv.iter().map(|(k, v)| (*k, *v)).collect();
        assert_eq!(collected, vec![(1, 10), (3, 30)]);
    }

    #[test]
    fn iter_after_removal_head() {
        let mut hv = HashVec::new();
        hv.insert(1, 10);
        hv.insert(2, 20);
        hv.insert(3, 30);

        hv.remove(&1);

        let collected: Vec<_> = hv.iter().map(|(k, v)| (*k, *v)).collect();
        assert_eq!(collected, vec![(2, 20), (3, 30)]);
    }

    #[test]
    fn iter_after_removal_tail() {
        let mut hv = HashVec::new();
        hv.insert(1, 10);
        hv.insert(2, 20);
        hv.insert(3, 30);
        hv.remove(&3);
        let collected: Vec<_> = hv.iter().map(|(k, v)| (*k, *v)).collect();
        assert_eq!(collected, vec![(1, 10), (2, 20)]);
    }

    #[test]
    fn iter_ignores_hashmap_order() {
        let mut hv = HashVec::new();
        hv.insert(42, 1);
        hv.insert(7, 2);
        hv.insert(99, 3);
        let collected: Vec<_> = hv.iter().map(|(k, _)| *k).collect();
        assert_eq!(collected, vec![42, 7, 99]);
    }

    #[test]
    fn multiple_iters_are_independent() {
        let mut hv = HashVec::new();
        hv.insert(1, 10);
        hv.insert(2, 20);

        let mut it1 = hv.iter();
        let mut it2 = hv.iter();

        assert_eq!(it1.next().map(|(k, _)| *k), Some(1));
        assert_eq!(it1.next().map(|(k, _)| *k), Some(2));
        assert_eq!(it1.next(), None);

        assert_eq!(it2.next().map(|(k, _)| *k), Some(1));
        assert_eq!(it2.next().map(|(k, _)| *k), Some(2));
        assert_eq!(it2.next(), None);
    }
    #[test]
    fn remove_only_element() {
        let mut hv = HashVec::new();
        hv.insert(1, 10);
        hv.remove(&1);

        assert!(hv.iter().next().is_none());
    }
}
