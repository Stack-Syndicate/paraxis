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

impl<K, V: Clone> HashVec<K, V> {
    pub fn iter(&'_ self) -> HashVecIter<'_, K, V> {
        HashVecIter {
            data: &self.data,
            next: self.head,
        }
    }
}
