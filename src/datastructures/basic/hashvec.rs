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

#[derive(Clone)]
pub struct HashVec<K, V: Clone> {
    data: Vec<Entry<K, V>>,
    map: AHashMap<K, usize>,
    head: Option<usize>,
    tail: Option<usize>,
}
impl<K: Hash + Eq + Clone, V: Clone> HashVec<K, V> {
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
}
impl<K, V: Clone> Index<usize> for HashVec<K, V> {
    type Output = Entry<K, V>;
    fn index(&self, index: usize) -> &Self::Output {
        return &self.data[index];
    }
}
impl<K, V: Clone> IndexMut<usize> for HashVec<K, V> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        return &mut self.data[index];
    }
}
impl<'a, K, V: Clone> HashVec<K, V> {
    pub fn iter(&'a self) -> Iter<'a, K, V> {
        Iter {
            data: &self.data,
            head: self.head,
            tail: self.tail,
        }
    }
}
impl<'a, K, V: Clone> IntoIterator for &'a HashVec<K, V> {
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
impl<'a, K, V: Clone> HashVec<K, V> {
    pub fn iter_mut(&'a mut self) -> IterMut<'a, K, V> {
        IterMut {
            ptr: self.data.as_mut_ptr(),
            head: self.head,
            tail: self.tail,
            _marker: std::marker::PhantomData,
        }
    }
}
impl<'a, K, V: Clone> IntoIterator for &'a mut HashVec<K, V> {
    type Item = &'a mut Entry<K, V>;
    type IntoIter = IterMut<'a, K, V>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter_mut()
    }
}
