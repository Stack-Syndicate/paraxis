# API

## `Scalar`

### Initialisation

```rust
let macro_init = scalar!(7);
let into_init = 7.into();
let new_init = Scalar::new(7);
```

### Operations

The operations $+$, $-$, $\times$, $\div$ are all supported, both for other `Scalar` objects and also for numerical primitives.

## `Vector`

### Initialisation

```rust
let macro_init = vector![7, 6, 5, 4, 3, 2, 1];
let into_init = vec![7, 6, 5, 4, 3, 2, 1].into();
let new_init = Vector::new([7, 6, 5, 4, 3, 2, 1]);
```

### Operations

1. The operations $+$, $-$ are valid for `Vector` objects acting on other `Vector` objects element wise.
2. `Scalar` objects can act on `Vector` objects for each element.
3. The cross product $\times$ is valid for two `Vector<3>` objects (`Vector` of dimension 3) and produces another `Vector<3>`.
4. The dot product $\cdot$ is valid for two `Vector` objects of any dimension.

#### Signatures

```rust
fn dot(&self, other: Vector) -> Scalar;
fn cross(&self, other: Vector<3>) -> Vector<3>;
```

### Algorithms
