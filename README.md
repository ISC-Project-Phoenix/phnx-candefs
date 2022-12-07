# Phoenix CAN definitions for Rust

This crate contains common code for converting CAN frames to defined CAN structures for Project Phoenix.

```rust
use phnx_candefs::*;
use bxcan::Id::Extended;

// Create a frame for the bus, generic over Frame implementer
let frame: bxcan::Frame = EncoderCount {
count: 20,
velocity: 10.2,
}
.into_frame()
.unwrap();

if let Extended(id) = frame.id() {
assert_eq ! (id.as_raw(), 0x7);
} else {
assert ! (false)
}

// Convert a frame from the bus into one of our defined structures
let conv = CanMessage::from_frame(frame).unwrap();

if let CanMessage::EncoderCount(ec) = conv {
assert_eq!(ec.velocity, 10.2);
assert_eq!(ec.count, 20);
}
```