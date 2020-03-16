# Crate `opencv_ros_camera` for the [Rust language](https://www.rust-lang.org/)

[![Crates.io][ci]][cl] ![MIT/Apache][li] [![docs.rs][di]][dl]

[ci]: https://img.shields.io/crates/v/opencv-ros-camera.svg
[cl]: https://crates.io/crates/opencv-ros-camera/

[li]: https://img.shields.io/crates/l/opencv-ros-camera.svg?maxAge=2592000

[di]: https://docs.rs/opencv-ros-camera/badge.svg
[dl]: https://docs.rs/opencv-ros-camera/

Geometric models of OpenCV/ROS cameras for photogrammetry

## About

This crate provides a geometric model of a camera compatible with OpenCV as
used by ROS (the Robot Operating System). The crate is in pure Rust, can be
compiled in `no_std` mode, implements the
[`IntrinsicsParameters`](https://docs.rs/cam-geom/latest/cam_geom/trait.IntrinsicParameters.html)
trait from the [`cam-geom`](https://crates.io/crates/cam-geom) and provides
support to read and write camera models in various formats.

In greater detail:

- Compatible with camera calibrations made by the ROS
  [`camera_calibration`](http://wiki.ros.org/camera_calibration) package,
  including
  [monocular](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration)
  and
  [stereo](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration)
  calibrations. Despite this compatibility, does not depend on ROS or
  OpenCV. (Furthermore, there is also no dependency on the excellent
  [rosrust](https://crates.io/crates/rosrust) crate).
- Can be compiled without the Rust standard library or allocator to support
  embedded applications.
- The [`RosOpenCvIntrinsics`](struct.RosOpenCvIntrinsics.html) type
  implements [the `IntrinsicsParameters` trait from the `cam-geom`
  crate](https://docs.rs/cam-geom/latest/cam_geom/trait.IntrinsicParameters.html).
  Thus, a
  [`cam_geom::Camera`](https://docs.rs/cam-geom/latest/cam_geom/struct.Camera.html)
  can be created to handle intrinsic parameters using `RosOpenCvIntrinsics`
  and camera pose handled by [the `ExtrinsicParameters` struct from
  `cam-geom`](https://docs.rs/cam-geom/latest/cam_geom/struct.ExtrinsicParameters.html).
  (See the example below.)
- When compiled with the `serde-serialize` feature, read camera calibrations
  saved by the ROS `camera_calibration/cameracalibrator.py` node in
  `~/.ros/camera_info/camera_name.yaml` with
  [`from_ros_yaml`](fn.from_ros_yaml.html).
- When compiled with the `serde-serialize` feature, read and write the
  [`RosOpenCvIntrinsics`](struct.RosOpenCvIntrinsics.html) struct using
  serde.

## Example - read a ROS YAML file and create a `cam_geom::Camera` from it.

Let's say we have YAML file saved by the ROS
`camera_calibration/cameracalibrator.py` node. How can we create a
[`cam_geom::Camera`](https://docs.rs/cam-geom/latest/cam_geom/struct.Camera.html)
from this?

```rust
use nalgebra::{Matrix2x3, Unit, Vector3};

// Here we have the YAML file contents hardcoded in a string. Ordinarily, you
// would read this from a file such as `~/.ros/camera_info/camera_name.yaml`, but
// for this example, it is hardcoded here.
let yaml_buf = b"image_width: 659
image_height: 494
camera_name: Basler_21029382
camera_matrix:
  rows: 3
  cols: 3
  data: [516.385667640757, 0, 339.167079537312, 0, 516.125799367807, 227.37993524141, 0, 0, 1]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.331416226762003, 0.143584747015566, 0.00314558656668844, -0.00393597842852019, 0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
projection_matrix:
  rows: 3
  cols: 4
  data: [444.369750976562, 0, 337.107817516087, 0, 0, 474.186859130859, 225.062742824321, 0, 0, 0, 1, 0]";

// The ROS YAML file does not contain the pose (no extrinsic parameters). Here we
// specify them directly. The camera is at (10,0,0), looing at (0,0,0), with up (0,0,1).
let camcenter = Vector3::new(10.0, 0.0, 0.0);
let lookat = Vector3::new(0.0, 0.0, 0.0);
let up = Unit::new_normalize(Vector3::new(0.0, 0.0, 1.0));
let pose = cam_geom::ExtrinsicParameters::from_view(&camcenter, &lookat, &up);

// We need the `serde-serialize` feature for the `from_ros_yaml` function.
#[cfg(feature = "serde-serialize")]
{
    let from_ros = opencv_ros_camera::from_ros_yaml(&yaml_buf[..]).unwrap();

    // Finally, create camera from intrinsic and extrinsic parameters.
    let camera = cam_geom::Camera::new(from_ros.intrinsics, pose);
}
```

## testing

Test `no_std` compilation with:

```
# install target with: "rustup target add thumbv7em-none-eabihf"
cargo check --no-default-features --target thumbv7em-none-eabihf
```

Run unit tests with:

```
cargo test --features std
cargo test --features serde-serialize
```

serde support requires std.

## re-render README.md

```
cargo install cargo-readme
cargo readme > README.md
```

==========================================================

## Code of conduct

Anyone who interacts with this software in any space, including but not limited
to this GitHub repository, must follow our [code of
conduct](code_of_conduct.md).

## License

Licensed under either of these:

 * Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE) or
   https://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or
   https://opensource.org/licenses/MIT)
