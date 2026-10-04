# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased

### Added
- `RosOpenCvIntrinsicsPod`, a `repr(C)` and `Copy` struct holding just the four
  matrices of `RosOpenCvIntrinsics`. Converts both ways. `RosOpenCvIntrinsicsPodF32`
  and `RosOpenCvIntrinsicsPodF64` are aliases for the two usual cases.
- A `serde-pod` feature, which derives serde impls for the POD type. It
  needs neither std nor serde_yaml, unlike `serde-serialize` `serde-serialize`
  turns it on as well.

## [0.17.0](https://github.com/strawlab/opencv-ros-camera/compare/0.16.0...0.17.0) - 2026-08-07

### Added
- *(deps)* [**breaking**] update nalgebra to 0.35

## [0.16.0] - 2025-08-01

[0.16.0]: https://github.com/strawlab/opencv-ros-camera/releases/tag/0.16.0
