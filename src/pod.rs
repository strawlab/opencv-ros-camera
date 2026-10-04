//! A plain-data form of [`RosOpenCvIntrinsics`].
//!
//! ```
//! use opencv_ros_camera::{RosOpenCvIntrinsics, RosOpenCvIntrinsicsPod};
//!
//! let cam = RosOpenCvIntrinsics::<f32>::from_params(100.0, 0.0, 100.0, 320.0, 240.0);
//! let pod = RosOpenCvIntrinsicsPod::from(&cam);
//! let roundtrip = RosOpenCvIntrinsics::try_from(pod)?;
//! assert_eq!(cam, roundtrip);
//! # Ok::<(), opencv_ros_camera::Error>(())
//! ```

use nalgebra::{RealField, SMatrix, Vector5};

use crate::{Distortion, Error, Result, RosOpenCvIntrinsics};

/// Cache-free, `repr(C)` plain-data form of [`RosOpenCvIntrinsics`].
///
/// Matrices are stored in column-major order, as in `nalgebra`. Create one with
/// `RosOpenCvIntrinsicsPod::from(&intrinsics)` and go back with
/// [`to_intrinsics`](Self::to_intrinsics) or `RosOpenCvIntrinsics::try_from`.
///
/// Going back can fail with [`Error::InvalidInput`], because a `rect` that
/// cannot be inverted does not describe a usable camera.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(
    feature = "serde-pod",
    derive(serde::Serialize, serde::Deserialize),
    serde(bound(
        serialize = "R: serde::Serialize",
        deserialize = "R: serde::Deserialize<'de>"
    ))
)]
pub struct RosOpenCvIntrinsicsPod<R> {
    /// The intrinsic parameter matrix `P`, 3x4, column-major.
    pub p: [R; 12],
    /// The intrinsic parameter matrix `K`, 3x3, column-major.
    pub k: [R; 9],
    /// The OpenCV distortion parameters `D`.
    pub distortion: [R; 5],
    /// The stereo rectification matrix, 3x3, column-major.
    pub rect: [R; 9],
}

/// The `f32` flavor of [`RosOpenCvIntrinsicsPod`], 140 bytes.
///
/// On the C side this is `struct { float p[12]; float k[9]; float d[5]; float rect[9]; }`.
pub type RosOpenCvIntrinsicsPodF32 = RosOpenCvIntrinsicsPod<f32>;

/// The `f64` flavor of [`RosOpenCvIntrinsicsPod`], 280 bytes.
///
/// On the C side this is `struct { double p[12]; double k[9]; double d[5]; double rect[9]; }`.
pub type RosOpenCvIntrinsicsPodF64 = RosOpenCvIntrinsicsPod<f64>;

impl<R: RealField + Copy> RosOpenCvIntrinsicsPod<R> {
    /// Convert into a [`RosOpenCvIntrinsics`], rebuilding its derived cache.
    ///
    /// Returns `Err(Error::InvalidInput)` if `rect` cannot be inverted.
    pub fn to_intrinsics(&self) -> Result<RosOpenCvIntrinsics<R>> {
        RosOpenCvIntrinsics::from_components(
            SMatrix::<R, 3, 4>::from_column_slice(&self.p),
            SMatrix::<R, 3, 3>::from_column_slice(&self.k),
            Distortion::from_opencv_vec(Vector5::from_column_slice(&self.distortion)),
            SMatrix::<R, 3, 3>::from_column_slice(&self.rect),
        )
    }
}

impl<R: RealField + Copy> From<&RosOpenCvIntrinsics<R>> for RosOpenCvIntrinsicsPod<R> {
    fn from(orig: &RosOpenCvIntrinsics<R>) -> Self {
        let mut p = [orig.p[(0, 0)]; 12];
        p.copy_from_slice(orig.p.as_slice());
        let mut k = [orig.k[(0, 0)]; 9];
        k.copy_from_slice(orig.k.as_slice());
        let mut distortion = [orig.distortion.radial1(); 5];
        distortion.copy_from_slice(orig.distortion.opencv_vec().as_slice());
        let mut rect = [orig.rect[(0, 0)]; 9];
        rect.copy_from_slice(orig.rect.as_slice());
        Self {
            p,
            k,
            distortion,
            rect,
        }
    }
}

impl<R: RealField + Copy> From<RosOpenCvIntrinsics<R>> for RosOpenCvIntrinsicsPod<R> {
    #[inline]
    fn from(orig: RosOpenCvIntrinsics<R>) -> Self {
        Self::from(&orig)
    }
}

impl<R: RealField + Copy> TryFrom<RosOpenCvIntrinsicsPod<R>> for RosOpenCvIntrinsics<R> {
    type Error = Error;

    #[inline]
    fn try_from(orig: RosOpenCvIntrinsicsPod<R>) -> Result<Self> {
        orig.to_intrinsics()
    }
}

impl<R: RealField + Copy> TryFrom<&RosOpenCvIntrinsicsPod<R>> for RosOpenCvIntrinsics<R> {
    type Error = Error;

    #[inline]
    fn try_from(orig: &RosOpenCvIntrinsicsPod<R>) -> Result<Self> {
        orig.to_intrinsics()
    }
}
