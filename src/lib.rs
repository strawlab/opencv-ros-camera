//! Geometric models of OpenCV/ROS cameras for photogrammetry
//!
//! # About
//!
//! This crate provides a geometric model of a camera compatible with OpenCV as
//! used by ROS (the Robot Operating System). The crate is in pure Rust, can be
//! compiled in `no_std` mode, implements the
//! [`IntrinsicParameters`](https://docs.rs/cam-geom/latest/cam_geom/trait.IntrinsicParameters.html)
//! trait from the [`cam-geom`](https://crates.io/crates/cam-geom) and provides
//! support to read and write camera models in various formats.
//!
//! In greater detail:
//!
//! - Compatible with camera calibrations made by the ROS
//!   [`camera_calibration`](http://wiki.ros.org/camera_calibration) package,
//!   including
//!   [monocular](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration)
//!   and
//!   [stereo](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration)
//!   calibrations. Despite this compatibility, does not depend on ROS or
//!   OpenCV. (Furthermore, there is also no dependency on the excellent
//!   [rosrust](https://crates.io/crates/rosrust) crate).
//! - Can be compiled without the Rust standard library or allocator to support
//!   embedded applications.
//! - The [`RosOpenCvIntrinsics`](struct.RosOpenCvIntrinsics.html) type
//!   implements [the `IntrinsicParameters` trait from the `cam-geom`
//!   crate](https://docs.rs/cam-geom/latest/cam_geom/trait.IntrinsicParameters.html).
//!   Thus, a
//!   [`cam_geom::Camera`](https://docs.rs/cam-geom/latest/cam_geom/struct.Camera.html)
//!   can be created to handle intrinsic parameters using `RosOpenCvIntrinsics`
//!   and camera pose handled by [the `ExtrinsicParameters` struct from
//!   `cam-geom`](https://docs.rs/cam-geom/latest/cam_geom/struct.ExtrinsicParameters.html).
//!   (See the example below.)
//! - When compiled with the `serde-serialize` feature, read camera calibrations
//!   saved by the ROS `camera_calibration/cameracalibrator.py` node in
//!   `~/.ros/camera_info/camera_name.yaml` with
//!   [`from_ros_yaml`](fn.from_ros_yaml.html).
//! - When compiled with the `serde-serialize` feature, read and write the
//!   [`RosOpenCvIntrinsics`](struct.RosOpenCvIntrinsics.html) struct using
//!   serde.
//!
//! # Example - read a ROS YAML file and create a `cam_geom::Camera` from it.
//!
//! Let's say we have YAML file saved by the ROS
//! `camera_calibration/cameracalibrator.py` node. How can we create a
//! [`cam_geom::Camera`](https://docs.rs/cam-geom/latest/cam_geom/struct.Camera.html)
//! from this?
//!
//! ```
//! use nalgebra::{Matrix2x3, Unit, Vector3};
//!
//! // Here we have the YAML file contents hardcoded in a string. Ordinarily, you
//! // would read this from a file such as `~/.ros/camera_info/camera_name.yaml`, but
//! // for this example, it is hardcoded here.
//! let yaml_buf = b"image_width: 659
//! image_height: 494
//! camera_name: Basler_21029382
//! camera_matrix:
//!   rows: 3
//!   cols: 3
//!   data: [516.385667640757, 0, 339.167079537312, 0, 516.125799367807, 227.37993524141, 0, 0, 1]
//! distortion_model: plumb_bob
//! distortion_coefficients:
//!   rows: 1
//!   cols: 5
//!   data: [-0.331416226762003, 0.143584747015566, 0.00314558656668844, -0.00393597842852019, 0]
//! rectification_matrix:
//!   rows: 3
//!   cols: 3
//!   data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
//! projection_matrix:
//!   rows: 3
//!   cols: 4
//!   data: [444.369750976562, 0, 337.107817516087, 0, 0, 474.186859130859, 225.062742824321, 0, 0, 0, 1, 0]";
//!
//! // The ROS YAML file does not contain the pose (no extrinsic parameters). Here we
//! // specify them directly. The camera is at (10,0,0), looking at (0,0,0), with up (0,0,1).
//! let camcenter = Vector3::new(10.0, 0.0, 0.0);
//! let lookat = Vector3::new(0.0, 0.0, 0.0);
//! let up = Unit::new_normalize(Vector3::new(0.0, 0.0, 1.0));
//! let pose = cam_geom::ExtrinsicParameters::from_view(&camcenter, &lookat, &up);
//!
//! // We need the `serde-serialize` feature for the `from_ros_yaml` function.
//! #[cfg(feature = "serde-serialize")]
//! {
//!     let from_ros = opencv_ros_camera::from_ros_yaml(&yaml_buf[..]).unwrap();
//!
//!     // Finally, create camera from intrinsic and extrinsic parameters.
//!     let camera = cam_geom::Camera::new(from_ros.intrinsics, pose);
//! }
//! ```
//!
//! # testing
//!
//! Test `no_std` compilation with:
//!
//! ```text
//! # install target with: "rustup target add thumbv7em-none-eabihf"
//! cargo check --no-default-features --target thumbv7em-none-eabihf
//! ```
//!
//! Run unit tests with:
//!
//! ```text
//! cargo test
//! cargo test --features serde-serialize
//! ```
//!
//! serde support requires std.
//!
//! # re-render README.md
//!
//! ```text
//! cargo install cargo-readme
//! cargo readme > README.md
//! ```

#![deny(rust_2018_idioms, unsafe_code, missing_docs)]
#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(not(feature = "std"))]
extern crate core as std;

#[cfg(feature = "serde-serialize")]
use serde::{Deserialize, Serialize};

use nalgebra::{
    allocator::Allocator,
    base::storage::{Owned, Storage},
    convert, one, zero, DefaultAllocator, Dim, Matrix3, OMatrix, RealField, SMatrix, Vector2,
    Vector3, Vector5, U1, U2, U3,
};

use cam_geom::{
    coordinate_system::CameraFrame, ray_bundle_types::SharedOriginRayBundle, Bundle,
    IntrinsicParameters, Pixels, Points, RayBundle,
};

#[cfg(feature = "std")]
mod ros_file_support;
#[cfg(feature = "std")]
pub use ros_file_support::{NamedIntrinsicParameters, RosCameraInfo, RosMatrix};

#[cfg(feature = "serde-serialize")]
pub use ros_file_support::from_ros_yaml;

/// Possible errors.
#[derive(Debug)]
#[cfg_attr(feature = "std", derive(thiserror::Error))]
#[non_exhaustive]
pub enum Error {
    #[cfg_attr(feature = "std", error("invalid input"))]
    /// invalid input
    InvalidInput,
    #[cfg_attr(feature = "std", error("error parsing YAML"))]
    /// error parsing YAML
    YamlParseError,
    #[cfg_attr(feature = "std", error("unknown distortion model"))]
    /// unknown distortion model
    UnknownDistortionModel,
    #[cfg_attr(feature = "std", error("bad matrix size"))]
    /// bad matrix size
    BadMatrixSize,
}

#[cfg(feature = "serde-serialize")]
impl std::convert::From<serde_yaml::Error> for Error {
    #[inline]
    fn from(_orig: serde_yaml::Error) -> Self {
        Error::YamlParseError
    }
}

/// Result type
pub type Result<T> = std::result::Result<T, Error>;

/// A perspective camera model with distortion compatible with OpenCV and ROS.
///
/// This camera model is compatible with OpenCV and ROS, including stereo
/// rectification and Brown-Conrady
/// [distortion](https://en.wikipedia.org/wiki/Distortion_(optics)). To load
/// from a ROS YAML file, see the [`from_ros_yaml`](fn.from_ros_yaml.html)
/// function.
///
/// See [this page](http://wiki.ros.org/image_pipeline/CameraInfo) for an
/// expanded definition of the parameters.
///
/// To convert from a
/// [`NamedIntrinsicParameters`](struct.NamedIntrinsicParameters.html) struct,
/// use its
/// [`intrinsics`](struct.NamedIntrinsicParameters.html#structfield.intrinsics)
/// field.
///
/// See the [module-level documentation for more information](index.html).
#[derive(Debug, Clone, PartialEq)]
pub struct RosOpenCvIntrinsics<R: RealField> {
    /// If these intrinsics have zero skew, they are "opencv compatible" and this is `true`.
    pub is_opencv_compatible: bool,
    /// The intrinsic parameter matrix `P`.
    pub p: SMatrix<R, 3, 4>,
    /// The intrinsic parameter matrix `K`. Scaled from `P`.
    pub k: SMatrix<R, 3, 3>,
    /// The non-linear distortion parameters `D` specifying image warping.
    pub distortion: Distortion<R>,
    /// The stereo rectification matrix.
    pub rect: SMatrix<R, 3, 3>,
    cache: Cache<R>,
}

impl<R: RealField> From<cam_geom::IntrinsicParametersPerspective<R>> for RosOpenCvIntrinsics<R> {
    fn from(orig: cam_geom::IntrinsicParametersPerspective<R>) -> Self {
        Self::from_params(orig.fx(), orig.skew(), orig.fy(), orig.cx(), orig.cy())
    }
}

#[derive(Debug, Clone, PartialEq)]
struct Cache<R: RealField> {
    pnorm: SMatrix<R, 3, 4>,
    rect_t: Matrix3<R>,
    rti: Matrix3<R>,
}

/// Undistorted 2D pixel locations
///
/// See [the wikipedia page on
/// distortion](https://en.wikipedia.org/wiki/Distortion_(optics)) for a
/// discussion. This type represents pixel coordinates which have been
/// undistorted.
///
/// This is a newtype wrapping an `nalgebra::Matrix`.
pub struct UndistortedPixels<R: RealField, NPTS: Dim, STORAGE> {
    /// The undistorted pixel coordinates.
    pub data: nalgebra::Matrix<R, NPTS, U2, STORAGE>,
}

impl<R: RealField> RosOpenCvIntrinsics<R> {
    /// Construct intrinsics from raw components.
    ///
    /// Returns `Err(Error::InvalidInput)` if `rect` cannot be inverted.
    pub fn from_components(
        p: SMatrix<R, 3, 4>,
        k: SMatrix<R, 3, 3>,
        distortion: Distortion<R>,
        rect: SMatrix<R, 3, 3>,
    ) -> Result<Self> {
        let is_opencv_compatible = p[(0, 1)] == zero();
        let pnorm = p.clone() / p[(2, 2)].clone();
        let rect_t = rect.transpose();
        let mut rti = Matrix3::<R>::identity();
        if !nalgebra::linalg::try_invert_to(rect_t.clone(), &mut rti) {
            return Err(Error::InvalidInput);
        }

        let cache = Cache { pnorm, rect_t, rti };
        Ok(Self {
            is_opencv_compatible,
            p,
            k,
            distortion,
            rect,
            cache,
        })
    }

    /// Construct intrinsics from individual parameters with no distortion.
    ///
    /// `fx` and `fy` are the horizontal and vertical focal lengths. `skew` is
    /// the pixel skew (typically near zero). `cx` and `cy` is the center of the
    /// optical axis in pixel coordinates.
    #[inline]
    pub fn from_params(fx: R, skew: R, fy: R, cx: R, cy: R) -> Self {
        Self::from_params_with_distortion(fx, skew, fy, cx, cy, Distortion::zero())
    }

    /// Construct intrinsics from individual parameters.
    ///
    /// `fx` and `fy` are the horizontal and vertical focal lengths. `skew` is
    /// the pixel skew (typically near zero). `cx` and `cy` is the center of the
    /// optical axis in pixel coordinates. `distortion` is a vector of the
    /// distortion terms.
    pub fn from_params_with_distortion(
        fx: R,
        skew: R,
        fy: R,
        cx: R,
        cy: R,
        distortion: Distortion<R>,
    ) -> Self {
        let zero: R = zero();
        let one: R = one();
        let p = SMatrix::<R, 3, 4>::new(
            fx.clone(),
            skew.clone(),
            cx.clone(),
            zero.clone(),
            zero.clone(),
            fy.clone(),
            cy.clone(),
            zero.clone(),
            zero.clone(),
            zero.clone(),
            one.clone(),
            zero.clone(),
        );
        let k =
            SMatrix::<R, 3, 3>::new(fx, skew, cx, zero.clone(), fy, cy, zero.clone(), zero, one);
        let rect = Matrix3::<R>::identity();
        // Since rect can be inverted, this will not fail and we can unwrap.
        Self::from_components(p, k, distortion, rect).unwrap()
    }

    /// Convert undistorted pixel coordinates to distorted pixel coordinates.
    ///
    /// This will take coordinates from, e.g. a linear camera model, warp them
    /// into their distorted counterparts. This distortion thus models the
    /// action of a real lens.
    #[allow(clippy::many_single_char_names)]
    pub fn distort<NPTS, IN>(
        &self,
        undistorted: &UndistortedPixels<R, NPTS, IN>,
    ) -> Pixels<R, NPTS, Owned<R, NPTS, U2>>
    where
        NPTS: Dim,
        IN: nalgebra::base::storage::Storage<R, NPTS, U2>,
        DefaultAllocator: Allocator<NPTS, U2>,
    {
        let mut result = Pixels::new(OMatrix::zeros_generic(
            NPTS::from_usize(undistorted.data.nrows()),
            U2::from_usize(2),
        ));

        let p = &self.p;
        let fx = p[(0, 0)].clone();
        let cx = p[(0, 2)].clone();
        let tx = p[(0, 3)].clone();
        let fy = p[(1, 1)].clone();
        let cy = p[(1, 2)].clone();
        let ty = p[(1, 3)].clone();

        let one: R = one();
        let two: R = convert(2.0);

        let d = &self.distortion;
        let k1 = d.radial1();
        let k2 = d.radial2();
        let p1 = d.tangential1();
        let p2 = d.tangential2();
        let k3 = d.radial3();

        let k = &self.k;
        let kfx = k[(0, 0)].clone();
        let kcx = k[(0, 2)].clone();
        let kfy = k[(1, 1)].clone();
        let kcy = k[(1, 2)].clone();

        for i in 0..undistorted.data.nrows() {
            let x = (undistorted.data[(i, 0)].clone() - cx.clone() - tx.clone()) / fx.clone();
            let y = (undistorted.data[(i, 1)].clone() - cy.clone() - ty.clone()) / fy.clone();

            let xy1 = Vector3::new(x.clone(), y.clone(), one.clone());
            let xyw = self.cache.rect_t.clone() * xy1;
            let xp = xyw[0].clone() / xyw[2].clone();
            let yp = xyw[1].clone() / xyw[2].clone();

            let r2 = xp.clone() * xp.clone() + yp.clone() * yp.clone();
            let r4 = r2.clone() * r2.clone();
            let r6 = r4.clone() * r2.clone();
            let a1 = two.clone() * xp.clone() * yp.clone();

            let barrel = one.clone()
                + k1.clone() * r2.clone()
                + k2.clone() * r4.clone()
                + k3.clone() * r6.clone();
            let xpp = xp.clone() * barrel.clone()
                + p1.clone() * a1.clone()
                + p2.clone() * (r2.clone() + two.clone() * (xp.clone() * xp.clone()));
            let ypp = yp.clone() * barrel.clone()
                + p1.clone() * (r2.clone() + two.clone() * (yp.clone() * yp.clone()))
                + p2.clone() * a1.clone();

            let u = xpp.clone() * kfx.clone() + kcx.clone();
            let v = ypp.clone() * kfy.clone() + kcy.clone();

            result.data[(i, 0)] = u;
            result.data[(i, 1)] = v;
        }
        result
    }

    /// Convert distorted pixel coordinates to undistorted pixel coordinates.
    ///
    /// This will take distorted coordinates from, e.g. detections from a real
    /// camera image, and undo the effect of the distortion model. This
    /// "undistortion" thus converts coordinates from a real lens into
    /// coordinates that can be used with a linear camera model.
    ///
    /// This method calls [undistort_ext](Self::undistort_ext) using the default
    /// termination criteria.
    pub fn undistort<NPTS, IN>(
        &self,
        distorted: &Pixels<R, NPTS, IN>,
    ) -> UndistortedPixels<R, NPTS, Owned<R, NPTS, U2>>
    where
        NPTS: Dim,
        IN: nalgebra::base::storage::Storage<R, NPTS, U2>,
        DefaultAllocator: Allocator<NPTS, U2>,
    {
        self.undistort_ext(distorted, None)
    }

    /// Convert distorted pixel coordinates to undistorted pixel coordinates.
    ///
    /// This will take distorted coordinates from, e.g. detections from a real
    /// camera image, and undo the effect of the distortion model. This
    /// "undistortion" thus converts coordinates from a real lens into
    /// coordinates that can be used with a linear camera model.
    ///
    /// If the termination criteria are not specified, the default of five
    /// iterations is used.
    #[allow(clippy::many_single_char_names)]
    pub fn undistort_ext<NPTS, IN>(
        &self,
        distorted: &Pixels<R, NPTS, IN>,
        criteria: impl Into<Option<TermCriteria>>,
    ) -> UndistortedPixels<R, NPTS, Owned<R, NPTS, U2>>
    where
        NPTS: Dim,
        IN: nalgebra::base::storage::Storage<R, NPTS, U2>,
        DefaultAllocator: Allocator<NPTS, U2>,
    {
        let criteria = criteria.into().unwrap_or(TermCriteria::MaxIter(5));
        let mut result = UndistortedPixels {
            data: OMatrix::zeros_generic(
                NPTS::from_usize(distorted.data.nrows()),
                U2::from_usize(2),
            ),
        };

        let k = &self.k;
        let fx = k[(0, 0)].clone();
        let cx = k[(0, 2)].clone();
        let fy = k[(1, 1)].clone();
        let cy = k[(1, 2)].clone();

        let p = &self.p;
        let fxp = p[(0, 0)].clone();
        let cxp = p[(0, 2)].clone();
        let fyp = p[(1, 1)].clone();
        let cyp = p[(1, 2)].clone();

        let d = &self.distortion;

        let t1 = d.tangential1();
        let t2 = d.tangential2();

        let one: R = one();
        let two: R = convert(2.0);

        for i in 0..distorted.data.nrows() {
            // Apply intrinsic parameters to get normalized, distorted coordinates
            let xd = (distorted.data[(i, 0)].clone() - cx.clone()) / fx.clone();
            let yd = (distorted.data[(i, 1)].clone() - cy.clone()) / fy.clone();

            let mut x = xd.clone();
            let mut y = yd.clone();
            let mut count = 0;

            loop {
                if let TermCriteria::MaxIter(max_count) = criteria {
                    count += 1;
                    if count > max_count {
                        break;
                    }
                }

                let r2 = x.clone() * x.clone() + y.clone() * y.clone();
                let icdist = one.clone()
                    / (one.clone()
                        + ((d.radial3() * r2.clone() + d.radial2()) * r2.clone() + d.radial1())
                            * r2.clone());
                let delta_x = two.clone() * t1.clone() * x.clone() * y.clone()
                    + t2.clone() * (r2.clone() + two.clone() * x.clone() * x.clone());
                let delta_y = t1.clone() * (r2.clone() + two.clone() * y.clone() * y.clone())
                    + two.clone() * t2.clone() * x.clone() * y.clone();
                x = (xd.clone() - delta_x) * icdist.clone();
                y = (yd.clone() - delta_y) * icdist.clone();

                if let TermCriteria::Eps(eps) = criteria {
                    let r2 = x.clone() * x.clone() + y.clone() * y.clone();
                    let cdist = one.clone()
                        + ((d.radial3() * r2.clone() + d.radial2()) * r2.clone() + d.radial1())
                            * r2.clone();
                    let delta_x = two.clone() * t1.clone() * x.clone() * y.clone()
                        + t2.clone() * (r2.clone() + two.clone() * x.clone() * x.clone());
                    let delta_y = t1.clone() * (r2.clone() + two.clone() * y.clone() * y.clone())
                        + two.clone() * t2.clone() * x.clone() * y.clone();
                    let xp0 = x.clone() * cdist.clone() + delta_x.clone();
                    let yp0 = y.clone() * cdist.clone() + delta_y.clone();

                    let xywt = self.cache.rti.clone() * Vector3::new(xp0, yp0, one.clone());
                    let xp = xywt[0].clone() / xywt[2].clone();
                    let yp = xywt[1].clone() / xywt[2].clone();

                    let up = x.clone() * fxp.clone() + cxp.clone();
                    let vp = y.clone() * fyp.clone() + cyp.clone();

                    let error = (Vector2::new(xp, yp) - Vector2::new(up, vp)).norm();
                    if error < convert(eps) {
                        break;
                    }
                }
            }

            let xp = x;
            let yp = y;

            let uh = Vector3::new(xp.clone(), yp.clone(), one.clone());
            let xywt = self.cache.rti.clone() * uh.clone();
            let x = xywt[0].clone() / xywt[2].clone();
            let y = xywt[1].clone() / xywt[2].clone();

            let up = x.clone() * fxp.clone() + cxp.clone();
            let vp = y.clone() * fyp.clone() + cyp.clone();
            result.data[(i, 0)] = up.clone();
            result.data[(i, 1)] = vp.clone();
        }
        result
    }

    /// Convert 3D coordinates in `CameraFrame` to undistorted pixel coords.
    pub fn camera_to_undistorted_pixel<IN, NPTS>(
        &self,
        camera: &Points<CameraFrame, R, NPTS, IN>,
    ) -> UndistortedPixels<R, NPTS, Owned<R, NPTS, U2>>
    where
        IN: Storage<R, NPTS, U3>,
        NPTS: Dim,
        DefaultAllocator: Allocator<NPTS, U2>,
        DefaultAllocator: Allocator<U1, U2>,
    {
        let mut result = UndistortedPixels {
            data: OMatrix::zeros_generic(NPTS::from_usize(camera.data.nrows()), U2::from_usize(2)),
        };

        // TODO: can we remove this loop?
        for i in 0..camera.data.nrows() {
            let x = nalgebra::Point3::new(
                camera.data[(i, 0)].clone(),
                camera.data[(i, 1)].clone(),
                camera.data[(i, 2)].clone(),
            )
            .to_homogeneous();
            let rst = self.p.clone() * x;

            result.data[(i, 0)] = rst[0].clone() / rst[2].clone();
            result.data[(i, 1)] = rst[1].clone() / rst[2].clone();
        }
        result
    }

    /// Convert undistorted pixel coordinates to 3D coords in the `CameraFrame`.
    pub fn undistorted_pixel_to_camera<IN, NPTS>(
        &self,
        undistorteds: &UndistortedPixels<R, NPTS, IN>,
    ) -> RayBundle<CameraFrame, SharedOriginRayBundle<R>, R, NPTS, Owned<R, NPTS, U3>>
    where
        IN: Storage<R, NPTS, U2>,
        NPTS: Dim,
        DefaultAllocator: Allocator<NPTS, U3>,
        DefaultAllocator: Allocator<U1, U2>,
    {
        let p = self.cache.pnorm.clone();

        let mut result = RayBundle::new_shared_zero_origin(OMatrix::zeros_generic(
            NPTS::from_usize(undistorteds.data.nrows()),
            U3::from_usize(3),
        ));

        // TODO: eliminate this loop
        for i in 0..undistorteds.data.nrows() {
            // Create a slice view of a single pixel coordinate.
            let undistorted = UndistortedPixels {
                data: undistorteds.data.row(i),
            };

            let uv_rect_x = undistorted.data[(0, 0)].clone();
            let uv_rect_y = undistorted.data[(0, 1)].clone();

            // Convert undistorted point into camcoords.
            let y = (uv_rect_y.clone() - p[(1, 2)].clone() - p[(1, 3)].clone()) / p[(1, 1)].clone();
            let x = (uv_rect_x.clone()
                - p[(0, 1)].clone() * y.clone()
                - p[(0, 2)].clone()
                - p[(0, 3)].clone())
                / p[(0, 0)].clone();
            let z = one();

            result.data[(i, 0)] = x;
            result.data[(i, 1)] = y;
            result.data[(i, 2)] = z;
        }
        result
    }
}

/// Specifies distortion using the Brown-Conrady "plumb bob" model.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct Distortion<R: RealField>(Vector5<R>);

impl<R: RealField> Distortion<R> {
    /// build from vector ordered [radial1, radial2, tangential1, tangential2, radial3]
    #[inline]
    pub fn from_opencv_vec(v: Vector5<R>) -> Self {
        Distortion(v)
    }

    /// OpenCV ordered vector of distortion terms.
    ///
    /// The order is [radial1, radial2, tangential1, tangential2, radial3].
    #[inline]
    pub fn opencv_vec(&self) -> &Vector5<R> {
        &self.0
    }

    /// Construct a zero distortion model.
    #[inline]
    pub fn zero() -> Self {
        Distortion(Vector5::new(zero(), zero(), zero(), zero(), zero()))
    }

    /// The first radial distortion term, sometimes called `k1`.
    #[inline]
    pub fn radial1(&self) -> R {
        self.0[0].clone()
    }

    /// The first radial distortion term, sometimes called `k1` (mutable reference).
    #[inline]
    pub fn radial1_mut(&mut self) -> &mut R {
        &mut self.0[0]
    }

    /// The second radial distortion term, sometimes called `k2`.
    #[inline]
    pub fn radial2(&self) -> R {
        self.0[1].clone()
    }

    /// The second radial distortion term, sometimes called `k2` (mutable reference).
    #[inline]
    pub fn radial2_mut(&mut self) -> &mut R {
        &mut self.0[1]
    }

    /// The first tangential distortion term, sometimes called `p1`.
    #[inline]
    pub fn tangential1(&self) -> R {
        self.0[2].clone()
    }

    /// The first tangential distortion term, sometimes called `p1` (mutable reference).
    #[inline]
    pub fn tangential1_mut(&mut self) -> &mut R {
        &mut self.0[2]
    }

    /// The second tangential distortion term, sometimes called `p2`.
    #[inline]
    pub fn tangential2(&self) -> R {
        self.0[3].clone()
    }

    /// The second tangential distortion term, sometimes called `p2` (mutable reference).
    #[inline]
    pub fn tangential2_mut(&mut self) -> &mut R {
        &mut self.0[3]
    }

    /// The third radial distortion term, sometimes called `k3`.
    #[inline]
    pub fn radial3(&self) -> R {
        self.0[4].clone()
    }

    /// The third radial distortion term, sometimes called `k3` (mutable reference).
    #[inline]
    pub fn radial3_mut(&mut self) -> &mut R {
        &mut self.0[4]
    }

    /// Return `true` if there is approximately zero distortion, else `false`.
    pub fn is_linear(&self) -> bool {
        let v = &self.0;
        let sum_squared = v.dot(v);
        sum_squared < nalgebra::convert(1e-16)
    }
}

impl<R: RealField> IntrinsicParameters<R> for RosOpenCvIntrinsics<R> {
    type BundleType = SharedOriginRayBundle<R>;

    fn pixel_to_camera<IN, NPTS>(
        &self,
        pixels: &Pixels<R, NPTS, IN>,
    ) -> RayBundle<CameraFrame, Self::BundleType, R, NPTS, Owned<R, NPTS, U3>>
    where
        Self::BundleType: Bundle<R>,
        IN: Storage<R, NPTS, U2>,
        NPTS: Dim,
        DefaultAllocator: Allocator<NPTS, U2>,
        DefaultAllocator: Allocator<NPTS, U3>,
        DefaultAllocator: Allocator<U1, U2>,
    {
        let undistorted = self.undistort::<NPTS, IN>(pixels);
        self.undistorted_pixel_to_camera(&undistorted)
    }

    fn camera_to_pixel<IN, NPTS>(
        &self,
        camera: &Points<CameraFrame, R, NPTS, IN>,
    ) -> Pixels<R, NPTS, Owned<R, NPTS, U2>>
    where
        IN: Storage<R, NPTS, U3>,
        NPTS: Dim,
        DefaultAllocator: Allocator<NPTS, U2>,
    {
        let undistorted = self.camera_to_undistorted_pixel(camera);
        self.distort(&undistorted)
    }
}

/// Extension trait to add `world_to_undistorted_pixel()` method.
pub trait CameraExt<R: RealField> {
    /// Convert 3D coordinates in the `WorldFrame` to undistorted pixel coordinates.
    fn world_to_undistorted_pixel<NPTS, InStorage>(
        &self,
        world: &Points<cam_geom::WorldFrame, R, NPTS, InStorage>,
    ) -> UndistortedPixels<R, NPTS, Owned<R, NPTS, U2>>
    where
        NPTS: Dim,
        InStorage: Storage<R, NPTS, U3>,
        DefaultAllocator: Allocator<NPTS, U3>,
        DefaultAllocator: Allocator<NPTS, U2>;
}

impl<R: RealField> CameraExt<R> for cam_geom::Camera<R, RosOpenCvIntrinsics<R>> {
    fn world_to_undistorted_pixel<NPTS, InStorage>(
        &self,
        world: &Points<cam_geom::WorldFrame, R, NPTS, InStorage>,
    ) -> UndistortedPixels<R, NPTS, Owned<R, NPTS, U2>>
    where
        NPTS: Dim,
        InStorage: Storage<R, NPTS, U3>,
        DefaultAllocator: Allocator<NPTS, U3>,
        DefaultAllocator: Allocator<NPTS, U2>,
    {
        let camera_frame = self.extrinsics().world_to_camera(world);
        self.intrinsics().camera_to_undistorted_pixel(&camera_frame)
    }
}

#[cfg(feature = "serde-serialize")]
impl<R: RealField + serde::Serialize> serde::Serialize for RosOpenCvIntrinsics<R> {
    fn serialize<S>(&self, serializer: S) -> std::result::Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        use serde::ser::SerializeStruct;

        // 4 is the number of fields we serialize from the struct.
        let mut state = serializer.serialize_struct("RosOpenCvIntrinsics", 4)?;
        state.serialize_field("p", &self.p)?;
        state.serialize_field("k", &self.k)?;
        state.serialize_field("distortion", &self.distortion)?;
        state.serialize_field("rect", &self.rect)?;
        state.end()
    }
}

#[cfg(feature = "serde-serialize")]
impl<'de, R: RealField + serde::Deserialize<'de>> serde::Deserialize<'de>
    for RosOpenCvIntrinsics<R>
{
    fn deserialize<D>(deserializer: D) -> std::result::Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        use serde::de;
        use std::fmt;

        #[derive(Deserialize)]
        #[serde(field_identifier, rename_all = "lowercase")]
        enum Field {
            P,
            K,
            Distortion,
            Rect,
        }

        struct IntrinsicParametersVisitor<'de, R2: RealField + serde::Deserialize<'de>>(
            std::marker::PhantomData<&'de R2>,
        );

        impl<'de, R2: RealField + serde::Deserialize<'de>> serde::de::Visitor<'de>
            for IntrinsicParametersVisitor<'de, R2>
        {
            type Value = RosOpenCvIntrinsics<R2>;

            fn expecting(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
                formatter.write_str("struct RosOpenCvIntrinsics")
            }

            fn visit_seq<V>(
                self,
                mut seq: V,
            ) -> std::result::Result<RosOpenCvIntrinsics<R2>, V::Error>
            where
                V: serde::de::SeqAccess<'de>,
            {
                let p = seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(0, &self))?;
                let k = seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(1, &self))?;
                let distortion = seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(1, &self))?;
                let rect = seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(1, &self))?;
                // Ok(RosOpenCvIntrinsics::from_components(p, k, distortion, rect))
                RosOpenCvIntrinsics::from_components(p, k, distortion, rect)
                    .map_err(|e| de::Error::custom(e))
            }

            fn visit_map<V>(
                self,
                mut map: V,
            ) -> std::result::Result<RosOpenCvIntrinsics<R2>, V::Error>
            where
                V: serde::de::MapAccess<'de>,
            {
                let mut p = None;
                let mut k = None;
                let mut distortion = None;
                let mut rect = None;
                while let Some(key) = map.next_key()? {
                    match key {
                        Field::P => {
                            if p.is_some() {
                                return Err(de::Error::duplicate_field("p"));
                            }
                            p = Some(map.next_value()?);
                        }
                        Field::K => {
                            if k.is_some() {
                                return Err(de::Error::duplicate_field("k"));
                            }
                            k = Some(map.next_value()?);
                        }
                        Field::Distortion => {
                            if distortion.is_some() {
                                return Err(de::Error::duplicate_field("distortion"));
                            }
                            distortion = Some(map.next_value()?);
                        }
                        Field::Rect => {
                            if rect.is_some() {
                                return Err(de::Error::duplicate_field("rect"));
                            }
                            rect = Some(map.next_value()?);
                        }
                    }
                }
                let p = p.ok_or_else(|| de::Error::missing_field("p"))?;
                let k = k.ok_or_else(|| de::Error::missing_field("k"))?;
                let distortion =
                    distortion.ok_or_else(|| de::Error::missing_field("distortion"))?;
                let rect = rect.ok_or_else(|| de::Error::missing_field("rect"))?;
                RosOpenCvIntrinsics::from_components(p, k, distortion, rect)
                    .map_err(|e| de::Error::custom(e))
            }
        }

        const FIELDS: &'static [&'static str] = &["p", "k", "distortion", "rect"];
        deserializer.deserialize_struct(
            "RosOpenCvIntrinsics",
            FIELDS,
            IntrinsicParametersVisitor(std::marker::PhantomData),
        )
    }
}

#[cfg(feature = "serde-serialize")]
fn _test_intrinsics_is_serialize() {
    // Compile-time test to ensure RosOpenCvIntrinsics implements Serialize trait.
    fn implements<T: serde::Serialize>() {}
    implements::<RosOpenCvIntrinsics<f64>>();
}

#[cfg(feature = "serde-serialize")]
fn _test_intrinsics_is_deserialize() {
    // Compile-time test to ensure RosOpenCvIntrinsics implements Deserialize trait.
    fn implements<'de, T: serde::Deserialize<'de>>() {}
    implements::<RosOpenCvIntrinsics<f64>>();
}

/// The type defines termination criteria for iterative algorithms.
#[derive(Debug, Clone, Copy)]
pub enum TermCriteria {
    /// The maximum number of iterations.
    MaxIter(usize),
    /// The desired accuracy at which the iterative algorithm stops.
    Eps(f64),
}
