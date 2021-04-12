// This module requires std.

#[cfg(feature = "serde-serialize")]
use std::io::Read;

use na::{
    allocator::Allocator,
    core::{
        dimension::{DimMin, U3},
        Matrix3, OMatrix, RowVector5,
    },
};

use na::{DefaultAllocator, DimName, RealField};
use nalgebra as na;

use crate::{Error, Result, RosOpenCvIntrinsics};

#[cfg(feature = "serde-serialize")]
use serde::{Deserialize, Serialize};

/// Camera calibration info as saved by ROS.
///
/// This is a low-level structure only intended for interoperation with ROS. To
/// convert to a more Rust-friedly type, use
/// [`NamedIntrinsicParameters::try_from()`](struct.NamedIntrinsicParameters.html#method.try_from).
/// To create an instance of this structure from a
/// [`NamedIntrinsicParameters`](struct.NamedIntrinsicParameters.html) struct,
/// use [`RosCameraInfo::from()`](struct.RosCameraInfo.html#method.from).
///
/// This structure implements the format written by `writeCalibrationYml` in ROS
/// code `camera_calibration_parsers/src/parse_yml.cpp`. It can be serialized or
/// deserialized with serde. (It is strange that writeCalibrationYml has its own
/// serializer rather than using the ROS message type
/// [`sensor_msgs/CameraInfo`](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html).)
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct RosCameraInfo<R: RealField> {
    /// The width of the image sensor (in pixels).
    pub image_width: usize,
    /// The height of the image sensor (in pixels).
    pub image_height: usize,
    /// The name of the camera
    pub camera_name: String,
    /// The camera matrix `k`.
    pub camera_matrix: RosMatrix<R>,
    /// The name of the distortion model. Only "plumb_bob" is supported.
    pub distortion_model: String,
    /// The coefficients of the distortion parameters.
    pub distortion_coefficients: RosMatrix<R>,
    /// The stereo rectification matrix.
    pub rectification_matrix: RosMatrix<R>,
    /// The projection matrix `p`.
    pub projection_matrix: RosMatrix<R>,
}

impl<R: RealField> From<NamedIntrinsicParameters<R>> for RosCameraInfo<R> {
    fn from(orig: NamedIntrinsicParameters<R>) -> Self {
        let d = &orig.intrinsics.distortion;

        let distortion = vec![
            d.radial1(),
            d.radial2(),
            d.tangential1(),
            d.tangential2(),
            d.radial3(),
        ];
        Self {
            image_width: orig.width,
            image_height: orig.height,
            camera_name: orig.name,
            camera_matrix: to_ros(orig.intrinsics.k),
            distortion_model: "plumb_bob".to_string(),
            distortion_coefficients: to_ros_matrix(1, 5, distortion.as_slice()),
            rectification_matrix: to_ros(orig.intrinsics.rect),
            projection_matrix: to_ros(orig.intrinsics.p),
        }
    }
}

/// Matrix saved by ROS.
///
/// This is a low-level structure only intended for interoperation with ROS,
/// specifically as the type of fields within the
/// [`RosCameraInfo`](struct.RosCameraInfo.html) struct.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct RosMatrix<R: RealField> {
    /// Number of rows in the matrix.
    pub rows: usize,
    /// Number of columns in the matrix.
    pub cols: usize,
    /// The data in the matrix stored as a column-major `Vec`.
    pub data: Vec<R>,
}

fn to_ros<R: RealField, SS: DimName, OS: DimName>(arr: na::OMatrix<R, SS, OS>) -> RosMatrix<R>
where
    DefaultAllocator: Allocator<R, SS, SS>,
    DefaultAllocator: Allocator<R, SS>,
    DefaultAllocator: Allocator<R, OS, SS>,
    DefaultAllocator: Allocator<R, SS, OS>,
    DefaultAllocator: Allocator<R, OS, OS>,
    DefaultAllocator: Allocator<R, OS>,
    DefaultAllocator: Allocator<(usize, usize), OS>,
    OS: DimMin<OS, Output = OS>,
{
    // need to transpose the data since na is column major and ros is row major.
    let a2 = arr.transpose();
    RosMatrix {
        rows: arr.nrows(),
        cols: arr.ncols(),
        data: a2.as_slice().to_vec(),
    }
}

#[inline]
pub(crate) fn to_ros_matrix<R: RealField>(rows: usize, cols: usize, data: &[R]) -> RosMatrix<R> {
    RosMatrix {
        rows,
        cols,
        data: Vec::from(data),
    }
}

pub(crate) fn get_nalgebra_matrix<R, D1, D2>(
    ros_matrix: &RosMatrix<R>,
) -> Result<OMatrix<R, D1, D2>>
where
    R: RealField,
    D1: DimName,
    D2: DimName,
    DefaultAllocator: Allocator<R, D1, D1>,
    DefaultAllocator: Allocator<R, D1>,
    DefaultAllocator: Allocator<R, D2, D1>,
    DefaultAllocator: Allocator<R, D1, D2>,
    DefaultAllocator: Allocator<R, D2, D2>,
    DefaultAllocator: Allocator<R, D2>,
{
    if ros_matrix.rows as usize != D1::dim() {
        return Err(Error::BadMatrixSize);
    }
    if ros_matrix.cols as usize != D2::dim() {
        return Err(Error::BadMatrixSize);
    }
    if ros_matrix.data.len() != (ros_matrix.rows * ros_matrix.cols) as usize {
        return Err(Error::BadMatrixSize);
    }
    let data_converted: Vec<R> = ros_matrix
        .data
        .clone()
        .into_iter()
        .map(na::convert)
        .collect();
    Ok(OMatrix::from_row_slice_generic(
        D1::name(),
        D2::name(),
        &data_converted,
    ))
}

/// A struct with `RosOpenCvIntrinsics`, camera name and image sensor dimensions.
///
/// This is primarily used to read YAML files saved by ROS. Create this struct
/// with the [`from_ros_yaml`](fn.from_ros_yaml.html) function.
///
/// To extract a [`RosOpenCvIntrinsics`](struct.RosOpenCvIntrinsics.html)
/// structure from this struct, use the
/// [`intrinsics`](struct.NamedIntrinsicParameters.html#structfield.intrinsics)
/// field.
///
/// See the [module-level documentation for more information](index.html).
pub struct NamedIntrinsicParameters<R: RealField> {
    /// Name of the camera.
    pub name: String,
    /// The width of the image sensor (in pixels).
    pub width: usize,
    /// The height of the image sensor (in pixels).
    pub height: usize,
    /// The intrinsic parameters.
    pub intrinsics: RosOpenCvIntrinsics<R>,
}

impl<R: RealField> std::convert::TryFrom<RosCameraInfo<R>> for NamedIntrinsicParameters<R> {
    type Error = Error;
    fn try_from(ros_camera: RosCameraInfo<R>) -> Result<NamedIntrinsicParameters<R>> {
        let intrinsics = {
            let p = get_nalgebra_matrix(&ros_camera.projection_matrix)?;
            let k: OMatrix<R, U3, U3> = get_nalgebra_matrix(&ros_camera.camera_matrix)?;
            if ros_camera.distortion_model != "plumb_bob" {
                return Err(Error::UnknownDistortionModel);
            }
            let d: RowVector5<R> = get_nalgebra_matrix(&ros_camera.distortion_coefficients)?;
            let rect: Matrix3<R> = get_nalgebra_matrix(&ros_camera.rectification_matrix)?;
            let distortion = crate::Distortion::from_opencv_vec(d.transpose());
            RosOpenCvIntrinsics::from_components(p, k, distortion, rect)?
        };
        Ok(NamedIntrinsicParameters {
            name: ros_camera.camera_name,
            width: ros_camera.image_width,
            height: ros_camera.image_height,
            intrinsics,
        })
    }
}

#[cfg(feature = "serde-serialize")]
/// Construct NamedIntrinsicParameters from ROS format YAML data.
///
/// This is a small wrapper around `serde_yaml::from_reader()` and
/// [`NamedIntrinsicParameters::try_from()`](struct.NamedIntrinsicParameters.html#method.try_from).
///
/// See the [module-level documentation for more information](index.html),
/// including an example of use.
pub fn from_ros_yaml<R, Rd>(reader: Rd) -> Result<NamedIntrinsicParameters<R>>
where
    R: RealField + serde::de::DeserializeOwned,
    Rd: Read,
{
    let ros_camera: RosCameraInfo<R> = serde_yaml::from_reader(reader)?;
    Ok(std::convert::TryInto::try_into(ros_camera)?)
}
