#[cfg(feature = "serde-serialize")]
mod serde_tests {
    use nalgebra::RealField;

    use cam_geom::intrinsic_test_utils::roundtrip_intrinsics;
    use opencv_ros_camera::*;

    fn check_roundtrip<R: RealField + serde::de::DeserializeOwned>(eps: R) {
        use std::convert::TryInto;

        let buf = include_str!("ros/camera.yaml");
        let ros_camera: RosCameraInfo<R> = serde_yaml::from_str(buf).unwrap();

        let width = ros_camera.image_width;
        let height = ros_camera.image_height;

        let named: NamedIntrinsicParameters<R> = ros_camera.try_into().unwrap();

        let cam = named.intrinsics;
        roundtrip_intrinsics(&cam, width, height, 5, 65, nalgebra::convert(eps));
    }

    #[test]
    fn roundtrip_f32() {
        check_roundtrip::<f32>(0.02f32);
    }

    #[test]
    fn roundtrip_f64() {
        check_roundtrip::<f64>(0.02);
    }
}
