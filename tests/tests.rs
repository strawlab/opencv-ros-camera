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

mod pod_tests {
    use nalgebra::{Matrix3, Vector5};
    use opencv_ros_camera::*;

    fn sample() -> RosOpenCvIntrinsics<f64> {
        RosOpenCvIntrinsics::from_params_with_distortion(
            100.0,
            0.1,
            101.0,
            320.0,
            240.0,
            Distortion::from_opencv_vec(Vector5::new(-0.3, 0.1, 0.001, -0.002, 0.05)),
        )
    }

    #[test]
    fn roundtrip_preserves_all_public_fields() {
        let cam = sample();
        let pod = RosOpenCvIntrinsicsPod::from(&cam);
        let back = RosOpenCvIntrinsics::try_from(pod).unwrap();
        assert_eq!(cam, back);
        assert_eq!(cam.is_opencv_compatible, back.is_opencv_compatible);
    }

    #[test]
    fn roundtrip_preserves_non_identity_rect() {
        let cam = sample();
        let rect = Matrix3::new(0.99, -0.01, 0.0, 0.01, 0.99, 0.0, 0.0, 0.0, 1.0);
        let cam = RosOpenCvIntrinsics::from_components(cam.p, cam.k, cam.distortion.clone(), rect)
            .unwrap();
        let back = RosOpenCvIntrinsics::try_from(RosOpenCvIntrinsicsPod::from(&cam)).unwrap();
        assert_eq!(cam.rect, back.rect);
        assert_eq!(cam, back);
    }

    #[test]
    fn column_major_layout() {
        let cam = sample();
        let pod = RosOpenCvIntrinsicsPod::from(&cam);
        // p is 3x4 column-major: index 1 is row 1, column 0.
        assert_eq!(pod.p[1], cam.p[(1, 0)]);
        assert_eq!(pod.p[3], cam.p[(0, 1)]);
        assert_eq!(pod.k[3], cam.k[(0, 1)]);
    }

    #[test]
    fn singular_rect_is_rejected() {
        let cam = sample();
        let mut pod = RosOpenCvIntrinsicsPod::from(&cam);
        pod.rect = [0.0; 9];
        assert!(matches!(
            RosOpenCvIntrinsics::try_from(pod),
            Err(Error::InvalidInput)
        ));
    }
}
