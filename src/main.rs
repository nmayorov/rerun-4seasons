#![allow(non_snake_case)]
#![allow(dead_code)]

mod input;

use colorgrad::Gradient;
use std::env;
use std::path::Path;

fn isometry_to_rerun(transform: &nalgebra::Isometry3<f64>) -> rerun::Transform3D {
    let t = transform.translation.vector;
    let q = transform.rotation.quaternion().coords;
    rerun::Transform3D::from_translation_rotation(
        [t.x as f32, t.y as f32, t.z as f32],
        rerun::Quaternion::from_xyzw([q.x as f32, q.y as f32, q.z as f32, q.w as f32]),
    )
}

fn point_to_rerun(vector: &nalgebra::Point3<f64>) -> rerun::Vec3D {
    rerun::Vec3D::new(vector.x as f32, vector.y as f32, vector.z as f32)
}

fn color_range(
    values: impl Iterator<Item = f64>,
    min: f64,
    max: f64,
    colormap: impl Gradient,
) -> impl Iterator<Item = rerun::Color> {
    values.map(move |x| {
        let t = (x - min) / (max - min);
        let [r, g, b, _] = colormap.at(t as f32).to_rgba8();
        rerun::Color::from_rgb(r, g, b)
    })
}

fn position_to_lat_lon(
    position_world: &nalgebra::Point3<f64>,
    scale: f64,
    static_transforms: &input::Transforms,
) -> (f64, f64) {
    let position_ecef = static_transforms.T_ecef_enu
        * static_transforms.T_world_enu.inverse()
        * static_transforms.T_S_AS
        * (scale * position_world);
    let ecef_point = nav_types::ECEF::new(position_ecef.x, position_ecef.y, position_ecef.z);
    let lla_point = nav_types::WGS84::from(ecef_point);
    (lla_point.latitude_degrees(), lla_point.longitude_degrees())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    const GLOBAL_POINT_CLOUD_SUBSAMPLE: usize = 2;
    const LOCAL_POINT_CLOUD_WINDOW: usize = 100;

    let args: Vec<String> = env::args().collect();

    let base_directory = Path::new(
        args.get(1)
            .expect("Pass path to a directory with data as the first argument"),
    );
    let output_path = args.get(2).map(|s| s.as_str()).unwrap_or("result.rrd");

    let static_transforms = input::read_static_transforms(base_directory);
    let gt_poses = input::read_gt_poses(base_directory);
    let key_frames = input::read_keyframes(base_directory);

    if key_frames.len() != gt_poses.len() {
        panic!("Number of key frames must be the same as the number of poses in GNSSPoses.txt");
    }

    let T_car_cam = static_transforms.T_car_imu * static_transforms.T_cam_imu.inverse();
    let intrinsics = &key_frames
        .first()
        .ok_or("No single keyframe file was read")?
        .intrinsics;

    let rec = rerun::RecordingStreamBuilder::new("4seasons_visualization").save(output_path)?;
    rec.log_static("world/car/cam", &isometry_to_rerun(&T_car_cam))?;
    rec.log_static(
        "world/car/cam",
        &rerun::Pinhole::from_focal_length_and_resolution(
            [intrinsics.fx as f32, intrinsics.fy as f32],
            [intrinsics.width as f32, intrinsics.height as f32],
        )
        .with_principal_point([intrinsics.cx as f32, intrinsics.cy as f32])
        .with_image_plane_distance(2.0),
    )?;
    rec.log_static(
        "world/car/observation_point",
        &rerun::Points3D::new([[0.0, -5.0, 3.0]]).with_radii([0.0]),
    )?;

    let gt_trajectory = gt_poses
        .iter()
        .map(|(_, T_world_cam, _)| point_to_rerun(&T_world_cam.translation.vector.into()))
        .collect::<Vec<_>>();
    rec.log_static(
        "world/gt_trajectory",
        &rerun::Points3D::new(gt_trajectory)
            .with_radii([0.05])
            .with_colors([rerun::Color::WHITE]),
    )?;

    let vio_trajectory = key_frames
        .iter()
        .map(|keyframe| point_to_rerun(&keyframe.T_world_cam.translation.vector.into()))
        .collect::<Vec<_>>();
    rec.log_static(
        "world/vio_trajectory",
        &rerun::Points3D::new(vio_trajectory)
            .with_radii([0.05])
            .with_colors([rerun::Color::from_rgb(255, 0, 255)]),
    )?;

    let geo_points = gt_poses
        .iter()
        .map(|&(timestamp, T_world_camp, scale)| {
            (
                timestamp,
                position_to_lat_lon(
                    &T_world_camp.translation.vector.into(),
                    scale,
                    &static_transforms,
                ),
            )
        })
        .collect::<Vec<_>>();

    for i in 0..geo_points.len() {
        let from = i.saturating_sub(100);
        let points = geo_points[from..=i].iter().map(|(_, point)| point);
        let n = points.len();
        let radii = (0..n).map(|i| 2.0 + (i as f32) / (n as f32) * 8.0);
        let grad = colorgrad::preset::cool();
        let colors = (0..n).map(|i| {
            let [r, g, b, _] = grad.at(1.0 - (i as f32) / (n as f32)).to_rgba8();
            rerun::Color::from_rgb(r, g, b)
        });
        rec.set_timestamp_nanos_since_epoch("global_time", geo_points[i].0);
        rec.log(
            "geo_position",
            &rerun::GeoPoints::from_lat_lon(points)
                .with_radii(radii.map(rerun::Radius::new_ui_points))
                .with_colors(colors),
        )?;
    }

    let global_point_cloud = gt_poses
        .iter()
        .zip(&key_frames)
        .flat_map(|((_, T_world_cam, _), keyframe)| {
            keyframe
                .key_points_cam
                .iter()
                .map(move |point| T_world_cam * point)
        })
        .step_by(GLOBAL_POINT_CLOUD_SUBSAMPLE);

    rec.log_static(
        "world/global_point_cloud",
        &rerun::Points3D::new(global_point_cloud.map(|point| point_to_rerun(&point)))
            .with_colors([rerun::Color::from_rgb(255, 100, 0)])
            .with_radii([0.05]),
    )?;

    for keyframe in &key_frames {
        rec.set_timestamp_nanos_since_epoch("global_time", keyframe.timestamp);
        rec.log(
            "world/car",
            &isometry_to_rerun(&(keyframe.T_world_cam * T_car_cam.inverse())),
        )?;
        let image_points = keyframe
            .key_points_pixel
            .iter()
            .map(|pixel| rerun::external::glam::Vec2::new(pixel.u as f32, pixel.v as f32));
        rec.log(
            "world/car/cam/key_points",
            &rerun::Points2D::new(image_points)
                .with_radii([2.0])
                .with_colors(color_range(
                    keyframe.key_points_pixel.iter().map(|pixel| pixel.depth),
                    1.0,
                    50.0,
                    colorgrad::preset::magma(),
                )),
        )?;
    }

    for i in 0..key_frames.len() {
        let start = i.saturating_sub(LOCAL_POINT_CLOUD_WINDOW / 2);
        let end = (start + LOCAL_POINT_CLOUD_WINDOW).min(key_frames.len());
        let points = key_frames[start..end]
            .iter()
            .map(|keyframe| keyframe.key_points_world.iter())
            .flatten();

        rec.set_timestamp_nanos_since_epoch("global_time", key_frames[i].timestamp);
        rec.log(
            "world/local_point_cloud",
            &rerun::Points3D::new(points.clone().map(point_to_rerun))
                .with_colors(color_range(
                    points.map(|point| point.z),
                    -3.0,
                    30.0,
                    colorgrad::preset::cool(),
                ))
                .with_radii([0.05]),
        )?;
    }

    rec.log_static(
        "metrics/vio_error",
        &rerun::SeriesLines::new().with_names([
            "cross_track, m",
            "along_track, m",
            "vertical, m",
            "roll, deg",
            "pitch, deg",
            "yaw, deg",
        ]),
    )?;
    for ((timestamp, T_world_cam, _), key_frame) in gt_poses.iter().zip(&key_frames) {
        let T_world_car = T_world_cam * T_car_cam.inverse();
        let T_world_vio = key_frame.T_world_cam * T_car_cam.inverse();
        let T_car_vio = T_world_car.inverse() * T_world_vio;
        let (roll, pitch, yaw) = T_car_vio.rotation.euler_angles();
        rec.set_timestamp_nanos_since_epoch("global_time", *timestamp);
        rec.log(
            "metrics/vio_error",
            &rerun::Scalars::new([
                T_car_vio.translation.x,
                T_car_vio.translation.y,
                T_car_vio.translation.z,
                roll.to_degrees(),
                pitch.to_degrees(),
                yaw.to_degrees(),
            ]),
        )?;
    }

    for (timestamp, image) in input::read_images(base_directory) {
        rec.set_timestamp_nanos_since_epoch("global_time", timestamp);
        rec.log(
            "world/car/cam/image",
            &rerun::Image::from_elements(
                image.as_slice(),
                image.size().into(),
                rerun::ColorModel::L,
            ),
        )?;
    }

    Ok(())
}
