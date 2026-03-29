#![allow(non_snake_case)]
#![allow(dead_code)]

mod input;

use colorgrad::Gradient;
use std::env;
use std::fs::read_to_string;
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
    let args: Vec<String> = env::args().collect();
    let config_content =
        read_to_string(&args.get(1).expect("Pass path to a TOML configuration file"))
            .expect("Can't read configuration file");
    let config =
        toml::from_str::<toml::Value>(&config_content).expect("Can't parse given file as TOML");
    let base_directory = Path::new(
        config["data_directory"]
            .as_str()
            .expect("Can't parse data_directory from config"),
    );
    let output_path = config["output_path"]
        .as_str()
        .expect("Can't parse output_path from config");

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

    if config["global_point_cloud"]["enable"].as_bool().unwrap() {
        let subsample = config["global_point_cloud"]["subsample"]
            .as_integer()
            .unwrap() as usize;
        let global_point_cloud = gt_poses
            .iter()
            .zip(&key_frames)
            .flat_map(|((_, T_world_cam, _), keyframe)| {
                keyframe
                    .key_points_cam
                    .iter()
                    .map(move |point| T_world_cam * point)
            })
            .step_by(subsample);

        rec.log_static(
            "world/global_point_cloud",
            &rerun::Points3D::new(global_point_cloud.map(|point| point_to_rerun(&point)))
                .with_colors([rerun::Color::from_rgb(255, 100, 0)])
                .with_radii([0.05]),
        )?;
    }

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

    if config["local_point_cloud"]["enable"].as_bool().unwrap() {
        let window = config["local_point_cloud"]["frame_count_window"]
            .as_integer()
            .unwrap() as usize;
        for i in 0..key_frames.len() {
            let start = i.saturating_sub(window / 4);
            let end = (start + 3 * window / 4).min(key_frames.len());
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

    if config["video"]["enable"].as_bool().unwrap() {
        let fps = config["video"]["fps"].as_integer().unwrap();
        let step = 1_000_000_000 / fps;

        let mut previous_timestamp = None;
        for (timestamp, image) in input::read_images(base_directory) {
            let log = if let Some(previous_timestamp) = previous_timestamp {
                timestamp >= previous_timestamp + step
            } else {
                true
            };
            if log {
                rec.set_timestamp_nanos_since_epoch("global_time", timestamp);
                rec.log(
                    "world/car/cam/image",
                    &rerun::Image::from_elements(
                        image.as_raw(),
                        image.dimensions().into(),
                        rerun::ColorModel::L,
                    ),
                )?;
                previous_timestamp = Some(timestamp);
            }
        }
    }

    Ok(())
}
