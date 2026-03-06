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
) -> impl Iterator<Item = rerun::Color> {
    let grad = colorgrad::preset::turbo();
    values.map(move |x| {
        let t = (x - min) / (max - min);
        let [r, g, b, _] = grad.at(t as f32).to_rgba8();
        rerun::Color::from_rgb(r, g, b)
    })
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();

    let base_directory = Path::new(
        args.get(1)
            .expect("Pass path to a directory with data as the first argument"),
    );
    let output_path = args.get(2).map(|s| s.as_str()).unwrap_or("result.rrd");

    let static_transforms = input::read_static_transforms(base_directory);
    let gt_poses = input::read_gt_poses(base_directory);
    let key_frames = input::read_keyframes(base_directory);
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
        "world/car/obs_point",
        &rerun::Points3D::new([[0.0, -5.0, 3.0]]).with_radii([0.0]),
    )?;

    let gt_trajectory = gt_poses
        .iter()
        .map(|(_, isometry)| point_to_rerun(&isometry.translation.vector.into()))
        .collect::<Vec<_>>();
    rec.log_static(
        "world/gt_trajectory",
        &rerun::Points3D::new(gt_trajectory)
            .with_radii([0.05])
            .with_colors([rerun::Color::from_rgb(255, 255, 255)]),
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

    let point_cloud: Vec<_> = key_frames
        .iter()
        .flat_map(|keyframe| keyframe.key_points_world.clone())
        .step_by(15)
        .collect();
    rec.log_static(
        "world/point_cloud",
        &rerun::Points3D::new(point_cloud.iter().map(point_to_rerun))
            .with_colors(color_range(
                point_cloud.iter().map(|point| point.z),
                -3.0,
                30.0,
            ))
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
                )),
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
