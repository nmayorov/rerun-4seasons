#![allow(non_snake_case)]
#![allow(dead_code)]

mod input;
mod util;

use std::env;
use std::path::Path;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();

    let base_directory = Path::new(
        args.get(1)
            .expect("Pass path to a directory with data as the first argument"),
    );

    let gt_poses = input::read_gt_poses(base_directory);
    let transforms = input::read_static_transforms(base_directory);
    let T_car_cam = transforms.T_car_imu * transforms.T_cam_imu.inverse();

    let rec = rerun::RecordingStreamBuilder::new("4seasons_visualization").save("result.rrd")?;
    let trajectory = gt_poses
        .iter()
        .map(|(_, isometry)| util::point_to_rerun(&isometry.translation.vector.into()))
        .collect::<Vec<_>>();

    rec.log_static(
        "world/trajectory",
        &rerun::Points3D::new(trajectory)
            .with_radii([0.01])
            .with_colors([rerun::Color::WHITE]),
    )?;

    let key_frames = input::read_keyframes(base_directory);

    let intrinsics = &key_frames
        .first()
        .ok_or("No single keyframe file was read")?
        .intrinsics;
    rec.log_static("world/car/cam", &util::isometry_to_rerun(&T_car_cam))?;
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
        &rerun::Points3D::new([[0.0, -5.0, 3.0]])
            .with_radii([0.01])
            .with_colors([rerun::Color::TRANSPARENT]),
    )?;

    let point_cloud_world: Vec<_> = key_frames
        .iter()
        .flat_map(|keyframe| keyframe.points_world.clone())
        .step_by(15)
        .collect();

    let colors = util::color_range(point_cloud_world.iter().map(|point| point.z), -3.0, 30.0);
    let points = point_cloud_world
        .iter()
        .map(|point| util::point_to_rerun(point))
        .collect::<Vec<_>>();

    rec.log_static(
        "world/point_cloud",
        &rerun::Points3D::new(points)
            .with_colors(colors)
            .with_radii([0.05]),
    )?;

    for keyframe in &key_frames {
        rec.set_timestamp_nanos_since_epoch("global_time", keyframe.timestamp);
        rec.log(
            "world/car",
            &util::isometry_to_rerun(&(keyframe.T_world_cam * T_car_cam.inverse())),
        )?;
        let colors =
            util::color_range(keyframe.pixel_coords.iter().map(|point| point.z), 1.0, 50.0);
        let points = keyframe
            .pixel_coords
            .iter()
            .map(|coordinates| {
                rerun::external::glam::Vec2::new(coordinates.x as f32, coordinates.y as f32)
            })
            .collect::<Vec<_>>();
        rec.log(
            "world/car/cam/key_points",
            &rerun::Points2D::new(points)
                .with_radii([2.0])
                .with_colors(colors),
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
