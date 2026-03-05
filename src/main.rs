#![allow(non_snake_case)]

mod input;
mod output;
mod style;

use rerun::external::glam;
use std::fs::File;
use std::path::Path;
use std::{env, fs};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();

    let path = Path::new(&args[1]);

    let poses = input::read_poses(&File::open(path.join("GNSSPoses.txt"))?);
    let static_transforms = input::read_transforms(&path.join("Transformations.txt"))?;

    let rec =
        rerun::RecordingStreamBuilder::new("4seasons_visualization.rrd").save("result.rrd")?;
    let trajectory = poses
        .iter()
        .map(|(_, isometry)| {
            glam::Vec3::new(
                isometry.translation.x as f32,
                isometry.translation.y as f32,
                isometry.translation.z as f32,
            )
        })
        .collect::<Vec<_>>();

    rec.log_static("world/trajectory", &rerun::LineStrips3D::new([trajectory]))?;

    for (time, T_word_cam) in &poses {
        rec.set_timestamp_nanos_since_epoch("global_time", *time);
        let T_world_imu = T_word_cam * static_transforms.T_cam_imu;
        rec.log("world/car", &output::isometry_to_rerun(&T_world_imu))?;
    }

    let key_frames = {
        let mut key_frames = Vec::new();
        let keyframe_path = path.join("KeyFrameData");
        let paths = fs::read_dir(keyframe_path)?;
        for path in paths {
            if let Ok(entry) = path {
                let file = File::open(entry.path())?;
                key_frames.push(input::read_keyframes(file));
            }
        }
        key_frames
    };

    let world_points: Vec<_> = key_frames
        .iter()
        .map(|keyframe| {
            let mut world_points = Vec::new();
            for point in &keyframe.points_cam {
                world_points.push(keyframe.T_world_cam * point);
            }
            world_points
        })
        .flatten()
        .step_by(15)
        .collect();

    let colors = style::color_by_z(&world_points);
    let points = world_points
        .iter()
        .map(|point| output::point_to_rerun(point))
        .collect::<Vec<_>>();

    rec.log_static(
        "world/point_cloud",
        &rerun::Points3D::new(points)
            .with_colors(colors)
            .with_radii([0.2]),
    )?;

    for keyframe in &key_frames {
        rec.set_timestamp_nanos_since_epoch("global_time", keyframe.timestamp);
        let points = keyframe.points_cam
            .iter()
            .map(|point| output::point_to_rerun(point))
            .collect::<Vec<_>>();
        rec.log(
            "world/car/cam/point_cloud",
            &rerun::Points3D::new(points)
                .with_radii([0.2])
                .with_colors([rerun::Color::from_rgb(255, 255, 0)]),
        )?;
    }

    let intrinsics = &key_frames.first().unwrap().intrinsics;
    rec.log_static(
        "world/car/cam",
        &output::isometry_to_rerun(&static_transforms.T_cam_imu.inverse()),
    )?;
    rec.log_static(
        "world/car/cam",
        &rerun::Pinhole::from_focal_length_and_resolution(
            [intrinsics.fx as f32, intrinsics.fy as f32],
            [intrinsics.width as f32, intrinsics.height as f32],
        )
        .with_principal_point([intrinsics.cx as f32, intrinsics.cy as f32])
        .with_image_plane_distance(2.0),
    )?;
    output::add_images(
        &rec,
        "world/car/cam",
        &path.join("undistorted_images").join("cam0"),
    );

    Ok(())
}
