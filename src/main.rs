#![allow(non_snake_case)]

mod input;
mod output;

use rerun::external::glam;
use std::fs::File;
use std::path::Path;
use std::{env, fs};

fn color_by_z(points: &[nalgebra::Point3<f64>]) -> Vec<rerun::Color> {
    // let z_min = points.iter().map(|p| p.z).fold(f64::INFINITY, f64::min);
    // let z_max = points.iter().map(|p| p.z).fold(f64::NEG_INFINITY, f64::max);

    // let mean = points.iter().map(|p| p.z).sum::<f64>() / points.len() as f64;
    // let std =
    //     (points.iter().map(|p| (p.z - mean).powi(2)).sum::<f64>() / points.len() as f64).sqrt();
    // let z_min = mean - 2.0 * std;
    // let z_max = mean + 2.0 * std;

    let z_min = 0.0;
    let z_max = 30.0;

    points
        .iter()
        .map(|p| {
            let t = ((p.z - z_min) / (z_max - z_min)) as f32;
            turbo(t)
        })
        .collect()
}

// Turbo colormap approximation — much better than rainbow
fn turbo(t: f32) -> rerun::Color {
    let r = (34.61
        + t * (1172.33 - t * (10793.56 - t * (33300.12 - t * (38394.49 - t * 14825.05)))))
        .clamp(0.0, 255.0) as u8;
    let g = (23.31 + t * (557.33 + t * (1225.33 - t * (3574.96 - t * (1073.77 + t * 707.56)))))
        .clamp(0.0, 255.0) as u8;
    let b = (27.2 + t * (3211.1 - t * (15327.97 - t * (27814.0 - t * (22569.18 - t * 6838.66)))))
        .clamp(0.0, 255.0) as u8;
    rerun::Color::from_rgb(r, g, b)
}

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

    rec.set_timestamp_nanos_since_epoch("global_time", poses[0].0);
    // rec.log(
    //     "world/car",
    //     &rerun::Boxes3D::from_half_sizes([(2.0, 4.0, 1.0)])
    //         .with_radii([0.2])
    //         .with_colors([rerun::Color::WHITE]),
    // )?;
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

    let colors = color_by_z(&world_points);
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
