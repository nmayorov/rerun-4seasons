use nalgebra as na;
use rerun::external::glam;
use std::fs::{File, read_to_string};
use std::io::{BufRead, BufReader, Read};
use std::path::Path;
use std::{env, fs};

struct Transforms {
    T_cam_imu: nalgebra::Isometry3<f64>,
    T_gnss_imu: nalgebra::Isometry3<f64>,
}

struct CamIntrinsics {
    fx: f64,
    fy: f64,
    cx: f64,
    cy: f64,
    width: f64,
    height: f64,
}

fn parse_transform(items: &[&str]) -> nalgebra::Isometry3<f64> {
    let translation = nalgebra::Translation3::new(
        items[0].parse().unwrap(),
        items[1].parse().unwrap(),
        items[2].parse().unwrap(),
    );
    let rotation = nalgebra::Quaternion::new(
        items[6].parse().unwrap(),
        items[3].parse().unwrap(),
        items[4].parse().unwrap(),
        items[5].parse().unwrap(),
    );
    nalgebra::Isometry3::from_parts(
        translation,
        nalgebra::UnitQuaternion::from_quaternion(rotation),
    )
}

fn convert_to_rerun(transform: &nalgebra::Isometry3<f64>) -> rerun::Transform3D {
    let t = transform.translation.vector;
    let q = transform.rotation.quaternion().coords;

    rerun::Transform3D::from_translation_rotation(
        [t.x as f32, t.y as f32, t.z as f32],
        rerun::Quaternion::from_xyzw([q.x as f32, q.y as f32, q.z as f32, q.w as f32]),
    )
}

fn point_to_rerun(vector: &na::Point3<f64>) -> glam::Vec3 {
    glam::Vec3::new(vector.x as f32, vector.y as f32, vector.z as f32)
}

fn parse_poses(file: &File) -> Vec<(i64, nalgebra::Isometry3<f64>)> {
    let reader = BufReader::new(file);
    let mut lines = reader.lines();
    lines.next();

    let mut result = Vec::new();
    for line in lines {
        if let Ok(line) = line {
            let items = line.split(",").collect::<Vec<_>>();
            let transform = parse_transform(&items[1..8]);
            result.push((items[0].parse::<i64>().unwrap(), transform));
        }
    }

    result
}

fn parse_transforms(path: &Path) -> Result<Transforms, std::io::Error> {
    let content = read_to_string(path)?;
    let lines = content.lines().collect::<Vec<_>>();
    Ok(Transforms {
        T_cam_imu: parse_transform(&lines[4].split(",").collect::<Vec<_>>()),
        T_gnss_imu: parse_transform(&lines[10].split(",").collect::<Vec<_>>()),
    })
}

fn parse_keyframes(file: File) -> (i64, na::Isometry3<f64>, Vec<na::Point3<f64>>) {
    let content = {
        let mut file = file;
        let mut content = String::new();
        file.read_to_string(&mut content);
        content
    };
    let lines = content.lines().collect::<Vec<_>>();

    let timestamp = {
        let index = lines
            .iter()
            .position(|line| line == &"# timestamp")
            .unwrap()
            + 1;
        lines[index].parse::<i64>().unwrap()
    };

    let intrinsics = {
        let index = lines
            .iter()
            .position(|line| line == &"# fx, fy, cx, cy, width, height, npoints")
            .unwrap()
            + 1;
        let items = lines[index].split(",").collect::<Vec<_>>();
        CamIntrinsics {
            fx: items[0].parse().unwrap(),
            fy: items[1].parse().unwrap(),
            cx: items[2].parse().unwrap(),
            cy: items[3].parse().unwrap(),
            width: items[4].parse().unwrap(),
            height: items[5].parse().unwrap(),
        }
    };

    let T_world_cam = {
        let index = lines
            .iter()
            .position(|line| line == &"# camToWorld: translation vector, rotation quaternion")
            .unwrap()
            + 1;
        let items = lines[index].split(",").collect::<Vec<_>>();
        parse_transform(&items)
    };

    let points_cam = {
        let start = lines
            .iter()
            .position(|line| line == &"# Point Cloud Data : ")
            .unwrap()
            + 3;
        let mut points = Vec::new();
        for &line in lines[start..].iter().step_by(2) {
            let items = line.split(",").collect::<Vec<_>>();
            let u = items[0].parse::<f64>().unwrap();
            let v = items[1].parse::<f64>().unwrap();
            let inv_depth = items[2].parse::<f64>().unwrap();
            points.push(na::Point3::new(
                (u - intrinsics.cx) / (intrinsics.fx * inv_depth),
                (v - intrinsics.cy) / (intrinsics.fy * inv_depth),
                1.0 / inv_depth,
            ));
        }
        points
    };

    (timestamp, T_world_cam, points_cam)
}

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

    let poses = parse_poses(&File::open(path.join("GNSSPoses.txt"))?);
    let static_transforms = parse_transforms(&path.join("Transformations.txt")).unwrap();

    let rec = rerun::RecordingStreamBuilder::new("4seasons_visualization").connect_grpc()?;
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

    rec.log_static(
        "world/trajectory",
        &rerun::Points3D::new(trajectory)
            .with_colors([rerun::Color::WHITE])
            .with_radii([0.2]),
    )?;
    rec.log_static(
        "world/car/cam",
        &convert_to_rerun(&static_transforms.T_cam_imu.inverse()),
    )?;

    rec.set_timestamp_nanos_since_epoch("global_time", poses[0].0);
    rec.log(
        "world/car",
        &rerun::Boxes3D::from_half_sizes([(2.0, 4.0, 1.0)])
            .with_radii([0.2])
            .with_colors([rerun::Color::WHITE]),
    )?;
    for (time, T_word_cam) in &poses {
        rec.set_timestamp_nanos_since_epoch("global_time", *time);
        let T_world_imu = T_word_cam * static_transforms.T_cam_imu;
        rec.log("world/car", &convert_to_rerun(&T_world_imu))?;
    }

    let point_cloud = {
        let mut point_cloud = Vec::new();
        let keyframe_path = path.join("KeyFrameData");
        let paths = fs::read_dir(keyframe_path).unwrap();
        for path in paths {
            if let Ok(entry) = path {
                let file = File::open(entry.path())?;
                point_cloud.push(parse_keyframes(file));
            }
        }
        point_cloud
    };

    let world_points: Vec<_> = point_cloud
        .iter()
        .map(|(timestamp, T_world_cam, points)| {
            let mut world_points = Vec::new();
            for point in points {
                world_points.push(T_world_cam * point);
            }
            world_points
        })
        .flatten()
        .step_by(15)
        .collect();

    let colors = color_by_z(&world_points);
    let points = world_points
        .iter()
        .map(|point| point_to_rerun(point))
        .collect::<Vec<_>>();

    rec.log_static(
        "world/point_cloud",
        &rerun::Points3D::new(points)
            .with_colors(colors)
            .with_radii([0.2]),
    )?;

    for (timestamp, _, points) in point_cloud {
        rec.set_timestamp_nanos_since_epoch("global_time", timestamp);
        let points = points
            .iter()
            .map(|point| point_to_rerun(point))
            .collect::<Vec<_>>();
        rec.log(
            "world/car/cam/point_cloud",
            &rerun::Points3D::new(points)
                .with_radii([0.2])
                .with_colors([rerun::Color::from_rgb(255, 255, 0)]),
        );
    }

    Ok(())
}
