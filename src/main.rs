use std::env;
use std::fs::{File, read_to_string};
use std::io::{BufRead, BufReader};
use std::path::Path;
use rerun::external::glam;

struct Transforms {
    T_cam_imu: nalgebra::Isometry3<f64>,
    T_gnss_imu: nalgebra::Isometry3<f64>,
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

    rec.log_static("world/trajectory", &rerun::Points3D::new(trajectory))?;

    rec.set_timestamp_nanos_since_epoch("global_time", poses[0].0);
    rec.log(
        "world/car",
        &rerun::Boxes3D::from_half_sizes([(2.0, 4.0, 1.0)]),
    )?;
    for (time, T_word_cam) in &poses {
        rec.set_timestamp_nanos_since_epoch("global_time", *time);
        let T_world_imu = T_word_cam * static_transforms.T_cam_imu;
        rec.log("world/car", &convert_to_rerun(&T_world_imu))?;
    }

    Ok(())
}
