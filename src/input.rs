use std::fs::File;
use std::io::{BufRead, BufReader, Read};
use std::path::Path;

pub struct CamIntrinsics {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub width: f64,
    pub height: f64,
}

pub struct Transforms {
    pub T_car_imu: nalgebra::Isometry3<f64>,
    pub T_cam_imu: nalgebra::Isometry3<f64>,
    pub T_gnss_imu: nalgebra::Isometry3<f64>,
}

pub struct KeyFrame {
    pub timestamp: i64,
    pub intrinsics: CamIntrinsics,
    pub T_world_cam: nalgebra::Isometry3<f64>,
    pub pixel_coords: Vec<nalgebra::Vector3<f64>>,
    pub points_world: Vec<nalgebra::Point3<f64>>,
}

fn parse_transform(items: &[&str]) -> Option<nalgebra::Isometry3<f64>> {
    if items.len() < 7 {
        return None;
    }
    let translation = nalgebra::Translation3::new(
        items[0].parse().ok()?,
        items[1].parse().ok()?,
        items[2].parse().ok()?,
    );
    let rotation = nalgebra::Quaternion::new(
        items[6].parse().ok()?,
        items[3].parse().ok()?,
        items[4].parse().ok()?,
        items[5].parse().ok()?,
    );
    Some(nalgebra::Isometry3::from_parts(
        translation,
        nalgebra::UnitQuaternion::from_quaternion(rotation),
    ))
}

pub fn read_transforms(path: &Path) -> Result<Transforms, std::io::Error> {
    let content = std::fs::read_to_string(path)?;
    let lines = content.lines().collect::<Vec<_>>();
    Ok(Transforms {
        T_car_imu: nalgebra::Isometry3::from_parts(
            nalgebra::Vector3::<f64>::zeros().into(),
            nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::PI),
        ),
        T_cam_imu: parse_transform(&lines[4].split(",").collect::<Vec<_>>()).unwrap(),
        T_gnss_imu: parse_transform(&lines[10].split(",").collect::<Vec<_>>()).unwrap(),
    })
}

pub fn read_gt_poses(base_directory: &Path) -> Vec<(i64, nalgebra::Isometry3<f64>)> {
    let file =
        File::open(base_directory.join("GNSSPoses.txt")).expect("No GNSSPoses.txt file found");
    let reader = BufReader::new(file);
    let lines = reader.lines().skip(1);
    let mut result = Vec::new();
    for line in lines {
        let line = line.expect("Error parsing GNSSPoses.txt");
        let items = line.split(",").collect::<Vec<_>>();
        if items.len() < 8 {
            panic!("Error parsing GNSSPoses.txt");
        }
        let timestamp = items[0]
            .parse::<i64>()
            .expect("Error parsing GNSSPoses.txt");
        let transform = parse_transform(&items[1..8]).expect("Error parsing GNSSPoses.txt");
        result.push((timestamp, transform));
    }
    result
}

pub fn read_keyframes(file: File) -> KeyFrame {
    let content = {
        let mut file = file;
        let mut content = String::new();
        file.read_to_string(&mut content).unwrap();
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
        parse_transform(&items).unwrap()
    };

    let (pixel_coords, points_world) = {
        let start = lines
            .iter()
            .position(|line| line == &"# Point Cloud Data : ")
            .unwrap()
            + 3;
        let mut pixel_coords = Vec::new();
        let mut points_world = Vec::new();
        for &line in lines[start..].iter().step_by(2) {
            let items = line.split(",").collect::<Vec<_>>();
            let u = items[0].parse::<f64>().unwrap();
            let v = items[1].parse::<f64>().unwrap();
            let depth = 1.0 / items[2].parse::<f64>().unwrap();
            pixel_coords.push(nalgebra::Vector3::new(u, v, depth));

            let point_cam = nalgebra::Point3::new(
                (u - intrinsics.cx) * depth / intrinsics.fx,
                (v - intrinsics.cy) * depth / intrinsics.fy,
                depth,
            );
            points_world.push(T_world_cam * point_cam);
        }
        (pixel_coords, points_world)
    };

    KeyFrame {
        timestamp,
        intrinsics,
        T_world_cam,
        pixel_coords,
        points_world,
    }
}
