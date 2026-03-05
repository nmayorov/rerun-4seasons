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
    pub T_cam_imu: nalgebra::Isometry3<f64>,
    pub T_gnss_imu: nalgebra::Isometry3<f64>,
}

pub struct KeyFrame {
    pub timestamp: i64,
    pub intrinsics: CamIntrinsics,
    pub pixel_coords: Vec<nalgebra::Vector3<f64>>,
    pub points_world: Vec<nalgebra::Point3<f64>>,
}

pub fn parse_transform(items: &[&str]) -> nalgebra::Isometry3<f64> {
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

pub fn read_transforms(path: &Path) -> Result<Transforms, std::io::Error> {
    let content = std::fs::read_to_string(path)?;
    let lines = content.lines().collect::<Vec<_>>();
    Ok(Transforms {
        T_cam_imu: parse_transform(&lines[4].split(",").collect::<Vec<_>>()),
        T_gnss_imu: parse_transform(&lines[10].split(",").collect::<Vec<_>>()),
    })
}

pub fn read_poses(file: &File) -> Vec<(i64, nalgebra::Isometry3<f64>)> {
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
        parse_transform(&items)
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
        pixel_coords,
        points_world,
    }
}
