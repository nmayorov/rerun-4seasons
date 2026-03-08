use kornia::image::Image;
use kornia::io::png::read_image_png_mono8;
use kornia::tensor::CpuAllocator;
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
}

pub struct Pixel {
    pub u: f64,
    pub v: f64,
    pub depth: f64,
}

pub struct KeyFrame {
    pub timestamp: i64,
    pub intrinsics: CamIntrinsics,
    pub T_world_cam: nalgebra::Isometry3<f64>,
    pub key_points_pixel: Vec<Pixel>,
    pub key_points_cam: Vec<nalgebra::Point3<f64>>,
    pub key_points_world: Vec<nalgebra::Point3<f64>>,
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

pub fn read_static_transforms(base_directory: &Path) -> Transforms {
    let content = std::fs::read_to_string(base_directory.join("Transformations.txt"))
        .expect("Can't read Transformations.txt");
    let lines = content.lines().collect::<Vec<_>>();
    let T_cam_imu = (|| {
        let index = lines
            .iter()
            .position(|line| line == &"# TS_cam_imu: translation vector, rotation quaternion")?
            + 1;
        parse_transform(&lines.get(index)?.split(",").collect::<Vec<_>>())
    })()
    .expect("Error reading TS_cam_imu");
    Transforms {
        T_car_imu: nalgebra::Isometry3::from_parts(
            nalgebra::Vector3::<f64>::zeros().into(),
            nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::PI),
        ),
        T_cam_imu,
    }
}

pub fn read_gt_poses(base_directory: &Path) -> Vec<(i64, nalgebra::Isometry3<f64>)> {
    let content = std::fs::read_to_string(base_directory.join("GNSSPoses.txt"))
        .expect("Error reading GNSSPoses.txt file found");
    let lines = content.lines().skip(1);
    let mut result = Vec::new();
    for line in lines {
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

pub fn read_keyframes(base_directory: &Path) -> Vec<KeyFrame> {
    let mut result = Vec::new();
    let keyframe_path = base_directory.join("KeyFrameData");
    let paths = std::fs::read_dir(keyframe_path).expect("No KeyFrameData directory");
    for path in paths {
        let entry = path.expect("Error reading KeyFrameData directory");
        if let Some(keyframe) = read_keyframe_file(&entry.path()) {
            result.push(keyframe);
        }
    }
    result.sort_by_key(|kf| kf.timestamp);
    result
}

fn read_keyframe_file(path: &Path) -> Option<KeyFrame> {
    let content =
        std::fs::read_to_string(path).expect("Error reading file in KeyFrameData directory");
    let lines = content.lines().collect::<Vec<_>>();
    let timestamp = {
        let index = lines.iter().position(|line| line == &"# timestamp")? + 1;
        lines[index].parse::<i64>().ok()?
    };

    let intrinsics = {
        let index = lines
            .iter()
            .position(|line| line == &"# fx, fy, cx, cy, width, height, npoints")?
            + 1;
        let items = lines.get(index)?.split(",").collect::<Vec<_>>();
        if items.len() < 6 {
            return None;
        }
        CamIntrinsics {
            fx: items[0].parse().ok()?,
            fy: items[1].parse().ok()?,
            cx: items[2].parse().ok()?,
            cy: items[3].parse().ok()?,
            width: items[4].parse().ok()?,
            height: items[5].parse().ok()?,
        }
    };

    let T_world_cam = {
        let index = lines
            .iter()
            .position(|line| line == &"# camToWorld: translation vector, rotation quaternion")?
            + 1;
        let items = lines.get(index)?.split(",").collect::<Vec<_>>();
        parse_transform(&items)?
    };

    let (pixel_coords, points_cam, points_world) = {
        let start = lines
            .iter()
            .position(|line| line == &"# Point Cloud Data : ")?
            + 3;
        let mut pixel_coords = Vec::new();
        let mut points_cam = Vec::new();
        let mut points_world = Vec::new();
        for &line in lines.get(start..)?.iter().step_by(2) {
            let items = line.split(",").collect::<Vec<_>>();
            if items.len() < 3 {
                return None;
            }
            let u = items[0].parse::<f64>().ok()?;
            let v = items[1].parse::<f64>().ok()?;
            let depth = 1.0 / items[2].parse::<f64>().ok()?;
            pixel_coords.push(Pixel { u, v, depth });
            let point_cam = nalgebra::Point3::new(
                (u - intrinsics.cx) * depth / intrinsics.fx,
                (v - intrinsics.cy) * depth / intrinsics.fy,
                depth,
            );
            points_cam.push(point_cam);
            points_world.push(T_world_cam * point_cam);
        }
        (pixel_coords, points_cam, points_world)
    };

    Some(KeyFrame {
        timestamp,
        intrinsics,
        T_world_cam,
        key_points_pixel: pixel_coords,
        key_points_cam: points_cam,
        key_points_world: points_world,
    })
}

pub fn read_images(
    base_directory: &Path,
) -> impl Iterator<Item = (i64, Image<u8, 1, CpuAllocator>)> {
    let items = std::fs::read_dir(base_directory.join("undistorted_images").join("cam0"))
        .expect("Can't read undistorted_images/cam0 directory");
    items.filter_map(|entry| {
        let path = entry.ok()?.path();
        let timestamp = path.file_stem()?.to_str()?.parse::<i64>().ok()?;
        let image = read_image_png_mono8(path).ok()?;
        Some((timestamp, image))
    })
}
