use std::fs;
use std::path::Path;
use kornia::io::png::read_image_png_mono8;
use rerun::external::glam;

pub fn isometry_to_rerun(transform: &nalgebra::Isometry3<f64>) -> rerun::Transform3D {
    let t = transform.translation.vector;
    let q = transform.rotation.quaternion().coords;
    rerun::Transform3D::from_translation_rotation(
        [t.x as f32, t.y as f32, t.z as f32],
        rerun::Quaternion::from_xyzw([q.x as f32, q.y as f32, q.z as f32, q.w as f32]),
    )
}

pub fn point_to_rerun(vector: &nalgebra::Point3<f64>) -> glam::Vec3 {
    glam::Vec3::new(vector.x as f32, vector.y as f32, vector.z as f32)
}

pub fn add_images(rec: &rerun::RecordingStream, entity: &str, directory: &Path) {
    let items = fs::read_dir(directory).unwrap();
    for path in items {
        if let Ok(entry) = path {
            let path = entry.path();
            rec.set_timestamp_nanos_since_epoch(
                "global_time",
                path.file_stem()
                    .unwrap()
                    .to_str()
                    .unwrap()
                    .parse::<i64>()
                    .unwrap(),
            );
            let image = read_image_png_mono8(path).unwrap();
            rec.log(
                entity,
                &rerun::Image::from_elements(
                    image.as_slice(),
                    image.size().into(),
                    rerun::ColorModel::L,
                ),
            ).unwrap();
        }
    }
}
