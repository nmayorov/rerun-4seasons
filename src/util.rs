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

pub fn color_range(values: impl Iterator<Item = f64>, min: f64, max: f64) -> Vec<rerun::Color> {
    values
        .map(|x| {
            let t = (x - min) / (max - min);
            select_color_turbo(t as f32)
        })
        .collect()
}

fn select_color_turbo(t: f32) -> rerun::Color {
    let r = (34.61
        + t * (1172.33 - t * (10793.56 - t * (33300.12 - t * (38394.49 - t * 14825.05)))))
        .clamp(0.0, 255.0) as u8;
    let g = (23.31 + t * (557.33 + t * (1225.33 - t * (3574.96 - t * (1073.77 + t * 707.56)))))
        .clamp(0.0, 255.0) as u8;
    let b = (27.2 + t * (3211.1 - t * (15327.97 - t * (27814.0 - t * (22569.18 - t * 6838.66)))))
        .clamp(0.0, 255.0) as u8;
    rerun::Color::from_rgb(r, g, b)
}
