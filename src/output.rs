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
