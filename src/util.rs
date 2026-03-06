use colorgrad::Gradient;

pub fn isometry_to_rerun(transform: &nalgebra::Isometry3<f64>) -> rerun::Transform3D {
    let t = transform.translation.vector;
    let q = transform.rotation.quaternion().coords;
    rerun::Transform3D::from_translation_rotation(
        [t.x as f32, t.y as f32, t.z as f32],
        rerun::Quaternion::from_xyzw([q.x as f32, q.y as f32, q.z as f32, q.w as f32]),
    )
}

pub fn point_to_rerun(vector: &nalgebra::Point3<f64>) -> rerun::Vec3D {
    rerun::Vec3D::new(vector.x as f32, vector.y as f32, vector.z as f32)
}

pub fn color_range(
    values: impl Iterator<Item = f64>,
    min: f64,
    max: f64,
) -> impl Iterator<Item = rerun::Color> {
    let grad = colorgrad::preset::turbo();
    values.map(move |x| {
        let t = (x - min) / (max - min);
        let [r, g, b, _] = grad.at(t as f32).to_rgba8();
        rerun::Color::from_rgb(r, g, b)
    })
}
