pub fn color_by_z(points: &[nalgebra::Point3<f64>]) -> Vec<rerun::Color> {
    let z_min = 0.0;
    let z_max = 30.0;

    points
        .iter()
        .map(|p| {
            let t = ((p.z - z_min) / (z_max - z_min)) as f32;
            create_color_turbo_color_map(t)
        })
        .collect()
}

fn create_color_turbo_color_map(t: f32) -> rerun::Color {
    let r = (34.61
        + t * (1172.33 - t * (10793.56 - t * (33300.12 - t * (38394.49 - t * 14825.05)))))
        .clamp(0.0, 255.0) as u8;
    let g = (23.31 + t * (557.33 + t * (1225.33 - t * (3574.96 - t * (1073.77 + t * 707.56)))))
        .clamp(0.0, 255.0) as u8;
    let b = (27.2 + t * (3211.1 - t * (15327.97 - t * (27814.0 - t * (22569.18 - t * 6838.66)))))
        .clamp(0.0, 255.0) as u8;
    rerun::Color::from_rgb(r, g, b)
}
