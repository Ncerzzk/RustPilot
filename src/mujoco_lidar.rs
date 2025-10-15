use minifb::{Window, WindowOptions};
use std::f64::consts::PI;

pub fn init_lidar_window(width: usize, height: usize) -> Result<Window, Box<dyn std::error::Error>> {
    let window = Window::new(
        "LiDAR Scan",
        width,
        height,
        WindowOptions {
            borderless: true, // Disable window decoration
            ..WindowOptions::default()
        }
    )?;
    Ok(window)
}

pub fn update_lidar_buffer(
    lidar_width: usize,
    lidar_height: usize,
    buffer: &mut [u32],
    rf_ids: &[u16],
    angles: &[u16],
    simulation: &mujoco_rust::Simulation,
) {
    // Lidar coordinate line
    let center_x = lidar_width as f64 / 2.0;
    let center_y = lidar_height as f64 / 2.0;
    let max_radius = 10.0; // max distance:10m
    let pixels_per_meter = (lidar_width as f64 / 2.0 - 20.0) / max_radius;

    // update lidar window to clear the last recorded point
    buffer.fill(0xFFFFFFFF);

    for r in 1..=10 {
        let radius = r as f64 * pixels_per_meter;
        draw_circle_outline(buffer, lidar_width, lidar_height, center_x, center_y, radius, 0x800000);
    }

    for theta in (0..360).step_by(30) {
        let theta_rad = theta as f64 * PI / 180.0;
        let x1 = center_x;
        let y1 = center_y;
        let x2 = center_x + (max_radius * pixels_per_meter) * theta_rad.cos();
        let y2 = center_y + (max_radius * pixels_per_meter) * theta_rad.sin();
        draw_line(buffer, lidar_width, lidar_height, x1, y1, x2, y2, 0xFF000000); // black angle line
    }

    let mut points = Vec::new();
    for (i, &id) in rf_ids.iter().enumerate() {
        let distance = simulation.sensordata()[(id +1) as usize];
        let theta = angles[i] as f64 * PI / 180.0;
        if distance >= 0.0 && distance <= 10.0 {
            points.push((theta, distance));
        }
    }

    //  Draw black lidar dots
    for (theta, r) in &points {
        let x = center_x + (r * pixels_per_meter) * theta.cos();
        let y = center_y + (r * pixels_per_meter) * theta.sin();
        draw_circle(buffer, lidar_width, lidar_height, x, y, 3.0, 0xFF000000);
    }
}

pub fn draw_circle(buffer: &mut [u32], width: usize, height: usize, cx: f64, cy: f64, radius: f64, color: u32) {
    let cx = cx.round() as i32;
    let cy = cy.round() as i32;
    let radius = radius.round() as i32;
    for y in (cy - radius).max(0)..(cy + radius + 1).min(height as i32) {
        for x in (cx - radius).max(0)..(cx + radius + 1).min(width as i32) {
            let dx = x - cx;
            let dy = y - cy;
            if dx * dx + dy * dy <= radius * radius {
                buffer[(y * width as i32 + x) as usize] = color;
            }
        }
    }
}


//Draw hollow rings
pub fn draw_circle_outline(buffer: &mut [u32], width: usize, height: usize, cx: f64, cy: f64, radius: f64, color: u32) {
    let cx = cx.round() as i32;
    let cy = cy.round() as i32;
    let radius = radius.round() as i32;
    let mut x = radius;
    let mut y = 0;
    let mut err = 0;

    while x >= y {
        let points = [
            (cx + x, cy + y),
            (cx + y, cy + x),
            (cx - y, cy + x),
            (cx - x, cy + y),
            (cx - x, cy - y),
            (cx - y, cy - x),
            (cx + y, cy - x),
            (cx + x, cy - y),
        ];

        for (px, py) in points.iter() {
            if *px >= 0 && *px < width as i32 && *py >= 0 && *py < height as i32 {
                buffer[(*py * width as i32 + *px) as usize] = color;
            }
        }

        y += 1;
        err += 1 + 2 * y;
        if 2 * (err - x) + 1 > 0 {
            x -= 1;
            err += 1 - 2 * x;
        }
    }
}

//Draw lines in Bresenham algorithm
pub fn draw_line(buffer: &mut [u32], width: usize, height: usize, x1: f64, y1: f64, x2: f64, y2: f64, color: u32) {
    let mut x1 = x1.round() as i32;
    let mut y1 = y1.round() as i32;
    let x2 = x2.round() as i32;
    let y2 = y2.round() as i32;

    let dx = (x2 - x1).abs();
    let dy = (y2 - y1).abs();
    let sx = if x1 < x2 { 1 } else { -1 };
    let sy = if y1 < y2 { 1 } else { -1 };
    let mut err = dx - dy;

    loop {
        if x1 >= 0 && x1 < width as i32 && y1 >= 0 && y1 < height as i32 {
            buffer[(y1 * width as i32 + x1) as usize] = color;
        }
        if x1 == x2 && y1 == y2 {
            break;
        }
        let e2 = 2 * err;
        if e2 > -dy {
            err -= dy;
            x1 += sx;
        }
        if e2 < dx {
            err += dx;
            y1 += sy;
        }
    }
}