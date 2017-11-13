extern crate alga;
#[macro_use]
extern crate log;
extern crate num;
extern crate simple_logging;
extern crate image;
extern crate nalgebra as na;
extern crate num_traits;

//use alga::general::RingCommutative;
use log::LogLevelFilter;
use num_traits::Float;
//use num_traits::identities::Zero;
use std::fs::File;
use std::mem::swap;
use std::path::Path;


fn init_logging() {
    simple_logging::log_to_stderr(LogLevelFilter::Info).expect("Failed to initalize logging!");
    info!("Logging initalized!");
}

trait Shape {
    fn intersects(&self, ray: &Ray) -> bool;
}

struct Sphere {
    pose: na::Matrix4<f32>,
    radius: f32,
}

impl Shape for Sphere {
    fn intersects(&self, ray: &Ray) -> bool {
        //let L = self.pose.position() - ray.origin;
        false
    }
}


#[derive(Debug)]
struct Ray {
  origin: na::Point3<f32>,
  //direction: na::Unit<na::Vector3<f32>>
  direction: na::Vector3<f32>
}

#[derive(Debug)]
struct GraphicsContext {
    tf_root: na::Matrix4<f32>,
    projection: na::Perspective3<f32>,
    img_width: u32,
    img_height: u32,
    imgbuf: image::RgbImage,
}

impl GraphicsContext {
    fn unproject_point(&self, p: na::Point2<u32>) -> Ray {
        // Normalize pixel range from [0, width] t0 [-1, 1]
        let norm_px_x = (p.x - self.img_width/2) as f32 / self.img_width as f32;
        let norm_px_y = (p.y - self.img_height/2) as f32 / self.img_height as f32;

        // Compute two points in clip-space.
        // "ndc" = normalized device coordinates.
        let near_ndc_point = na::Point3::new(norm_px_x, norm_px_y, -1.0);
        let far_ndc_point  = na::Point3::new(norm_px_x, norm_px_y, 1.0);

        // Unproject them to view-space.
        let near_view_point = self.projection.unproject_point(&near_ndc_point);
        let far_view_point  = self.projection.unproject_point(&far_ndc_point);

        // Compute the view-space line parameters.
        let line_direction = (far_view_point - near_view_point);

        Ray {
          origin: near_view_point,
          direction: line_direction,
        }
    }

    fn project_point(&self, p: na::Vector3<f32>) -> na::Vector2<i64> {
        // Project and transform "Normalized Device Coordinates" (-1, 1) to (0, 1)
        let img_pt = (self.projection.project_vector(&p) + na::Vector3::new(1.0, 1.0, 1.0)) / 2.0;
        // Transform (0, 1) to (0, img_width) and (0, img_height)
        let px_x = (img_pt[0] * (self.img_width as f32)).round() as i64;
        let px_y = (img_pt[1] * (self.img_height as f32)).round() as i64;
        na::Vector2::new(px_x, px_y)
    }

    fn put_pixel(&mut self, x: i64, y: i64, color: image::Rgb<u8>) {
        if x >= 0 && x < self.img_width as i64 && y >= 0 && y < self.img_height as i64 {
            self.put_pixel_unchecked(x, y, color);
        }
    }

    fn put_pixel_unchecked(&mut self, x: i64, y: i64, color: image::Rgb<u8>) {
        self.imgbuf.put_pixel(x as u32, y as u32, color);
    }

    fn save(&self, image_file: &str) {
      //let ref mut fout = File::create(&Path::new(image_file)).unwrap();
      &self.imgbuf.save(image_file);
    }
}

fn project_point(
    p: na::Vector3<f32>,
    proj: &na::Perspective3<f32>,
    img_width: u32,
    img_height: u32,
) -> na::Vector2<i64> {
    // Project and transform "Normalized Device Coordinates" (-1, 1) to (0, 1)
    let img_pt = (proj.project_vector(&p) + na::Vector3::new(1.0, 1.0, 1.0)) / 2.0;
    // Transform (0, 1) to (0, img_width) and (0, img_height)
    let px_x = (img_pt[0] * (img_width as f32)).round() as i64;
    let px_y = (img_pt[1] * (img_height as f32)).round() as i64;
    na::Vector2::new(px_x, px_y)
}


fn draw_line(
    p0: &na::Matrix4<f32>,
    p1: &na::Matrix4<f32>,
    color: image::Rgb<u8>,
    mut context: &mut GraphicsContext,
) {
    draw_line_vec(&p0.position().clone_owned(), &p1.position().clone_owned(), color, context)
}

fn draw_line_vec(
    p0: &na::Vector3<f32>,
    p1: &na::Vector3<f32>,
    color: image::Rgb<u8>,
    mut context: &mut GraphicsContext,
) {
    let p0_px = context.project_point(p0.clone());
    let p1_px = context.project_point(p1.clone());
    let mut x0: i64 = p0_px[0];
    let mut y0: i64 = p0_px[1];
    let mut x1: i64 = p1_px[0];
    let mut y1: i64 = p1_px[1];
    let mut steep = false;
    // if the line is steep, we transpose the image
    if (x0 - x1).abs() < (y0 - y1).abs() {
        swap(&mut x0, &mut y0);
        swap(&mut x1, &mut y1);
        steep = true;
    }
    // make it left-to-right
    if x0 > x1 {
        swap(&mut x0, &mut x1);
        swap(&mut y0, &mut y1);
    }
    if x0 == x1 {
        x1 += 1;
    }
    for mut x in x0..x1 {
        let t = (x - x0) as f32 / ((x1 - x0) as f32);
        let mut y = ((y0 as f32) * (1.0 - t) + (y1 as f32) * t).round() as i64;
        if steep {
            // if transposed, de-transpose
            swap(&mut x, &mut y);
        }
        context.put_pixel(x, y, color);
    }
}


fn draw_axes(
    tf: &na::Matrix4<f32>,
    size: f32,
    mut context: &mut GraphicsContext,
) {
    let r = image::Rgb([255, 0, 0]);
    let g = image::Rgb([0, 255, 0]);
    let b = image::Rgb([0, 0, 255]);
    let unit_x_w = tf.append_translation(&na::Vector3::new(size, 0.0, 0.0));
    let unit_y_w = tf.append_translation(&na::Vector3::new(0.0, size, 0.0));
    let unit_z_w = tf.append_translation(&na::Vector3::new(0.0, 0.0, size));
    draw_line(&tf, &unit_x_w, r, &mut context);
    draw_line(&tf, &unit_y_w, g, &mut context);
    draw_line(&tf, &unit_z_w, b, &mut context);
}


fn draw_circle(
    tf: &na::Matrix4<f32>,
    radius: f32,
    imgbuf: &mut image::RgbImage,
    proj: &na::Perspective3<f32>,
    img_width: u32,
    img_height: u32,
) {
    let circle_pt_count = 64;
    for idx in 0..(circle_pt_count) {
        let angle = 2.0 * std::f32::consts::PI * (idx as f32 / circle_pt_count as f32);
        let world_pt_d = na::Vector3::new(radius * angle.cos(), radius * angle.sin(), 0.0);
        let world_pt = tf.clone().prepend_translation(&world_pt_d);
        let px = project_point(
            world_pt.position().clone_owned(),
            &proj,
            img_width,
            img_height,
        );
        if 0 < px[0] && px[0] < img_width as i64 && 0 < px[1] && px[1] < img_height as i64 {
            imgbuf.put_pixel(px[0] as u32, px[1] as u32, image::Rgb([255, 255, 255]));
        } else {
            error!("Pixels outside image! {}, {}", px[0], px[1]);
        }
    }
}


fn draw_cube(
    tf: &na::Matrix4<f32>,
    size: f32,
    color: image::Rgb<u8>,
    mut context: &mut GraphicsContext,
) {
    let dp = size / 2.0;
    let dn = size / -2.0;
    let corners: [na::Matrix4<f32>; 8] = [
      tf * na::Matrix4::new_translation(&na::Vector3::new(dp, dp, dp)), // 0
      tf * na::Matrix4::new_translation(&na::Vector3::new(dn, dp, dp)), // 1
      tf * na::Matrix4::new_translation(&na::Vector3::new(dn, dn, dp)), // 2
      tf * na::Matrix4::new_translation(&na::Vector3::new(dp, dn, dp)), // 3
      tf * na::Matrix4::new_translation(&na::Vector3::new(dp, dp, dn)), // 4
      tf * na::Matrix4::new_translation(&na::Vector3::new(dn, dp, dn)), // 5
      tf * na::Matrix4::new_translation(&na::Vector3::new(dn, dn, dn)), // 6
      tf * na::Matrix4::new_translation(&na::Vector3::new(dp, dn, dn)), // 7
    ];
    // Top
    draw_line(&corners[0], &corners[1], color, &mut context);
    draw_line(&corners[1], &corners[2], color, &mut context);
    draw_line(&corners[2], &corners[3], color, &mut context);
    draw_line(&corners[3], &corners[0], color, &mut context);
    // Bottom
    draw_line(&corners[4], &corners[5], color, &mut context);
    draw_line(&corners[5], &corners[6], color, &mut context);
    draw_line(&corners[6], &corners[7], color, &mut context);
    draw_line(&corners[7], &corners[4], color, &mut context);
    // Sides
    draw_line(&corners[0], &corners[4], color, &mut context);
    draw_line(&corners[1], &corners[5], color, &mut context);
    draw_line(&corners[2], &corners[6], color, &mut context);
    draw_line(&corners[3], &corners[7], color, &mut context);
}


trait Position<N: na::Scalar> {
    fn position(&self) -> na::MatrixSlice<N, na::U3, na::U1, na::U1, na::U4>;
}

impl<N: na::Scalar> Position<N> for na::Matrix4<N> {
    fn position(&self) -> na::MatrixSlice<N, na::U3, na::U1, na::U1, na::U4> {
        self.fixed_slice::<na::U3, na::U1>(0, 3)
    }
}

impl<N: na::Scalar> Position<N> for na::Vector4<N> {
    fn position(&self) -> na::MatrixSlice<N, na::U3, na::U1, na::U1, na::U4> {
        self.fixed_slice::<na::U3, na::U1>(0, 0)
    }
}


fn tf_pos_rpy<N: na::Scalar + alga::general::Real>(
    x: N,
    y: N,
    z: N,
    roll: N,
    pitch: N,
    yaw: N,
) -> na::Matrix4<N> {
    na::Matrix4::from_euler_angles(roll, pitch, yaw)
        .prepend_translation(&na::Vector3::<N>::new(x, y, z))
}


fn tf_rpy_pos<N: na::Scalar + alga::general::Real>(
    x: N,
    y: N,
    z: N,
    roll: N,
    pitch: N,
    yaw: N,
) -> na::Matrix4<N> {
    na::Matrix4::from_euler_angles(roll, pitch, yaw)
        .append_translation(&na::Vector3::<N>::new(x, y, z))
}




fn main() {
    init_logging();

    // Image
    let image_file = "/Users/daniel/GitHub/sandbox/guessing_game/test.png";
    let scale = 20.0;
    let img_width = (32.0 * scale).round() as u32;
    let img_height = (32.0 * scale).round() as u32;
    let aspect = img_width as f32 / img_height as f32;

    info!("Filling in background");
    let mut imgbuf = image::RgbImage::new(img_width, img_height);
    let mut imgbuf = image::RgbImage::from_fn(img_width, img_height, |x, y| {
        let x_pct: f32 = x as f32 / img_width as f32;
        let y_pct: f32 = y as f32 / img_height as f32;
        let brightness = 120.0;
        let mask = 0b11111000;
        let r: u8 = (brightness * (1.0 - x_pct)) as u8 & mask;
        let g: u8 = (brightness * y_pct) as u8 & mask;
        let b: u8 = (brightness * x_pct) as u8 & mask;
        image::Rgb([r, g, b])
    });

    // Projection
    let proj = na::Perspective3::new(aspect, 3.14 / 4.0, 0.1, 9.0);

    // Camera
    let origin_z = -8.0;
    let origin = tf_rpy_pos(2.8, 2.8, origin_z, 0.0, 0.0, 0.0);
    let tf_axes = tf_pos_rpy(-3.0, -3.0, origin_z, 0.0, 0.0, 0.0);

    let mut context = GraphicsContext {
        tf_root: origin.clone(),
        projection: proj,
        img_width: img_width,
        img_height: img_height,
        imgbuf: imgbuf,
    };

    info!("Drawing axes");
    draw_axes(&tf_axes, 0.325, &mut context);

    info!("Drawing cube");
    draw_cube(&origin, 0.4, image::Rgb([255, 255, 255]), &mut context);

    let sphere = Sphere {
      pose: origin.clone(),
      radius: 1.0,
    };

    let ray = context.unproject_point(na::Point2::new(img_width/2.0 as u32, img_height/2.0 as u32));
    info!("Ray: {:?}", ray);
    let ref p0 = ray.origin.coords;
    let ref p1 = ray.origin.coords + ray.direction;
    info!("P0: {}", p0);
    info!("P1: {}", p1);
    draw_line_vec(p0, p1, image::Rgb([255, 255, 255]), &mut context);

    info!("Intersects: {:?}", sphere.intersects(&ray));

    context.save(image_file);
}
