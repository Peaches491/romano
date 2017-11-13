#[macro_use]
extern crate log;
extern crate num;
extern crate simple_logging;
extern crate image;
extern crate nalgebra as na;
extern crate num_traits;


use log::LogLevelFilter;
use num_traits::Float;
//use num_traits::identities::Zero;

mod graphics;
use graphics::GraphicsContext;
mod shape;
use shape::Shape;
mod tf;
//use tf::Pose;


fn init_logging() {
    simple_logging::log_to_stderr(LogLevelFilter::Info).expect("Failed to initalize logging!");
    info!("Logging initalized!");
}


fn main() {
    init_logging();

    // Image
    let image_file = "test.png";
    let scale = 20.0;
    let img_width = (32.0 * scale).round() as u32;
    let img_height = (32.0 * scale).round() as u32;
    let aspect = img_width as f32 / img_height as f32;

    info!("Filling in background");

    // Projection
    let proj = na::Perspective3::new(aspect, 3.14 / 4.0, 0.1, 9.0);

    // Camera
    let origin_z = -8.0;
    let origin = tf::tf_rpy_pos(2.8, 2.8, origin_z, 0.0, 0.0, 0.0);
    let tf_axes = tf::tf_pos_rpy(-3.0, -3.0, origin_z, 0.0, 0.0, 0.0);

    let sphere = shape::Sphere {
      pose: origin.clone(),
      radius: 1.0,
    };

    let imgbuf = image::RgbImage::new(img_width, img_height);
    let mut context = GraphicsContext {
        tf_root: origin.clone(),
        projection: proj,
        img_width: img_width,
        img_height: img_height,
        imgbuf: imgbuf,
    };

    let imgbuf = image::RgbImage::from_fn(img_width, img_height, |x, y| {
        let ray = context.unproject_point(na::Point2::new(x, y));
        let val = match sphere.intersects(&ray) {
            true => 255,
            false => 0,
        };
        image::Rgb([val, val, val])
    });

    context.imgbuf = imgbuf;

    info!("Drawing axes");
    graphics::draw_axes(&tf_axes, 0.325, &mut context);

    info!("Drawing cube");
    graphics::draw_cube(&origin, 0.4, image::Rgb([255, 255, 255]), &mut context);

    context.save(image_file);
}
