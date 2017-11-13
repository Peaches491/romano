extern crate nalgebra as na;


use tf::Pose;


pub trait Shape {
    fn intersects(&self, ray: &Ray) -> bool;
}


pub struct Sphere {
    pub pose: na::Matrix4<f32>,
    pub radius: f32,
}


impl Shape for Sphere {
    fn intersects(&self, ray: &Ray) -> bool {
        let l = self.pose.position() - ray.origin;
        let t_ca = l.dot(&ray.direction);
        if t_ca < 0.0 {
            return false
        }
        let d = (l.dot(&l) - t_ca.powi(2)).sqrt();
        if d > self.radius {
            return false
        } else if d < 0.0 {
            return false
        } else {
            return true
        }
    }
}


#[derive(Debug)]
pub struct Ray {
  pub origin: na::Point3<f32>,
  pub direction: na::Vector3<f32>
}

