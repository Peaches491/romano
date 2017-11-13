extern crate alga;
extern crate nalgebra as na;


pub trait Pose<N: na::Scalar> {
    fn position(&self) -> na::Point<N, na::U3> {
        let v = self.translation();
        na::Point::<N, na::U3>::new(v[0], v[1], v[2])
    }
    fn get_translation(&self) -> na::Vector3<N> {
        self.translation().clone_owned()
    }
    fn translation(&self) -> na::MatrixSlice<N, na::U3, na::U1, na::U1, na::U4>;
}


impl<N: na::Scalar> Pose<N> for na::Matrix4<N> {
    fn translation(&self) -> na::MatrixSlice<N, na::U3, na::U1, na::U1, na::U4> {
        self.fixed_slice::<na::U3, na::U1>(0, 3)
    }
}


impl<N: na::Scalar> Pose<N> for na::Vector4<N> {
    fn translation(&self) -> na::MatrixSlice<N, na::U3, na::U1, na::U1, na::U4> {
        self.fixed_slice::<na::U3, na::U1>(0, 0)
    }
}


pub fn tf_pos_rpy<N: na::Scalar + alga::general::Real> (
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


pub fn tf_rpy_pos<N: na::Scalar + alga::general::Real> (
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
