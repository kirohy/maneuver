use kiss3d::scene::SceneNode;

use na::{Translation3, UnitQuaternion, Vector3};
use nalgebra as na;

use std::f32::consts::FRAC_PI_2;

pub fn put_xyz_axis(node: &mut SceneNode, scale: f32) {
    let axis_radius: f32 = scale * 0.1;
    let axis_length: f32 = scale;
    let arrow_radius: f32 = scale * 0.2;
    let arrow_length: f32 = scale * 0.3;

    let mut x_axis = node.add_cylinder(axis_radius, axis_length);
    let mut y_axis = node.add_cylinder(axis_radius, axis_length);
    let mut z_axis = node.add_cylinder(axis_radius, axis_length);

    let mut x_arrow = node.add_cone(arrow_radius, arrow_length);
    let mut y_arrow = node.add_cone(arrow_radius, arrow_length);
    let mut z_arrow = node.add_cone(arrow_radius, arrow_length);

    x_axis.append_rotation(&UnitQuaternion::from_axis_angle(
        &Vector3::z_axis(),
        -FRAC_PI_2,
    ));
    x_axis.append_translation(&Translation3::new(axis_length / 2.0, 0.0, 0.0));

    y_axis.append_translation(&Translation3::new(0.0, axis_length / 2.0, 0.0));

    z_axis.append_rotation(&UnitQuaternion::from_axis_angle(
        &Vector3::x_axis(),
        FRAC_PI_2,
    ));
    z_axis.append_translation(&Translation3::new(0.0, 0.0, axis_length / 2.0));

    x_arrow.append_rotation(&UnitQuaternion::from_axis_angle(
        &Vector3::z_axis(),
        -FRAC_PI_2,
    ));
    x_arrow.append_translation(&Translation3::new(
        axis_length + arrow_length / 2.0,
        0.0,
        0.0,
    ));

    y_arrow.append_translation(&Translation3::new(
        0.0,
        axis_length + arrow_length / 2.0,
        0.0,
    ));

    z_arrow.append_rotation(&UnitQuaternion::from_axis_angle(
        &Vector3::x_axis(),
        FRAC_PI_2,
    ));
    z_arrow.append_translation(&Translation3::new(0.0, 0.0, axis_length));

    x_axis.set_color(1.0, 0.0, 0.0);
    y_axis.set_color(0.0, 1.0, 0.0);
    z_axis.set_color(0.0, 0.0, 1.0);
    x_arrow.set_color(1.0, 0.0, 0.0);
    y_arrow.set_color(0.0, 1.0, 0.0);
    z_arrow.set_color(0.0, 0.0, 1.0);
}

pub struct Arm {
    upper: SceneNode,
    lower: SceneNode,
}

impl Arm {
    pub fn new(mut upper: SceneNode, radius: f32, upper_length: f32, lower_length: f32) -> Arm {
        let mut lower = upper.add_group();
        let mut upper_arm = upper.add_capsule(radius, upper_length);
        let mut elbow = upper.add_cylinder(radius, radius);
        let mut lower_arm = lower.add_capsule(radius, lower_length);

        upper_arm.append_rotation(&UnitQuaternion::from_axis_angle(
            &Vector3::z_axis(),
            FRAC_PI_2,
        ));
        upper_arm.append_translation(&Translation3::new(upper_length / 2.0 + radius, 0.0, 0.0));

        elbow.append_rotation(&UnitQuaternion::from_axis_angle(
            &Vector3::x_axis(),
            FRAC_PI_2,
        ));
        elbow.append_translation(&Translation3::new(upper_length + radius * 3.0, 0.0, 0.0));

        lower_arm.append_rotation(&UnitQuaternion::from_axis_angle(
            &Vector3::z_axis(),
            FRAC_PI_2,
        ));
        lower_arm.append_translation(&Translation3::new(
            lower_length / 2.0 + radius * 2.0,
            0.0,
            0.0,
        ));

        lower.append_translation(&Translation3::new(upper_length + radius * 3.0, 0.0, 0.0));

        upper_arm.set_color(1.0, 1.0, 0.0);
        elbow.set_color(1.0, 0.0, 1.0);
        lower_arm.set_color(0.0, 1.0, 1.0);

        Arm { upper, lower }
    }

    pub fn set_upper_posture(&mut self, q: UnitQuaternion<f32>) -> &Self {
        self.upper.set_local_rotation(q);
        self
    }

    pub fn set_theta_lower(&mut self, theta: f32) -> &Self {
        self.lower
            .set_local_rotation(UnitQuaternion::from_axis_angle(&Vector3::z_axis(), theta));
        self
    }
}
