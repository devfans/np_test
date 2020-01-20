extern crate nalgebra as na;

use na::{Point3, Vector3, Isometry3};
use ncollide3d::shape::{Ball, Cuboid, ShapeHandle};
use nphysics3d::force_generator::DefaultForceGeneratorSet;
use nphysics3d::joint::DefaultJointConstraintSet;
use nphysics3d::object::{
    BodyPartHandle, ColliderDesc, DefaultBodySet, DefaultColliderSet, Ground, RigidBodyDesc,
};
use nphysics3d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};
use nphysics3d::math::Velocity;
use nphysics_testbed3d::Testbed;
use rand::{ self, prelude::* };

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mechanical_world = DefaultMechanicalWorld::new(Vector3::new(0.0, 0.0, 0.0));
    let geometrical_world = DefaultGeometricalWorld::new();
    let mut bodies = DefaultBodySet::new();
    let mut colliders = DefaultColliderSet::new();
    let joint_constraints = DefaultJointConstraintSet::new();
    let force_generators = DefaultForceGeneratorSet::new();

    /*
     * Ground.
     */
    let ground_thickness = 0.2;
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector3::new(3.0, ground_thickness, 3.0)));

    let ground_handle = bodies.insert(Ground::new());
    let co = ColliderDesc::new(ground_shape)
        .translation(Vector3::y() * -ground_thickness)
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    macro_rules! new_plane {
        ($pos: expr, $shape: expr) => {
            {
                let shape_cuboid = Cuboid::new($shape);
                let collider_desc = ColliderDesc::new(ShapeHandle::new(shape_cuboid))
                    .density(1.0)
                    .position($pos);
                /*
                let mut rigid_body = RigidBodyDesc::new()
                    .gravity_enabled(false)
                    .build();
                rigid_body.set_status(BodyStatus::Static);
                */
                let rigid_body = Ground::new();
                let parent_handle = bodies.insert(rigid_body);
                let collider = collider_desc.build(BodyPartHandle(parent_handle, 0));
                let collider_handle = colliders.insert(collider);
            }
        }
    }
    let size = 100.;
    let edge = size * 1.2;
    new_plane!(Isometry3::translation(0., size, 0.), Vector3::new(edge, 2., edge));
    new_plane!(Isometry3::translation(0., -size, 0.), Vector3::new(edge, 2., edge));
    new_plane!(Isometry3::translation(size, 0., 0.), Vector3::new(2., edge, edge));
    new_plane!(Isometry3::translation(-size, 0., 0.), Vector3::new(2., edge, edge));
    new_plane!(Isometry3::translation(0., 0., size), Vector3::new(edge, edge, 2.));
    new_plane!(Isometry3::translation(0., 0., -size), Vector3::new(edge, edge, 2.));

    let rigid_body_desc = RigidBodyDesc::new()
        .gravity_enabled(false);
    let shape_cuboid = Ball::new(1.);
    let collider_desc = ColliderDesc::new(ShapeHandle::new(shape_cuboid))
        .density(1.0);
    let mut rng = rand::thread_rng();
    for _ in 0..1000 {
        let rigid_body = rigid_body_desc.clone().position(
            Isometry3::translation(
                rng.gen_range(0, 180) as u32 as f32 - 90.,
                rng.gen_range(0, 180) as u32 as f32 - 90.,
                rng.gen_range(0, 180) as u32 as f32 - 90.
            )).velocity(Velocity::linear(
                rng.gen_range(0, 200) as u32 as f32 - 100.,
                rng.gen_range(0, 200) as u32 as f32 - 100.,
                rng.gen_range(0, 200) as u32 as f32 - 100.,
            )).build();

        let parent_handle = bodies.insert(rigid_body);
        let collider = collider_desc.build(BodyPartHandle(parent_handle, 0));
        let collider_handle = colliders.insert(collider);
    }

    /*
     * Set up the testbed.
     */
    testbed.set_ground_handle(Some(ground_handle));
    testbed.set_world(
        mechanical_world,
        geometrical_world,
        bodies,
        colliders,
        joint_constraints,
        force_generators,
    );
    testbed.look_at(Point3::new(-80., 80.0, -80.0), Point3::new(0.0, 1.0, 0.0));
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Balls", init_world)]);

    testbed.run()
}

