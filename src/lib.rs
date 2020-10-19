use cglinalg::{
    Vector3,
    Vector4,
    Matrix4x4,
    Quaternion,
    Radians,
    ScalarFloat,
    Unit,
};

use core::fmt;


pub type PointLight<S> = Light<S, PointLightModel<S>>;
pub type SpotLight<S> = Light<S, SpotLightModel<S>>;

/// A type with this trait can be used as a lighting model. 
///
/// A lighting model is the model that a light uses to illuminate objects
/// in a scene. 
pub trait IlluminationModel {
    /// The type containing the parameters for constructing the lighting model.
    type Spec;

    /// Construct a camera model from a description of the 
    /// camera model's parameters.
    fn from_spec(spec: &Self::Spec) -> Self;
}

/// This type carries all the information describing the change in attitude of
/// a light in a scene in Euclidean space.
#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct DeltaAttitude<S> {
    /// The change in the position of the light.
    pub delta_position: Vector3<S>,
    /// The change in the orientation of the light about the **negative z-axis**.
    pub roll: Radians<S>,
    /// The change in the orientation of the light about the **positive y-axis**.
    pub yaw: Radians<S>,
    /// The change in the orientation of the light about the **positive x-axis**.
    pub pitch: Radians<S>,
}

impl<S> DeltaAttitude<S> where S: ScalarFloat {
    /// Construct a new change in attitude.
    #[inline]
    pub fn new<A: Into<Radians<S>>>(delta_position: Vector3<S>, roll: A, yaw: A, pitch: A) -> Self {
        Self {
            delta_position: delta_position,
            roll: roll.into(),
            yaw: yaw.into(),
            pitch: pitch.into(),
        }
    }

    /// Construct zero change in attitude.
    #[inline]
    pub fn zero() -> Self {
        Self {
            delta_position: Vector3::zero(),
            roll: Radians::zero(),
            yaw: Radians::zero(),
            pitch: Radians::zero(),
        }
    }
}

impl<S> fmt::Display for DeltaAttitude<S> where S: fmt::Display {
    fn fmt(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        write!(
            formatter,
            "DeltaAttitude [x={}, y={}, z={}, roll={}, yaw={}, pitch={}]",
            self.delta_position.x, 
            self.delta_position.y, 
            self.delta_position.z, 
            self.roll, 
            self.yaw, 
            self.pitch
        )
    }
}

#[derive(Clone,)]
pub struct PointLightModelSpec<S> {
    pub ambient: Vector3<S>,
    pub diffuse: Vector3<S>,
    pub specular: Vector3<S>,
}

impl<S> PointLightModelSpec<S> where S: ScalarFloat {
    /// Construct a new point light specification.
    #[inline]
    pub fn new(
        ambient: Vector3<S>, 
        diffuse: Vector3<S>, 
        specular: Vector3<S>) -> PointLightModelSpec<S> 
    {
        PointLightModelSpec {
            ambient: ambient,
            diffuse: diffuse,
            specular: specular,
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct PointLightModel<S> {
    pub ambient: Vector3<S>,
    pub diffuse: Vector3<S>,
    pub specular: Vector3<S>,
}

impl<S> IlluminationModel for PointLightModel<S> 
    where S: ScalarFloat
{
    type Spec = PointLightModelSpec<S>;

    #[inline]
    fn from_spec(spec: &Self::Spec) -> Self {
        PointLightModel {
            ambient: spec.ambient,
            diffuse: spec.diffuse,
            specular: spec.specular,
        }
    }
}


#[derive(Copy, Clone, Debug, PartialEq)]
pub struct SpotLightModelSpec<S> {
    cutoff: S,
    outer_cutoff: S,
    /// The spotlight illumination parameters.
    ambient: Vector3<S>,
    diffuse: Vector3<S>,
    specular: Vector3<S>,
    /// The spotlight attenuation parameters.
    constant: S,
    linear: S,
    quadratic: S,
}

impl<S> SpotLightModelSpec<S> where S: ScalarFloat {
    #[inline]
    pub fn new(
        cutoff: S,
        outer_cutoff: S,
        ambient: Vector3<S>,
        diffuse: Vector3<S>,
        specular: Vector3<S>,
        constant: S,
        linear: S,
        quadratic: S) -> SpotLightModelSpec<S> 
    {
        SpotLightModelSpec { 
            cutoff: cutoff,
            outer_cutoff: outer_cutoff,
            ambient: ambient,
            diffuse: diffuse,
            specular: specular,
            constant: constant,
            linear: linear,
            quadratic: quadratic,
        }
    }   
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct SpotLightModel<S> {
    pub cutoff: S,
    pub outer_cutoff: S,
    /// The spotlight illumination parameters.
    pub ambient: Vector3<S>,
    pub diffuse: Vector3<S>,
    pub specular: Vector3<S>,
    /// The spotlight attenuation parameters.
    pub constant: S,
    pub linear: S,
    pub quadratic: S,
}

impl<S> IlluminationModel for SpotLightModel<S> where S: ScalarFloat {
    type Spec = SpotLightModelSpec<S>;

    #[inline]
    fn from_spec(spec: &Self::Spec) -> Self {
        SpotLightModel {
            cutoff: spec.cutoff,
            outer_cutoff: spec.outer_cutoff,
            ambient: spec.ambient,
            diffuse: spec.diffuse,
            specular: spec.specular,
            constant: spec.constant,
            linear: spec.linear,
            quadratic: spec.quadratic,
        }
    }
}

/// A specification describing a rigid body transformation for the attitude 
/// (position and orientation) of a spotlight. The spec describes the location, 
/// local coordinate system, and rotation axis for the light in world space.
/// The coordinate transformation is right-handed orthonormal transformation.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct LightAttitudeSpec<S> {
    /// The location of the light in world space.
    position: Vector3<S>,
    /// The direction of the **negative z-axis** (forward axis) of the light.
    forward: Vector3<S>,
    /// The direction of the **positive x-axis** (right axis) of the light.
    right: Vector3<S>,
    /// The direction of the **positive y-axis** (up axis) of the light.
    up: Vector3<S>,
    /// The **axis of rotation** of the light. It is not necessary that 
    /// the axis of rotation of the light be the same as one of the coordinate
    /// axes.
    axis: Vector3<S>,
}

impl<S> LightAttitudeSpec<S> where S: ScalarFloat {
    /// Construct a new camera attitude specification.
    #[inline]
    pub fn new(
        position: Vector3<S>,
        forward: Vector3<S>,
        right: Vector3<S>,
        up: Vector3<S>,
        axis: Vector3<S>) -> Self {

        LightAttitudeSpec {
            position: position,
            forward: forward,
            right: right,
            up: up,
            axis: axis,
        }
    }
}

impl<S> fmt::Display for LightAttitudeSpec<S> where S: fmt::Display {
    fn fmt(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        write!(
            formatter,
            "LightAttitudeSpec [position={}, forward={}, right={} up={}, axis={}]",
            self.position, self.forward, self.right, self.up, self.axis
        )
    }
}

/// This type contains all the data for tracking the position and orientation
/// of a light in world space. The light attitude here uses a right-handed 
/// coordinate system facing along the light's **negative z-axis**.
/// The coordinate system is a right-handed coordinate system with orthonormal
/// basis vectors.
#[repr(C)]
#[derive(Clone, Debug)]
struct LightAttitude<S> {
    /// The world space position of the light.
    position: Vector3<S>,
    /// The distance from the light perpendicular to the light's **xy-plane**.
    forward: Vector4<S>,
    /// The horizontal axis of the light's coordinate system.
    right: Vector4<S>,
    /// The vertical axis of the light's coordinate system.
    up: Vector4<S>,
    /// The **axis of rotation** of the light. It is not necessary that 
    /// the axis of rotation of the light be the same as one of the coordinate
    /// axes.
    axis: Quaternion<S>,
    /// The translation matrix mapping objects from the world space coordinate
    /// frame to the coordinate frame centered at the eye position of the camera.
    translation_matrix: Matrix4x4<S>,
    /// The rotation matrix rotating the a vector in world space to the coordinate
    /// system of the camera's view space.
    rotation_matrix: Matrix4x4<S>,
    /// The viewing matrix of the camera mapping the complete translation + rotation
    /// of the camera.
    view_matrix: Matrix4x4<S>,
}

impl<S> LightAttitude<S> where S: ScalarFloat {
    /// Construct the camera's viewing transformation from its specification. 
    #[inline]
    fn from_spec(spec: &LightAttitudeSpec<S>) -> Self {
        let axis = Quaternion::from_parts(S::zero(), spec.axis);
        let translation_matrix = Matrix4x4::from_affine_translation(
            &(-spec.position)
        );
        let rotation_matrix = Matrix4x4::from(&axis);
        let view_matrix = rotation_matrix * translation_matrix;

        Self {
            position: spec.position,
            forward: spec.forward.to_homogeneous(),
            right: spec.right.to_homogeneous(),
            up: spec.up.to_homogeneous(),
            axis: axis,
            translation_matrix: translation_matrix,
            rotation_matrix: rotation_matrix,
            view_matrix: view_matrix,
        }
    }

    /// Get the camera's up direction in camera space.
    #[inline]
    fn up_axis_eye(&self) -> Vector3<S> {
        let zero = S::zero();
        let one = S::one();

        Vector3::new(zero, one, zero)
    }
        
    /// Get the camera's right axis in camera space.
    #[inline]
    fn right_axis_eye(&self) -> Vector3<S> {
        let zero = S::zero();
        let one = S::one();
        
        Vector3::new(one, zero ,zero)
    }
        
    /// Get the camera's forward axis in camera space.
    #[inline]
    fn forward_axis_eye(&self) -> Vector3<S> {
        let zero = S::zero();
        let one = S::one();
        
        Vector3::new(zero, zero, -one)
    }

    /// Update the light position based on the change in the camera's 
    /// attitude.
    #[inline]
    fn update_position_eye(&mut self, delta_attitude: &DeltaAttitude<S>) {
        self.position += self.forward.contract() * -delta_attitude.delta_position.z;
        self.position += self.up.contract()      *  delta_attitude.delta_position.y;
        self.position += self.right.contract()   *  delta_attitude.delta_position.x;

        let translation_inv = Matrix4x4::from_affine_translation(
            &self.position
        );
        self.translation_matrix = translation_inv.inverse().unwrap();
    }

    /// Update the light axes so we can rotate the camera about the new rotation axes.
    #[inline]
    fn update_orientation_eye(&mut self, delta_attitude: &DeltaAttitude<S>) {
        let axis_yaw = Unit::from_value(self.up.contract());
        let q_yaw = Quaternion::from_axis_angle(
            &axis_yaw, delta_attitude.yaw
        );
        self.axis = q_yaw * self.axis;

        let axis_pitch = Unit::from_value(self.right.contract());
        let q_pitch = Quaternion::from_axis_angle(
            &axis_pitch, delta_attitude.pitch
        );
        self.axis = q_pitch * self.axis;

        let axis_roll = Unit::from_value(self.forward.contract());
        let q_roll = Quaternion::from_axis_angle(
            &axis_roll, delta_attitude.roll, 
        );
        self.axis = q_roll * self.axis;

        let rotation_matrix_inv = Matrix4x4::from(&self.axis);
        self.forward = rotation_matrix_inv * Vector4::new(S::zero(), S::zero(), -S::one(), S::zero());
        self.right   = rotation_matrix_inv * Vector4::new(S::one(), S::zero(), S::zero(), S::zero());
        self.up      = rotation_matrix_inv * Vector4::new(S::zero(), S::one(), S::zero(), S::zero());
        self.rotation_matrix = rotation_matrix_inv.inverse().unwrap();
    }

    #[inline]
    fn update_position_world(&mut self, new_position: &Vector3<S>) {
        self.position = new_position.clone();
        let translation_inv = Matrix4x4::from_affine_translation(
            &self.position
        );
        self.translation_matrix = translation_inv.inverse().unwrap();
        self.view_matrix = self.rotation_matrix * self.translation_matrix;
    }

    /// Update the light's attitude based on the input change in light 
    /// attitude.
    #[inline]
    fn update(&mut self, delta_attitude: &DeltaAttitude<S>) {
        self.update_orientation_eye(delta_attitude);
        self.update_position_eye(delta_attitude);
        self.view_matrix = self.rotation_matrix * self.translation_matrix;
    }
}



pub struct Light<S, M> {
    model: M,
    attitude: LightAttitude<S>,
}

impl<S, M> Light<S, M>
    where S: ScalarFloat,
          M: IlluminationModel,
{
    pub fn new(
        model_spec: &M::Spec, 
        attitude_spec: &LightAttitudeSpec<S>) -> Self {

        Light {
            model: M::from_spec(model_spec),
            attitude: LightAttitude::from_spec(attitude_spec),
        }
    }

    /// Update the light's attitude (i.e. position and orientation) in
    /// the light's local coordinate frame.
    #[inline]
    pub fn update_attitude_eye(&mut self, delta_attitude: &DeltaAttitude<S>) {
        self.attitude.update(delta_attitude);
    }

    #[inline]
    pub fn update_position_world(&mut self, new_position: &Vector3<S>) {
        self.attitude.update_position_world(new_position);
    }

    #[inline]
    pub fn model(&self) -> &M {
        &self.model
    }

    /// Get the camera's position in world space.
    #[inline]
    pub fn position(&self) -> Vector3<S> { 
        self.attitude.position
    }

    /// Get the camera's up direction in world space.
    #[inline]
    pub fn up_axis(&self) -> Vector3<S> {
        self.attitude.up.contract()
    }
        
    /// Get the camera's right axis in world space.
    #[inline]
    pub fn right_axis(&self) -> Vector3<S> {
        self.attitude.right.contract()
    }
        
    /// Get the camera's forward axis in world space.
    #[inline]
    pub fn forward_axis(&self) -> Vector3<S> {
        self.attitude.forward.contract()
    }

    /// Get the camera's **vertical y-axis** in camera view space.
    #[inline]
    pub fn up_axis_eye(&self) -> Vector3<S> {
        self.attitude.up_axis_eye()
    }
        
    /// Get the camera's **horizontal x-axis** in camera view space.
     #[inline]
    pub fn right_axis_eye(&self) -> Vector3<S> {
        self.attitude.right_axis_eye()
    }
        
    /// Get the camera's **forward z-axis** in camera view space.
    #[inline]
    pub fn forward_axis_eye(&self) -> Vector3<S> {
        self.attitude.forward_axis_eye()
    }
        
    /// Get the camera's axis of rotation.
    #[inline]
    pub fn rotation_axis(&self) -> Vector3<S> {
        self.attitude.axis.v
    }

    #[inline]
    pub fn view_matrix(&self) -> &Matrix4x4<S> {
        &self.attitude.view_matrix
    }
 
    #[inline]
    pub fn model_matrix(&self) -> Matrix4x4<S> {
        Matrix4x4::from_affine_translation(&self.position())
    }
}

