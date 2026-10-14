use pyo3::prelude::*;
use std::any::Any;

// Acceleration models to be applied to bodies or spacecraft

pub trait AccelModel: Send + Sync {
    // Computes the 3D acceleration vector 
    // given a position and velocity
    // for each model.

    // "Models" are the types of acceleration being applied
    fn compute_accel(&self, pos: &[f64;3], vel: &[f64;3]) -> [f64;3]; // Output [f64;3] is the global 3d cartesian acceleration vector
    fn as_any_mut(&mut self) -> &mut dyn Any; // Used to dynamically update the acceleration model's inputs
    fn name(&self) -> &'static str; // Used only for debug

}

//================================================
// Simple Point Mass Gravitation Acceleration
//================================================

// Mu should be either the parameter for a single body to a space craft, or 
// the parameter between two different bodies for N-body attractions
#[pyclass(from_py_object)]
#[derive(Clone)]
pub struct PointGravity{
    #[pyo3(get, set)]
    pub mu: f64, // Gravitational parameter of the attracting body
    #[pyo3(get, set)]
    pub body_pos: [f64;3], // Position vector of the attracting body
    #[pyo3 (get, set)]
    pub attractor_id: u32, // ID of the attracting body
}

#[pymethods]
impl PointGravity {
    #[new]
    pub fn new(mu: f64, body_pos: [f64;3], attractor_id: u32) -> Self {
        PointGravity { mu, body_pos, attractor_id }
    }
}

impl AccelModel for PointGravity {
    fn compute_accel(&self, pos: &[f64;3], _vel: &[f64;3]) -> [f64;3] {
        // computes the relative x y z position vector between the attracting body and the given object
        let [rx, ry, rz] = [pos[0] - self.body_pos[0], pos[1] - self.body_pos[1], pos[2] - self.body_pos[2]];
        // computes the square of the vector
        let r_sq= rx * rx + ry * ry + rz * rz;
        // computes the magnitude of the vector
        let r: f64 = r_sq.sqrt();
        // on the off chance that one object is within another, output zero acceleration
        if r == 0.0 {return [0.0,0.0,0.0];}
        // computes an acceleration factor, and applies it to the vector
        let fac = - self.mu / (r_sq * r);
        //println!("DEBUG mu = {}", self.mu); // Debug statement; unused
        [rx * fac, ry * fac, rz * fac,]
        
    }

    fn name(&self) -> &'static str{"PointGravity"} // Outputs the model type; used only for debug

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    } // dynamically updates model whenever this function is called
    }




//================================================
// J2 Oblateness Acceleration
//================================================


// Mu is, as before, between either the attracting body and spacecraft, 
// or between the two bodies.
// J2 is the J2 oblateness parameter
// r_eq is the equatorial radius of the attracting body

// Currently this model is only applicable for bodies with their axis of rotation aligned with the global Z direction
        //Will need to add a rotation matrix to change this whenever I implement axial tilt for bodies

#[pyclass(from_py_object)]
#[derive(Clone)]
pub struct J2 {
    #[pyo3(get, set)]
    pub mu: f64,
    // Attracting body's gravitational parameter
    #[pyo3(get, set)]
    pub body_pos: [f64;3],
    // Position vector to attracting body
    #[pyo3(get, set)]
    pub r_eq: f64,
    // Equatorial radius of the attracting body
    #[pyo3(get, set)]
    pub j2: f64,
    // J2 factor for the attracting body
    #[pyo3(get, set)]
    pub attractor_id: u32, 
    // ID of the attracting body
}

#[pymethods]
impl J2 {
    #[new]
    pub fn new(mu: f64, body_pos: [f64;3], r_eq: f64, j2: f64, attractor_id: u32) -> Self {
        J2 { mu, body_pos, r_eq, j2, attractor_id}
    }
}

impl AccelModel for J2 {
    fn compute_accel(&self, pos: &[f64;3], _vel: &[f64;3]) -> [f64;3] {
        let J2 { mu, body_pos, r_eq, j2, attractor_id: _u32} = *self; // Pulling useful variables
        let [rx, ry, rz] = [pos[0] - body_pos[0], pos[1] - body_pos[1], pos[2] - body_pos[2]];
        // computes the relative x y z position between the attracting body and given object
        let r_sq = rx * rx + ry * ry + rz* rz;
        // computes the square of the position vector
        let r = r_sq.sqrt();
        // compute the magnitude of the position vector
        if r == 0.0 { return [0.0,0.0,0.0];}
        // if one object is inside the other return 0 accel. This should never occur but idk this might prevent (or cause) a bug

        let r_5th = r_sq * r_sq * r;
        // computes position vector magnitude to the fifth
        let z_r_sq = ( rz / r ) * ( rz / r );
        // computes the Z component's magintude squared
        let factor = -1.5 * j2 * mu * (r_eq * r_eq) / r_5th;
        // computes the leading factor of the J2 equation

        [
            factor * rx * (1.0 - 5.0 * z_r_sq),
            factor * ry * (1.0 - 5.0 * z_r_sq),
            factor * rz * (3.0 - 5.0 * z_r_sq),
        ]
        // computes J2 accel and outputs it

    }
    fn name(&self) -> &'static str {"J2"} // Only for debug; unused currently 

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    } // Used to dynamically update J2 model parameters
}
