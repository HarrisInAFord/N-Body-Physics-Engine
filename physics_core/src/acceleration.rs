use pyo3::prelude::*;
use std::any::Any;
use dyn_clone::DynClone;
use crate::{
    StateVector,
    RotationalState,
    WorldState,
    math::{mult_3x3_3x1, transpose_3x3, quat_to_rot}
};

//================================================
// Acceleration models to be applied to bodies or spacecraft

pub trait AccelModel: Send + Sync + DynClone {
    // Computes the 3D acceleration vector 
    // given a position and velocity
    // for each model.

    // "Models" are the types of acceleration being applied
    fn compute_accel(&self, sv: &StateVector, rs: &RotationalState, ws: &WorldState, dt:f64) -> [f64;3]; // Output [f64;3] is the global 3d cartesian acceleration vector
    fn as_any(&self) -> &dyn Any; // Used to reference or inspect the model 
    fn as_any_mut(&mut self) -> &mut dyn Any; // Used to dynamically update the acceleration model's inputs
    fn target_id(&self) -> u32; // ID of the attracting body; equal to the object's own ID if the model has no attracting body (ex: thrust)
    fn name(&self) -> &'static str; // Used only for debug
}

//================================================
// Simple Point Mass Gravitation Acceleration
//================================================

#[pyclass(from_py_object)]
#[derive(Clone)]
pub struct PointGravity{
    #[pyo3(get, set)]
    pub mu: f64, // Gravitational parameter of the attracting body
    #[pyo3(get, set)]
    pub body_pos: [f64;3], // Position vector of the attracting body
    #[pyo3 (get, set)]
    pub attractor_id: u32, // ID of the attracting body 
                           // (list built in Python, must start at 1, since 0 denotes a non-gravitational acceleration)
}

#[pymethods]
impl PointGravity {
    #[new]
    pub fn new(mu: f64, body_pos: [f64;3], attractor_id: u32) -> Self {
        PointGravity { mu, body_pos, attractor_id }
    }
}

impl AccelModel for PointGravity {
    fn compute_accel(&self, sv: &StateVector, _rs: &RotationalState, _ws:&WorldState, _dt:f64) -> [f64;3] {
        let pos = sv.position;
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

    fn as_any(&self) -> &dyn Any {self}

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    } // can dynamically updates model whenever this function is called

    fn target_id(&self) -> u32 {
        // Returns the attractor ID for the given model
        self.attractor_id
    }
    }


//================================================
// J2 Oblateness Acceleration
//================================================

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
    fn compute_accel(&self, sv: &StateVector, _rs: &RotationalState, ws:&WorldState, _dt: f64) -> [f64;3] {
        let pos = sv.position;
        // Pulling the position from the state vector
        let attractor_quat = ws.get_state(self.attractor_id).unwrap().1.quat;
        let rot_mat = quat_to_rot(attractor_quat);
        // Computes the rotation matrix from the body's quaternion
        let J2 { mu, body_pos, r_eq, j2, attractor_id: _u32} = *self; 
        // Pulling useful variables
        let [rx, ry, rz] = [pos[0] - body_pos[0], pos[1] - body_pos[1], pos[2] - body_pos[2]];
        // computes the relative x y z position between the attracting body and given object
        let r_sq = rx * rx + ry * ry + rz* rz;
        // computes the square of the position vector
        let r = r_sq.sqrt();
        // compute the magnitude of the position vector
        if r == 0.0 { return [0.0,0.0,0.0];}
        // if one object is inside the other return 0 accel. This should never occur but idk this might prevent (or cause) a bug

        let trans_rot_mat = transpose_3x3(&rot_mat);

        let [rx, ry, rz] = mult_3x3_3x1(&trans_rot_mat, &[rx, ry, rz]);
        // Rotates the global coordinate system relative position into the local planet's coordinate system 
        // (Z axis defined along the axis of rotation)

        let r_5th = r_sq * r_sq * r;
        // computes position vector magnitude to the fifth
        let z_r_sq = ( rz / r ) * ( rz / r );
        // computes the Z component's magintude squared
        let factor = -1.5 * j2 * mu * (r_eq * r_eq) / r_5th;
        // computes the leading factor of the J2 equation

        let a = // Computes J2 accel in local frame
        [
            factor * rx * (1.0 - 5.0 * z_r_sq),
            factor * ry * (1.0 - 5.0 * z_r_sq),
            factor * rz * (3.0 - 5.0 * z_r_sq),
        ];

        mult_3x3_3x1(&rot_mat, &a)
        // Rotates J2 accel from local frame back into global frame

    }
    
    fn name(&self) -> &'static str {"J2"} // Only for debug; unused currently 

    fn as_any(&self) -> &dyn Any {self} // For referencing the model

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    } // Used to dynamically update J2 model parameters

    fn target_id(&self) -> u32 {
        // Returns the ID of the attracting body
        self.attractor_id
    }
}


//================================================
// Mascons / Lumpy Moons
// This feature is partially inspired by the NASA 
// legend Francis Poppy Northcut
//
// NOTE: THIS GRAVITATIONAL MODEL SHOULD *NOT* BE 
// SIMULTANEOUSLY ACTIVE ALONGSIDE A POINT GRAVITY
// MODEL; UPON CLOSE APPROACH TO A 'LUMPY MOON' THE
// POINT GRAVITY SHOULD BE TRIMMED AND THE MASCON 
// MODEL APPLIED WITHIN PYTHON
//=================================================

#[pyclass(from_py_object)]
#[derive(Clone)]
pub struct Mascons {
     #[pyo3(get, set)]
    pub mu: f64, 
    // Gravitational parameter of the body
    #[pyo3(get, set)]
    pub body_pos: [f64;3],
    // Position of the attracting body
    #[pyo3(get, set)]
    pub con_pos: Vec<[f64;3]>,
    // vec of the local frame x,y,z mascon locations
    #[pyo3(get, set)]
    pub con_mag: Vec<f64>,
    // Corresponding magnitudes of the mascons, as a percentage of the body's mass
    #[pyo3(get,set)]
    pub attractor_id: u32
    // ID of the attracting body
}

#[pymethods]
impl Mascons {
    #[new]
    pub fn new(mu:f64, body_pos: [f64;3], con_pos_list: Vec<Vec<f64>>, con_mag: Vec<f64>, attractor_id: u32) -> Self {
        let mut con_pos = Vec::with_capacity(con_pos_list.len());
        for row in con_pos_list {
            /*  Error message throws an error rn so I have to fix this
            if row.len() != 3{
                return Err("Each Element in con_pos must be a list of length 3");
            }
            */
            con_pos.push([row[0],row[1],row[2]]);
        }
        Mascons {mu, body_pos, con_pos, con_mag, attractor_id} 
    }
}

impl AccelModel for Mascons {
    fn compute_accel(&self, sv:&StateVector, _rs:&RotationalState, ws:&WorldState, _dt:f64) -> [f64;3] {
        let pos = sv.position;
        let attractor_quat = ws.get_state(self.attractor_id).unwrap().1.quat;
        let rot_mat = quat_to_rot(attractor_quat);
        let Mascons {mu, body_pos, ref con_pos, ref con_mag, attractor_id: _u32} = *self;
        let mut accel_part = [0.0,0.0,0.0];
        let mut point_mass_mu = mu;

        for (idx, con) in con_pos.iter().enumerate() {

            let local_con_pos = mult_3x3_3x1(&rot_mat,  con);

            let [rx, ry, rz] = [
                                                pos[0] - (local_con_pos[0] + body_pos[0]),
                                                pos[1] - (local_con_pos[1] + body_pos[1]),
                                                pos[2] - (local_con_pos[2] + body_pos[2])
                                              ];

            let r_sq = rx * rx + ry * ry + rz * rz;

            let r = r_sq.sqrt();

            if r == 0.0 {return [0.0,0.0,0.0]}

            let con_mu = con_mag[idx] * self.mu;

            point_mass_mu -= con_mu;
            
            let fac = - ( con_mu ) / (r_sq * r);
            
            accel_part[0] += rx * fac;
            accel_part[1] += ry * fac;
            accel_part[2] += rz * fac
        }
        
        // computes the relative x y z position vector between the attracting body and the given object
        let [rx, ry, rz] = [
                                            pos[0] - self.body_pos[0],
                                            pos[1] - self.body_pos[1],
                                            pos[2] - self.body_pos[2]
                                          ];
        // computes the square of the vector
        let r_sq= rx * rx + ry * ry + rz * rz;
        // computes the magnitude of the vector
        let r: f64 = r_sq.sqrt();
        // on the off chance that one object is within another, output zero acceleration
        if r == 0.0 {return [0.0,0.0,0.0];}
        // computes an acceleration factor, and applies it to the vector
        let fac = - point_mass_mu / (r_sq * r);
        //println!("DEBUG mu = {}", self.mu); // Debug statement; unused
        let point_mass_accel = [rx * fac, ry * fac, rz * fac,];

        [
            point_mass_accel[0] + accel_part[0],
            point_mass_accel[1] + accel_part[1],
            point_mass_accel[2] + accel_part[2]
        ]
        
    }

    fn name(&self) -> &'static str{"Mascons"} // Outputs the model type; used only for debug

    fn as_any(&self) -> &dyn Any {self} // For referencing

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    } // dynamically updates model whenever this function is called

    fn target_id(&self) -> u32 {
        // Returns the ID of the attracting body
        self.attractor_id
    }

}

//================================================
// Thruster Burn Acceleration 
//================================================

#[pyclass(from_py_object)]
#[derive(Clone)]
pub struct Burn {
    #[pyo3(get,set)]
    pub thrust_mag: f64,
    // Thrust magnitude in Newtons
    #[pyo3(get,set)]
    pub thrust_vec: [f64;3],
    // 3d thrust unit vector 
    #[pyo3(get, set)]
    pub mass: f64,
    // Mass of the spacecraft at the given timestep
    #[pyo3(get,set)]
    pub m_dot: f64,
    // Mass flowrate of the propellant during the burn
    #[pyo3(get, set)]
    pub craft_id: u32
    // Craft's own ID (used to trim accel model as needed)
}

#[pymethods]
impl Burn {
    #[new]
    pub fn new(thrust_mag:f64, thrust_vec:[f64;3], mass: f64, m_dot:f64, craft_id: u32) -> Self {
        Burn {thrust_mag, thrust_vec, mass, m_dot, craft_id}
    }
}

impl AccelModel for Burn {
    fn compute_accel(&self, _sv:&StateVector, _rs:&RotationalState, _ws:&WorldState, dt:f64) -> [f64;3] {
        let Burn {thrust_mag, thrust_vec, mass, m_dot, craft_id: _u32 } = *self;
        // It's all F = ma ? 
        // Always has been

        let current_mass = mass - (m_dot * dt);
        let eff_mass = if current_mass > 0.0 { current_mass } else { f64::MIN_POSITIVE };

        [
            (thrust_mag * thrust_vec[0]) / eff_mass,
            (thrust_mag * thrust_vec[1]) / eff_mass,
            (thrust_mag * thrust_vec[2]) / eff_mass
        ]
    }

    
    fn name(&self) -> &'static str{"Burn"} // Outputs the model type; used only for debug

    fn as_any(&self) -> &dyn Any {self} // For referencing 

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    } // dynamically updates model whenever this function is called

    fn target_id(&self) -> u32 {
        self.craft_id // Return's the object's own ID as it's "target"
    }
}
