// physics core 
use pyo3::prelude::*;
mod integrator;   // Module used to integrate acceleration across all state vectors
mod acceleration; // Module used to calculate 3d acceleration vectors for each body
mod math;

use crate::{
    acceleration::PointGravity, 
    acceleration::J2, 
    acceleration::Mascons,
    acceleration::Burn,
    integrator::WorldState};


#[pyclass (from_py_object)]
#[derive(Clone)]
pub struct StateVector {
    #[pyo3(get,set)]
    pub position: [f64;3],
    #[pyo3(get,set)]
    pub velocity: [f64;3],
}

#[pyclass (from_py_object)]
#[derive(Clone)]
pub struct RotationalState {
    #[pyo3(get,set)]
    pub quat: [f64;4], // Quaternion of the object's orientation
    #[pyo3(get,set)]
    pub omega: [f64;3], // 3d vector of the object's angular velocity
}

#[pymethods]
impl StateVector { // Function that sets a Rust accessible StateVector object based on the Python input of position and velocity vectors
    #[new]
    pub fn new(position: Vec<f64>, velocity: Vec<f64>) -> PyResult<Self> {
        Ok(StateVector {
            position: [position[0], position[1], position[2]],
            velocity: [velocity[0], velocity[1], velocity[2]],
        })
    }
}

#[pymodule]
#[pyo3(name = "physics_core")]
fn physics_core(m: &Bound<'_, PyModule>) -> PyResult <()> {
    m.add_class::<PointGravity>()?;
    m.add_class::<J2>()?;
    m.add_class::<Mascons>()?;
    m.add_class::<Burn>()?;
    m.add_class::<StateVector>()?;
    m.add_class::<WorldState>()?;
    
    Ok(())
}
