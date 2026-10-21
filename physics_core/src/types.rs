use pyo3::prelude::*;
use crate::acceleration::AccelModel;

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

#[pymethods]
impl RotationalState {
    #[new]
    fn new(quat: [f64;4], omega: [f64;3]) -> Self {
        RotationalState { quat, omega }
    }
}



pub struct Object {
    pub id: u32,                          // Unique object ID
    pub name: String,                     // Object name
    pub ob_type: bool,                    // type of object, boolean, 1 for body 0 for spacecraft
    pub state: StateVector,               // Position & Velocity
    pub rot_state: RotationalState,       // Orientation & Angular velocity
    pub models: Vec<Box<dyn AccelModel>>, // Applicable acceleration models for the object
}

#[pyclass]
pub struct WorldState {
    pub objects: Vec<Object>,
    // WorldState is a vec<> of variable length that contains all objects (spacecraft and bodies) in the system
    // Each object holds its state and its relevant force models,
}
