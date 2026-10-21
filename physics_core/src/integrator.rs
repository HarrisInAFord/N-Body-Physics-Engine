use crate::acceleration::{AccelModel, J2, PointGravity};
use crate::StateVector;
use pyo3::prelude::*;
use pyo3::exceptions::PyTypeError;
use rayon::prelude::*;


pub struct Object {
    pub id: u32,                            // Unique object ID
    pub name: String,                       // Object name
    pub ob_type: bool,                      // type of object, boolean, 1 for body 0 for spacecraft
    pub state: StateVector,                 // Position & Velocity
    pub models: Vec<Box<dyn AccelModel>>,   // Applicable acceleration models for the object
}

#[pyclass]
pub struct WorldState {
    pub objects: Vec<Object>, 
    // WorldState is a vec<> of variable length that contains all objects (spacecraft and bodies) in the system
    // Each object holds its state and its relevant force models,
}



#[pymethods]
impl WorldState {
    #[new]
    pub fn new() -> Self {
        WorldState { objects: Vec::new(), }
        }

    pub fn add_object( // used by Python to update WorldState
        &mut self,
        id: u32,
        name: String,
        ob_type: bool,
        state: StateVector,
        models: Vec<Py<PyAny>>,
        py: Python<'_>,
    ) -> PyResult<()> {
        let mut trait_models: Vec<Box<dyn AccelModel>> = Vec::with_capacity(models.len());
    
        for model in models {
            if let Ok(pg) = model.extract::<PointGravity>(py) { // adds PointGravity model to the object, if applicable
                trait_models.push(Box::new(pg));
            } else if let Ok(j2) = model.extract::<J2>(py) { // adds J2 model to object, if applicable
                trait_models.push(Box::new(j2));
            } else {
                return Err(PyTypeError::new_err("Unsupported model type passed to WorldState.add_object")); // throws an error if the model is not supported
            } // Will need to expand as more acceleration types are added
        }
        self.objects.push(Object {
            id,
            name,
            ob_type,
            state,
            models: trait_models, 
        }); // Adds the new object to the WorldState vec<>
        Ok(())
    }

    fn update_models_from_objects(&mut self) { // Dynamically updates the parameters in the acceleration models
        // Build a lookup table: id → position
        let mut positions = std::collections::HashMap::new();
        for obj in &self.objects {
            positions.insert(obj.id, obj.state.position);
        }

        // For each object, update its models based on other objects
        for obj in &mut self.objects {
            for model in &mut obj.models {
                // PointGravity
                if let Some(pg) = model.as_any_mut().downcast_mut::<PointGravity>() {
                    if let Some(new_pos) = positions.get(&pg.attractor_id) {
                        pg.body_pos = *new_pos;
                    }
                }
        // Certainly there is a cleaner way to do this but I am lazy and vibecodded this
                // J2
                if let Some(j2) = model.as_any_mut().downcast_mut::<J2>() {
                    if let Some(new_pos) = positions.get(&j2.attractor_id) {
                        j2.body_pos = *new_pos;
                    }
                }
            }
        }
    }

    pub fn step_all(&mut self, py: Python<'_>, dt: f64) { 
        // This is a simply function called in python that mutates the state vector of all the objects with only the input of the time step
        self.update_models_from_objects();
            let objects = &mut self.objects;
            py.detach(move|| {
            objects.par_iter_mut().for_each(|object| { // Par_iter_mut to parallelize the operations; much faster than single thread for multi body systems
                rk4_sv_mutate( // Applies the RK4 algorithm to mutate the state vectors of each object
                    &object.models,
                    &mut object.state.position,
                    &mut object.state.velocity,
                    dt,
                );
            });
            });
        }
    pub fn get_state(&self, id: u32) -> Option<StateVector> { // Pulls the current state of an object given it's ID
        self.objects.iter().find(|obj| obj.id == id).map(|obj| obj.state.clone())

    }
}

// RK4 state vector mutation based on all applicable acceleration models
pub fn rk4_sv_mutate(
    models: &[Box<dyn AccelModel>],
    pos: &mut [f64;3], // Mutable borrow of this value: the function simply mutates it in place without need for a return
    vel: &mut [f64;3], // """
    dt: f64,
    
) {
    
    let accel_at = |p: &[f64; 3], v: &[f64; 3]| -> [f64; 3] {
        let mut total_a = [0.0, 0.0, 0.0];
        for model in models {
            let a = model.compute_accel(p, v);
            total_a[0] += a[0];
            total_a[1] += a[1];
            total_a[2] += a[2];
            //println!("model {:?} accel = {:?}", model.name(), a) //debug print statement; currently unused
        }
        total_a // Sums all the applicable acceleration on a given body
};
    // k1
    let pos_k1 = *pos;
    let vel_k1 = *vel;
    let accel_k1 = accel_at(&pos_k1, &vel_k1);

    //k2
    let pos_k2 = [
        pos_k1[0] + 0.5 * dt * vel_k1[0],
        pos_k1[1] + 0.5 * dt * vel_k1[1],
        pos_k1[2] + 0.5 * dt * vel_k1[2]
    ];
    let vel_k2 = [
        vel_k1[0] + 0.5 * dt * accel_k1[0],
        vel_k1[1] + 0.5 * dt * accel_k1[1],
        vel_k1[2] + 0.5 * dt * accel_k1[2]
    ];
    let accel_k2 = accel_at(&pos_k2, &vel_k2);

    //k3 
    let pos_k3 = [
        pos_k1[0] + 0.5 * dt * vel_k2[0],
        pos_k1[1] + 0.5 * dt * vel_k2[1],
        pos_k1[2] + 0.5 * dt * vel_k2[2],
    ];
    let vel_k3 = [
        vel_k1[0] + 0.5 * dt * accel_k2[0],
        vel_k1[1] + 0.5 * dt * accel_k2[1],
        vel_k1[2] + 0.5 * dt * accel_k2[2],
    ];
    let accel_k3 = accel_at(&pos_k3, &vel_k3);

    // k4
    let pos_k4 = [
        pos_k1[0] + dt * vel_k3[0],
        pos_k1[1] + dt * vel_k3[1],
        pos_k1[2] + dt * vel_k3[2],
    ];
    let vel_k4 = [
        vel_k1[0] + dt * accel_k3[0],
        vel_k1[1] + dt * accel_k3[1],
        vel_k1[2] + dt * accel_k3[2],
    ];
    let accel_k4 = accel_at(&pos_k4, &vel_k4);

    // Combine k1..k4 weighted sums
    for i in 0..3 {
        pos[i] += (dt / 6.0) * (vel_k1[i] + 2.0 * vel_k2[i] + 2.0 * vel_k3[i] + vel_k4[i]);
        vel[i] += (dt / 6.0) * (accel_k1[i] + 2.0 * accel_k2[i] + 2.0 * accel_k3[i] + accel_k4[i]);
    }
}
