use pyo3::exceptions::PyTypeError;
use pyo3::prelude::*;
use rayon::prelude::*;
use crate::acceleration::{AccelModel, Burn, J2, Mascons, PointGravity};
use crate::types::{WorldState, RotationalState, StateVector, Object};
use crate::math::{quat_evolve};

#[pymethods]
impl WorldState {
    #[new]
    pub fn new() -> Self {
        WorldState {
            objects: Vec::new(),
        }
    }

    pub fn add_object(
        // used by Python to update WorldState
        &mut self,
        id: u32,
        name: String,
        ob_type: bool,
        state: StateVector,
        rot_state: RotationalState,
        models: Vec<Py<PyAny>>,
        py: Python<'_>,
    ) -> PyResult<()> {
        let mut trait_models: Vec<Box<dyn AccelModel>> = Vec::with_capacity(models.len());

        for model in models {
            if let Ok(pg) = model.extract::<PointGravity>(py) {
                // adds PointGravity model to the object, if applicable
                trait_models.push(Box::new(pg));
            } else if let Ok(j2) = model.extract::<J2>(py) {
                // adds J2 model to object, if applicable
                trait_models.push(Box::new(j2));
            } else if let Ok(mascon) = model.extract::<Mascons>(py) {
                // adds Mascon model to object, if applicable
                // NOTE: Mascons and PointGravity should NEVER be simultaneously active; this must be ensured in Python (for now)
                trait_models.push(Box::new(mascon));
            } else if let Ok(burn) = model.extract::<Burn>(py) {
                trait_models.push(Box::new(burn));
            } else {
                return Err(PyTypeError::new_err(
                    "Unsupported model type passed to WorldState.add_object",
                )); // throws an error if the model is not supported
            } // Will need to expand as more acceleration types are added
        }
        self.objects.push(Object {
            id,
            name,
            ob_type,
            state,
            rot_state,
            models: trait_models,
        }); // Adds the new object to the WorldState vec<>
        Ok(())
    }

    fn update_models_from_objects(&mut self, dt: f64) {
        use std::collections::HashMap;

        // Build lookup table: id → position
        let mut positions: HashMap<u32, [f64; 3]> = HashMap::new();
        for obj in &self.objects {
            positions.insert(obj.id, obj.state.position);
        }

        // For each object, update its parameters
        for obj in &mut self.objects {
            // Evolve rotational state (quaternion + omega vector)
            obj.rot_state.quat = quat_evolve(
                &mut obj.rot_state.quat,
                obj.rot_state.omega, 
                dt,
            );

            // Update each model attached to this object ---
            for model in &mut obj.models {
                // PointGravity
                if let Some(pg) = model.as_any_mut().downcast_mut::<PointGravity>() {
                    if let Some(new_pos) = positions.get(&pg.attractor_id) {
                        pg.body_pos = *new_pos;
                    }
                }

                // J2
                if let Some(j2) = model.as_any_mut().downcast_mut::<J2>() {
                    if let Some(new_pos) = positions.get(&j2.attractor_id) {
                        j2.body_pos = *new_pos;
                    }
                }
                
                // Mascons
                if let Some(mascon) = model.as_any_mut().downcast_mut::<Mascons>(){
                    if let Some(new_pos) = positions.get(&mascon.attractor_id) {
                        mascon.body_pos = *new_pos
                }
                }

                // Burn logic is handled in Python for now
                
            }
        }
    }

    #[pyo3(name = "Graft_Model")]
    pub fn graft_model(
        &mut self,
        object_id: u32,
        model: Py<PyAny>,
        py: Python<'_>,
    ) -> PyResult<()> {
        // Used to add acceleration models to an object on demand within Python
        let obj = self
            .objects
            .iter_mut()
            .find(|o| o.id == object_id)
            .ok_or_else(|| {
                pyo3::exceptions::PyValueError::new_err(format!(
                    "Object with ID {} not found",
                    object_id
                ))
            })?;

        if let Ok(pg) = model.extract::<PointGravity>(py) {
            obj.models.push(Box::new(pg));
        } else if let Ok(j2) = model.extract::<J2>(py) {
            obj.models.push(Box::new(j2));
        } else if let Ok(mascon) = model.extract::<Mascons>(py) {
            obj.models.push(Box::new(mascon))
        } else if let Ok(burn) = model.extract::<Burn>(py) {
            obj.models.push(Box::new(burn))
        } else {
            return Err(PyTypeError::new_err(
                "Unsupported AccelModel passed to graft_model",
            ));
        }
        Ok(())
    }

    pub fn trim_model(
        &mut self,
        object_id: u32,
        target_id: u32,
        model: Py<PyAny>,
        py: Python<'_>,
    ) -> PyResult<()> {
        // Removes the specified acceleration model from the specified object
        let obj = self
            .objects
            .iter_mut()
            .find(|o| o.id == object_id)
            .ok_or_else(|| {
                pyo3::exceptions::PyValueError::new_err(format!(
                    "Object with ID {} not found",
                    object_id
                ))
            })?;

        let is_pg = model.extract::<PointGravity>(py).is_ok();
        let is_j2 = model.extract::<J2>(py).is_ok();
        let is_mascon = model.extract::<Mascons>(py).is_ok();
        let is_burn = model.extract::<Burn>(py).is_ok();

        if !is_pg && !is_j2 && !is_mascon && !is_burn {
            return Err(pyo3::exceptions::PyTypeError::new_err(
                "Unsupported AccelModel passed into trim_model",
            ));
        }
        obj.models.retain(|m: &Box<dyn AccelModel + 'static>| {
            let matches_target = m.target_id() == target_id;
            let matches_type = if is_pg {
                m.as_any().downcast_ref::<PointGravity>().is_some()
            } else if is_j2 {
                m.as_any().downcast_ref::<J2>().is_some()
            } else if is_mascon {
                m.as_any().downcast_ref::<Mascons>().is_some()
            } else {
                m.as_any().downcast_ref::<Burn>().is_some()
            }; // etc...

            !(matches_target && matches_type)
        });

        Ok(())
    }

    pub fn step_all(&mut self, py: Python<'_>, dt: f64) {
        // This is a simply function called in python that mutates the state vector of all the objects with only the input of the time step
        self.update_models_from_objects();
            let objects = &mut self.objects;
            py.detach(move|| {
            objects.par_iter_mut().for_each(|object| { // Par_iter_mut to parallelize the operations; much faster than single thread for multi body systems
                rk4_sv_mutate( // Applies the RK4 algorithm to mutate the state vectors of each object
                    &object.models,
                    &mut object.state,
                    &mut object.rot_state,
                    dt,
                );
            });
        });
    }

    pub fn get_state(&self, id: u32) -> Option<(StateVector, RotationalState)> {
        // Pulls the current state of an object given it's ID
        self.objects
            .iter()
            .find(|obj| obj.id == id)
            .map(|obj| (obj.state.clone(), obj.rot_state.clone()))
    }
}

// RK4 state vector mutation based on all applicable acceleration models
pub fn rk4_sv_mutate(
    models: &[Box<dyn AccelModel>],
    sv: &mut StateVector,
    rs: &mut RotationalState,
    dt: f64,
) {
    // evaluatew total acceleration at a given probe point and sub-step dt
    let accel_at = |pos: &[f64; 3], vel: &[f64; 3], sub_dt: f64| -> [f64; 3] {
        let temp_sv = StateVector {
            position: *pos,
            velocity: *vel,
        };
        let mut total_a = [0.0, 0.0, 0.0];
        for model in models {
            let a = model.compute_accel(&temp_sv, rs, sub_dt);
            total_a[0] += a[0];
            total_a[1] += a[1];
            total_a[2] += a[2];
        }
        total_a
    };

    let p0 = sv.position;
    let v0 = sv.velocity;

    // k1 (uses full dt)
    let accel_k1 = accel_at(&p0, &v0, dt);

    // k2 (uses half dt)
    let pos_k2 = [
        p0[0] + 0.5 * dt * v0[0],
        p0[1] + 0.5 * dt * v0[1],
        p0[2] + 0.5 * dt * v0[2],
    ];
    let vel_k2 = [
        v0[0] + 0.5 * dt * accel_k1[0],
        v0[1] + 0.5 * dt * accel_k1[1],
        v0[2] + 0.5 * dt * accel_k1[2],
    ];
    let accel_k2 = accel_at(&pos_k2, &vel_k2, 0.5 * dt);

    // k3 (uses half dt)
    let pos_k3 = [
        p0[0] + 0.5 * dt * vel_k2[0],
        p0[1] + 0.5 * dt * vel_k2[1],
        p0[2] + 0.5 * dt * vel_k2[2],
    ];
    let vel_k3 = [
        v0[0] + 0.5 * dt * accel_k2[0],
        v0[1] + 0.5 * dt * accel_k2[1],
        v0[2] + 0.5 * dt * accel_k2[2],
    ];
    let accel_k3 = accel_at(&pos_k3, &vel_k3, 0.5 * dt);

    // k4 (uses full dt)
    let pos_k4 = [
        p0[0] + dt * vel_k3[0],
        p0[1] + dt * vel_k3[1],
        p0[2] + dt * vel_k3[2],
    ];
    let vel_k4 = [
        v0[0] + dt * accel_k3[0],
        v0[1] + dt * accel_k3[1],
        v0[2] + dt * accel_k3[2],
    ];
    let accel_k4 = accel_at(&pos_k4, &vel_k4, dt);

    // Combine k1..k4 weighted sums and mutate sv in place
    for i in 0..3 {
        sv.position[i] += (dt / 6.0) * (v0[i] + 2.0 * vel_k2[i] + 2.0 * vel_k3[i] + vel_k4[i]);
        sv.velocity[i] += (dt / 6.0) * (accel_k1[i] + 2.0 * accel_k2[i] + 2.0 * accel_k3[i] + accel_k4[i]);
    }
}
