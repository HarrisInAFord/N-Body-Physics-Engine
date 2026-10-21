// physics core 
use pyo3::prelude::*;
mod integrator;   // Module used to integrate acceleration across all state vectors
mod acceleration; // Module used to calculate 3d acceleration vectors for each body
mod math;
mod types;

use crate::{
    acceleration::PointGravity, 
    acceleration::J2, 
    acceleration::Mascons,
    acceleration::Burn,
    types::WorldState,
    types::StateVector,
    types::RotationalState
};


#[pymodule]
#[pyo3(name = "physics_core")]
fn physics_core(m: &Bound<'_, PyModule>) -> PyResult <()> {
    m.add_class::<PointGravity>()?;
    m.add_class::<J2>()?;
    m.add_class::<Mascons>()?;
    m.add_class::<Burn>()?;
    m.add_class::<StateVector>()?;
    m.add_class::<RotationalState>()?;
    m.add_class::<WorldState>()?;
    
    Ok(())
}
