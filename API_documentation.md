# Physics Engine API Reference

This document outlines the core public functions, argument types, units, and return values exposed by the `physics_core` Rust library to Python.

### Python Exposed API: `integrator.rs` , `acceleration.rs`, and `lib.rs`

Unless otherwise noted: 

Values are in MKS units and vectors are in a global cartesian system.
Cartesian vectors are as such: [x,y,z]
Unit quaternions are [w,x,y,z]

## Object
**Object** is an struct that must be constructed in Rust by a **WorldState** method. It contains the following fields:
- id: `u32` // The unique object ID for each object, **32 bit** unsigned integer
- name: `String` // The name of the body. Does not need to be unique. **String**
- ob_type: `bool` // The type of the body. **True** for a celestial body, **False** for a spacecraft. Currently unused
- state: `StateVector` // The state vector of the object, containing 3D *position* and *velocity* vectors. **StateVector** is a custom struct detailed below 
- rot_state: `RotationalState` // The rotational state of the object, containing a *unit quaternion* and 3D *angular velocity* vector. **RotationalState** is a struct, detailed below
- models: `Vec<Box<dyn AccelModel>>` // A dynamically sized vector of all the applicable acceleration models on the object. **AccelModel** is a custom trait detailed below


Objects have no methods exposed to python themselves, but can be passed to the **WorldState** struct through one of **WorldState**'s methods.

## StateVector
**StateVector** is a struct that contains the global cartesian position and velocity of an object. It contains the following fields:
- position: `[f64;3]`
- velocity: `[f64;3]`

## RotationalState
**RotationalState** is a struct that contains the orientation and angular velocity of an object. It contains the following fields:
- quat: `[f64;4]` // Unit quaternion of the object's orientation
- omega: `[f64;3]` // 3D angular velocity vector

## WorldState
**WorldState** is a struct. While it is not exposed to Python itself, it has some methods exposed to Python. It contains the following fields:
- objects: `Vec<Object>` // A dynamically sized vector of all the objects present in the world. Objects are of the **Object** struct type


The **WorldState** type implements the following methods exposed to python:
- `add_object` // Adds a new **object** to the world state. Takes the following arguments:
    - id: `u32`
    - name: `String`
    - ob_type: `bool`
    - state: `StateVector`
    - rot_state: `RotationalState`
    - models: `list[AccelModel]`

Critically, *models* is a Python list of Rust **AccelModel** types, which are detailed below. 

- `graft_model` // Adds a new **AccelModel** to the specified **Object**
    - object_id: `u32`
    - model: `AccelModel`

- `trim_model` // Removes the specified **AccelModel** from the specified **Object**. Takes the following arguments:
    - object_id: `u32` // ID of the object to remove a model from
    - target_id: `u32` // If the model is gravitational, this is the ID of the attracting object whose model is being removed. If the model is a thruster burn, this must be identical to the `object_id`
    - model: `AccelModel` // The type of model being removed

- `step_all` // Steps the **WorldState** forwards by the specified timestep, using all applicable **AccelModel**s and the RK4 integrator. This is done in place, and does not return any value.
    - `dt`: f64 // Time step is in seconds, by convention
 
- `get_state` // Returns a tuple of the current **StateVector** and **RotationalState** of the specified object. Takes the following argument:
   - `id`: u32 // ID of the object being queried


## AccelModel
**AccelModel** is a *trait* in Rust that applies to a few different structs, where each  of these structs represents a different type of acceleration that may be applied to an **Object**. This design is intended to be modular and expandable. In the future, more types of acceleration than are accounted for now may be accounted for. Objects can also switch models on and off, for example, switching thrusters on and off. The methods for this trait are not accessible to Python, other than creating and adding new models to an object.
**AccelModel** has no *methods* exposed to Python, and all *structs* must be constructed using methods from **WorldState**. **AccelModel** implements the following methods:
- `compute_accel` // Computes the acceleration of the object given the model, position vector, and velocity vector 
    - sv: `StateVector` // State vector struct of the **Object**; detailed above
    - rs: `RotationalState` // Rotational state struct of the **Object**; detailed above
    - ws: `WorldState` // World State struct; used primarily to apply to models that require information about the rotation of other objects
    - -dt `f64` // Time step
- `as_any` // Returns the type of model as an immutable reference
- `as_any_mut` // Returns the model as a mutable reference. Used for updating attracting body positions at each timestep
- `target_id` // Returns the `u32` ID of the attracting body (or the ID of the **Object** the model applies to if the model is a thruster burn)
- `name` // Returns the name of the model as a `String`; used only for debugging

### Current Model Types:

- `PointGravity` // Point gravity is a simple gravitational model that assumes the body is a point mass centered at the core of the body. PointGravity has the following fields:
    - mu: `f64` // Gravitational parameter of the attracting body
    - body_pos: `[f64;3]` // 3D position vector of the attracting body
    - attractor_id: `u32` // ID of the attracting body
- `J2` // J2 Oblateness factor. Accounts for the equatorial buldge of celestial bodies perturbing inclined orbits. The axis of the planet must be specifed as well. J2 has the following fields:
    - mu: `f64` // Gravitational parameter of the attracting body
    - body_pos: `[f64;3]` // 3D position vector of the attracting body
    - r_eq: `f64` // Radius of the attracting body's equator
    - j2: `f64` // J2 oblateness factor of the attracting body
    - attractor_id: `u32` // ID of the attracting body
- `Mascons` // Replaces the simple `PointGravity` model with a model that can account for any number of gravitational concentrations on the body. These concentrations are expressed with a position vector in the local frame and a *fraction* of the bodies total mass in *decimal* form.  **One critical consiteration when using this model is that no attracting body should ever effect both Mascons and PointGravity onto the same object simultaneously, as Mascons accounts for the Point Gravity within itself. Applying both models will the object to effectively feel twice the acceleration it should.** The model has the following fields:
    - mu: `f64` // Gravitational parameter of the attracting body
    - body_pos: `[f64;3]` // 3D position vector of the attracting body
    - con_pos: `list[f64;3]` // A list of 3D position vectors of surface mass concentrations. It is up to the Python program / program writer to ensure that the vectors do indeed lie on or near to the surface of the body. Note also that con_pos and con_mag should be of identical lengths.
    - con_mag: `list[f64]` // A list of the magnitudes of the mass concentrations, as decimals. For example, if the mascon is 0.1% of the celestial body's mass, con_mag = [0.001]. The index of each value in con_mag should correspond to the mascon's position in con_pos
    - attractor_id: `u32` // ID of the attracting body
    *Note*: Burn is implemented as a basic acceleration model, but fuel management and spacecraft mass are not implemented within the Rust code, they must be written into the Python code accessing this physics engine.
- `Burn` // Thruster burn. Currently non-functional
    - thrust_mag: `f64` // Magnitude of the thrust. Should be in Newtons to match the mass of the craft being in kilograms
    - thrust_vec: `[f64;3]` // Unit vector of the thrust's vector. Typically will be the heading of the craft, but not necessarily
    - mass: `f64` // Mass of the space craft at the beginning of the time step being evaluated (before fuel burn is applied). Should be in kilograms to match the thrust being in Newtons
    - m_dot: `f64` // Mass flow rate of the burn, in kilograms per second
    - craft_id: `u32` // Spacecraft's ID number, used to trim burn models as needed

 ## Adding Objects to WorldState and Models to an Object:
Adding Objects to the WorldState, and AccelModels to an Object is very easy in this system:

### Initiating the list of Objects in Python
```Python
id = 1
name = "Example Body"
pos0 = [0.0,0.0,0.0] # Body located at the origin
vel0 = [0.0,0.0,0.0] # With zero velocity
ax_quat = [1.0,0.0,0.0,0.0] # Identity quaternion for an axis
w = [0.0,0.0,0.0] # Zero angular velocity
objects = {}
objects[name] = {
        "id": id,
        "name": name,
        "ob_type": True,
        "sv": physics_core.StateVector(pos0, vel0),
        "rs": physics_core.RotationalState(ax_quat, w),
        "models": [], # Fill out with applicable models
}
```

### Create a **PointGravity** Model
```Python
id = 1
name = "Sun"
pos0 = [0.0,0.0,0.0] # Body located at the origin
vel0 = [0.0,0.0,0.0] # With zero velocity

mu = 6.67e-11 * 1.988e30 # Example Mu for the Sun
sv = physics_core.StateVector(pos0, vel0)

objects["Earth"]["models"].append(physics_core.PointGravity(mu, sv.position, id)) # Adds the Sun's gravity as an acceleration applied to the Earth in a Python list of objects
```

### Create a **J2** Model
```Python
id = 2
name = "Earth"
pos0 = [1.5e11 ,0.0,0.0] # Rough position of the Earth
vel0 = [0.0,29780,0.0] # Rough velocity of the Earth
ax_quat = [1.0,0.0,0.0,0.0] # Identity quaternion for an axis
w = [0.0,0.0,7.27e-5] # Rough angular velocity of the Earth

mu = 6.67e-11 * 5.97e24 # Example Mu for the Earth
r_eq = 6378000.0 # Example equatorial radius of the Earth
j2 = 1.08e-3 # Example J2 of the Earth
sv = physics_core.StateVector(pos0, vel0)
rs = physics_core.RotationalState(ax_quat, w)

objects["Moon"]["models"].append(physics_core.J2(mu, sv.position, r_eq, j2, id)) # Adds the Earth's J2 as an acceleration applied to the Moon in a Python list of objects
```

### Create the **WorldState** with all acceleration models
```Python
ws = physics_core.WorldState()
for obj in objects.values():
    ws.add_object(obj["id"], obj["name"], obj["ob_type"], obj["sv"], obj["rs"], obj["models"])
```
### Step time forwards
```Python
dt = 10 # Ten second time step
ws.step_all(dt) # assuming ws is the **WorldState** specified above
```
### Get the state of any **Object** in the **WorldState**, and get the values of that state
```Python
state = ws.get_state(objects["Earth"]["id"])
position = state[0].position
velocity = state[0].velocity
quaternion = state[1].quat
angular_velocity = state[1].omega
```
Note that `get_state()` returns `None` when the requested ID does not exist

# Known Limitations
The following limitations are present in the current implementation of `physics_core` and may be addressed in future development.

- **N-Body RK4 Integration:** Although individual objects are integrated using fourth-order Runge-Kutta (RK4), the positions of other celestial bodies remain fixed during each object's RK4 substeps. Consequently, the integrator is not yet a fully coupled N-body RK4 implementation, potentially reducing accuracy in strongly interacting gravitational systems.

- **Attitude Integration:** Quaternion evolution currently uses first-order Euler integration with normalization rather than RK4. Angular velocity (`omega`) remains constant unless externally modified; rotational dynamics and applied torques are not yet implemented.

- **Acceleration Model Removal:** `trim_model()` removes all acceleration models matching the specified model type and `target_id`, rather than a particular instance. Multiple models of the same type associated with the same target cannot currently be removed independently.

- **Input Validation:** Some constructors lack comprehensive input validation. For example, `StateVector` expects three-element position and velocity vectors, while `Mascons` requires matching `con_pos` and `con_mag` lengths. Invalid inputs may cause Rust panics rather than Python exceptions.

- **Gravitational Model Dependencies:** `J2` and `Mascons` require their attracting bodies to exist in `WorldState` to retrieve orientation data. Missing attractors may cause a panic. Additionally, `Mascons` must not be applied alongside `PointGravity` for the same attractor, as this would double-count gravitational acceleration.

- **Thruster Burn Model:** `Burn` calculates thrust acceleration but does not automatically update spacecraft mass, propellant reserves, or thrust direction. These parameters must currently be managed externally by Python. The supplied `thrust_vec` must also be a unit vector in the global coordinate frame.

- **Python API Exposure:** `Object` and the `AccelModel` trait are internal Rust constructs, not directly exposed as Python classes. Objects must be created through `WorldState.add_object()`, and acceleration models must be instantiated using their concrete Python-exposed classes.