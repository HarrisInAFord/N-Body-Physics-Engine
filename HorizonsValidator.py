import json
import csv
import time
import re
import requests
import numpy as np
from tqdm import tqdm
import physics_core
from astroquery.jplhorizons import Horizons
from pathlib import Path

#==================================================
# Helper Functions
#==================================================

def to_seconds(time_str: str) -> float:
  time_str = str(time_str).strip().lower()

  # Match a number (int or float) followed by an optional unit (s, m, h, d)
  match = re.search(r"([\d.]+)\s*([a-z]+)", time_str)
  if not match:
    raise ValueError(f"Invalid time format: '{time_str}'")

  val, unit = match.groups()
  val = float(val)

  multipliers = {"s": 1, "m": 60, "h": 3600, "d": 86400}

  if unit:
    return val * multipliers[unit]
  return val  # Defaults to seconds if no unit is provided

def quat_from_2_vec(v1, v2):
   v1 = v1 / np.linalg.norm(v1)
   v2 = v2 / np.linalg.norm(v2)
   axis = np.cross(v1,v2)
   c = np.dot(v1, v2)
   s = np.sqrt((1+c)*2)
   return np.array([s/2, axis[0]/s, axis[1]/s, axis[2]/2])

def quat_from_two_vectors(v1, v2):
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)
    c = np.dot(v1, v2)
    axis = np.cross(v1, v2)
    s = np.sqrt((1 + c) * 2)
    return np.array([s/2, axis[0]/s, axis[1]/s, axis[2]/s])

def rotmat_from_axis(axis):
    axis = axis / np.linalg.norm(axis)  # ẑ

    # Choose a vector not parallel to axis
    if abs(axis[0]) < 0.9:
        temp = np.array([1.0, 0.0, 0.0])
    else:
        temp = np.array([0.0, 1.0, 0.0])

    x = np.cross(temp, axis)
    x /= np.linalg.norm(x)

    y = np.cross(axis, x)

    # Rotation matrix columns = body axes in inertial frame
    return np.column_stack((x, y, axis))

def sidereal_to_rad_s(period):
   return 2 * np.pi / (period * 3600) # Assumes period is given in hours

def axis_from_ra_dec(ra_deg, dec_deg):
    ra  = np.radians(ra_deg)
    dec = np.radians(dec_deg)
    return np.array([
        np.cos(dec) * np.cos(ra),
        np.cos(dec) * np.sin(ra),
        np.sin(dec)
    ])

def rotation_matrix_from_ra_dec(ra_deg, dec_deg):
    axis = axis_from_ra_dec(ra_deg, dec_deg)
    return rotmat_from_axis(axis)

def horizons_positions(name, epochs):
    # Use 'id' for major bodies; 'id_type' can be 'majorbody'
    obj = Horizons(id=name, location='@0', epochs=epochs)

    eph = obj.vectors()
    # Horizons returns x, y, z in AU; convert to meters
    AU = 1.495978707e11
    x = np.array(eph['x']) * AU
    y = np.array(eph['y']) * AU
    z = np.array(eph['z']) * AU
    vx = np.array(eph['vx']) * AU / 86400.0  # AU/day -> m/s
    vy = np.array(eph['vy']) * AU / 86400.0
    vz = np.array(eph['vz']) * AU / 86400.0
    return np.stack([x, y, z], axis=1), np.stack([vx, vy, vz], axis=1)

def omega_vector_from_ra_dec_period(ra_deg, dec_deg, period):
    axis = axis_from_ra_dec(ra_deg, dec_deg)
    omega = sidereal_to_rad_s(period)
    return omega * axis


#======================================================
# Constants
#======================================================


# Parameters relavent to N-Body simulator for all bodies from JPL's Solar System Dynamics, Planets: Physical Parameters and Gravity Fields and IAU 2021

original_G = 6.67430e-11  # m^3 kg^-1 s^-2

body_params = {
    "10":  {"mass_kg": 1.98847e30,  "j2": 0.0,         "radius_eq_m": 6.9634e8,   "RA_deg": 281.13,   "Dec_deg": 63.87,   "sid_per_days": 25.38},    # Sun
    "199": {"mass_kg": 3.3011e23,   "j2": 0.0,         "radius_eq_m": 2439700.0,  "RA_deg": 281.0103, "Dec_deg": 61.4155, "sid_per_days": 58.646},   # Mercury
    "299": {"mass_kg": 4.8675e24,   "j2": 4.458e-6,    "radius_eq_m": 6051800.0,  "RA_deg": 272.76,   "Dec_deg": 67.16,   "sid_per_days": -243.018}, # Venus
    "399": {"mass_kg": 5.97219e24,  "j2": 1.08263e-3,  "radius_eq_m": 6378137.0,  "RA_deg": 0.0,      "Dec_deg": 90,      "sid_per_days": 1},        # Earth
    "301": {"mass_kg": 7.342e22,    "j2": 2.032e-4,    "radius_eq_m": 1737400.0,  "RA_deg": 269.99,   "Dec_deg": 66.54,   "sid_per_days": 27.32},    # Moon
    "499": {"mass_kg": 6.4171e23,   "j2": 1.96045e-3,  "radius_eq_m": 3396200.0,  "RA_deg": 317.68,   "Dec_deg": 52.89,   "sid_per_days": 1.026},    # Mars
    "599": {"mass_kg": 1.89813e27,  "j2": 1.4736e-2,   "radius_eq_m": 71492000.0, "RA_deg": 268.057,  "Dec_deg": 64.495,  "sid_per_days": 0.41354},  # Jupiter
    "699": {"mass_kg": 568.317e24,  "j2": 1.62906e-2,  "radius_eq_m": 60268000.0, "RA_deg": 40.589,   "Dec_deg": 83.537,  "sid_per_days": 0.444},    # Saturn
    "799": {"mass_kg": 86.8099e24,  "j2": 3510.7e-6,   "radius_eq_m": 25559000.0, "RA_deg": 257.311,  "Dec_deg": -15.175, "sid_per_days": -0.7183},  # Uranus
    "899": {"mass_kg": 102.4092e24, "j2": 3539.3e-6,   "radius_eq_m": 24764000.0, "RA_deg": 299.36,   "Dec_deg": 43.46,   "sid_per_days": 0.67125}   # Neptune
}


# Bodies to validate
objs_horz = ["10", "199", "299", "399", "301", "499", "599", "699", "799", "899"]
# Currently validates against the Sun's barycenter, all major planets, and Earth's moon


#=====================================================================
# Get user input and query Horizons
#=====================================================================

print("Input parameters. Time step for the RK4 N-Body algorithm should be on the order of a few seconds. \n" \
"Time step for the JPL Horizons query should be on the order of a few hours to a few days. \n" \
"This is to prevent the calling of millions of lines of outputs from Horizons, while ensuring accuracy from the RK4 program. \n" \
"Time step for RK4 should be a simple number (ex: '30'), time step for Horizons should specify units (s, m, h, d,), (ex: '1d')")
dt = float(input("  Time step for RK4 N-Body algorithm, in seconds: "))
step = input("  Time step for the JPL Horizon's query: ")
step_num = to_seconds(step)

epochs = {'start': '2026-10-15', 'stop': '2027-10-15', 'step': step}

output_file_name = input("  Enter the name you wish to save the results CSV as: ") + ".csv"

horizons_data = {}
for name in objs_horz:
    pos, vel = horizons_positions(name, epochs)
    horizons_data[name] = {"pos": pos, "vel": vel,}

# Number of steps from Horizons
num_steps = horizons_data["399"]["pos"].shape[0]

# ====================================================
# Build WorldState from Horizons initial conditions
# ====================================================

world_states = {}
id_counter = 0

for name in objs_horz:
    id_counter += 1
    pos0 = horizons_data[name]["pos"][0]
    vel0 = horizons_data[name]["vel"][0]
    params = body_params[name]

    ra = params["RA_deg"]
    dec = params["Dec_deg"]
    period = params["sid_per_days"]

    ax_vec = axis_from_ra_dec(ra, dec )
    ax_quat = quat_from_2_vec(np.array([0.0,0.0,1.0]),ax_vec)
    w = omega_vector_from_ra_dec_period(ra, dec, period)

    world_states[name] = {
        "id": id_counter,
        "name": name,
        "ob_type": True,
        "sv": physics_core.StateVector(pos0.tolist(), vel0.tolist()),
        "rs": physics_core.RotationalState(ax_quat.tolist(), w.tolist()),
        "models": [],
        "data": params,
    }

# ====================================================
# Attach N-body + J2 models
# ====================================================

for obj in world_states.values():
    for other in world_states.values():
        if obj is other:
            continue
        mu = other["data"]["mass_kg"] * original_G
        
        attractor_id = other["id"]

        obj["models"].append(
            physics_core.PointGravity(mu, other["sv"].position, attractor_id)
        )

        j2 = other["data"]["j2"]
        r_eq = other["data"]["radius_eq_m"]
        rot_mat = rotation_matrix_from_ra_dec(other["data"]["RA_deg"], other["data"]["Dec_deg"])
        obj["models"].append(
            physics_core.J2(mu, other["sv"].position, r_eq, j2, attractor_id)
        )

# Adds the models to the objects in the world state
ws = physics_core.WorldState()
for obj in world_states.values():
    ws.add_object(obj["id"], obj["name"], obj["ob_type"], obj["sv"], obj["rs"], obj["models"])

# ====================================================
# Run simulation over same epochs
# ====================================================

sim_trajectories = {name: [] for name in world_states.keys()}

steps_per_horizons = int((step_num)/ dt)

sim_trajectories = {name: [] for name in world_states.keys()}

# Record t = 0
for obj in world_states.values():
    state = ws.get_state(obj["id"])
    sim_trajectories[obj["name"]].append(list(state[0].position))

# Step the RK4 N-Body integrator by dt until it has evaluated the same time span as Horizons
for step in tqdm(range(1, num_steps)):  
    for _ in range(steps_per_horizons):
        ws.step_all(dt)
    for obj in world_states.values():
        state = ws.get_state(obj["id"])
        sim_trajectories[obj["name"]].append(list(state[0].position))

# Convert to arrays
for name in sim_trajectories:
    sim_trajectories[name] = np.array(sim_trajectories[name])

# ====================================================
# Compare Horizons vs simulation
# ====================================================
bodies = list(world_states.keys())
errors_by_body = {}

# Compute errors for every body and print summary stats
for name in bodies:
  sim = np.asarray(sim_trajectories[name])
  hor = np.asarray(horizons_data[name]["pos"][: len(sim)])

  dist_err = np.linalg.norm(sim - hor, axis=1)
  errors_by_body[name] = dist_err

  print(
      f"{name}: mean position error = {dist_err.mean():.3e} m, "
      f"max = {dist_err.max():.3e} m"
  )

# Generate time array
hours_per_step = dt / 3600
steps = np.linspace(0, num_steps, num_steps)

# Write CSV cleanly using zip and writerows
with open(output_file_name, "w", newline="") as f:
  writer = csv.writer(f)

  # Header row
  writer.writerow(["steps"] + bodies)
  # Data rows
  error_columns = [errors_by_body[name][:num_steps] for name in bodies]
  writer.writerows(zip(steps, *error_columns))