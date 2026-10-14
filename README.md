# N-Body-Physics-Engine

# High-Performance Astrodynamics Engine

A hybrid physics simulation engine featuring a **Rust-powered multi-body numerical integrator** bridged to **Python** via **PyO3** and **Maturin**. Designed for real-time orbital mechanics, N-body gravitation, and trajectory validation against NASA JPL Horizons.

---

## Features

* **High-Performance Rust Core:** Leverages data-parallel iteration via **Rayon** for high-frequency RK4 numerical integration across celestial bodies and spacecraft.
* **Python Host Integration:** Compiled directly into a high-speed Python module using PyO3/Maturin, allowing seamless integration with scientific plotting tools and game loops.
* **Validation:** Benchmarked and verified against NASA JPL Horizons ephemerides over year-long integration windows.

---

## Building & Installation

Because this engine uses a **Rust** core bound to **Python**, building the project requires both a Rust toolchain and an active Python environment.

### Prerequisites
* **Rust Toolchain:** Installed via `rustup` (recommended).
* **Python 3.8+** installed on your system.

### Step-by-Step Compilation Guide (Windows PowerShell or IDE Terminal)

1. **Navigate to the Rust crate directory:**
```
 cd physics_core
```

Create and activate a **Python** virtual environment:
```
.\.venv\Scripts\Activate.ps1
```

Install Maturin and build dependencies:
```
pip install maturin
```
Compile and install the **Rust** Modules
```
maturin develop
```

## Performance Note

The **Rust** physics core uses the **Rayon** library for parallel processing. This is important for N-Body Physics to prevent excessive run-time. HOWEVER, this can use significant CPU resources (up to 95% of my CPU for up to a few minutes straight). If it is desired to limit the CPU usage of this code, limit the number of threads **Rayon** is allowed to use:
```
export RAYON_NUM_THREADS=4  # Linux / macOS
```
```
$env:RAYON_NUM_THREADS="4"  # Windows PowerShell
```
## Validation

To ensure fidelity, the engine's trajectory outputs were validated against **NASA JPL Horizons** ephemeris data for real-world solar objects. The code that performed these validations and the results of the validations is in the "validations" folder

## License
This project is open-sourced under the terms of the **MIT License**. See the LICENSE file for details. 
