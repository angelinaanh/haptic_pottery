# Virtual Pottery

A virtual pottery simulator with haptic feedback. The user shapes a body of clay
turning on a potter's wheel with the stylus of a haptic device (Sensable
PHANToM) or the mouse, and feels the clay push back. Built on **CHAI3D 3.3** and
**GLFW**.

The clay is modelled as circular sector elements (Han 2007, Lee 2008), with
response force propagation between neighbouring elements (Lee 2009).

## Two methods

The program has two interaction methods to compare. Switch between them with the
**Baseline** / **Method 2** buttons in the top right corner, or the `0` / `2` keys.

| | Baseline | Method 2 |
|---|---|---|
| Contact force | mean penetration `(c_v/n) Σ pᵢfᵢ` | penetration volume `k_V Σ pᵢAᵢfᵢ` |
| Full model | 1 kHz, inside the servo loop | 250 Hz, own thread + local stiffness |
| Propagation | tetragonal hulls (Lee 2009) | separable convolution |
| Force filter | 10 Hz Butterworth | none needed |

### Method 2

- **Penetration-volume force:** `F = k_V · Σ pᵢ·Aᵢ·fᵢ`, with `Aᵢ = R·Δθ·Δh`.
  Newly touched elements enter with `p ≈ 0`, so the force is continuous and
  nearly independent of the grid resolution. Friction is `μ|F_n|`; viscous
  resistance scales with the contact area, `−c2·min(1, A/A_ref)·v`.
  Code: `CollisionResponder::contactForce`.
- **Multirate local stiffness:** the physics thread runs the full model at
  250 Hz and computes `K = k_V Σ Aᵢ fᵢfᵢᵀ`. The 1 kHz servo loop only evaluates
  `F ≈ F₀ − K(x − x₀)`, cross-fading between consecutive models.
  Code: `Haptic_Thread/LocalForceModel.*`.
- **Separable propagation:** the `1/((c_l δ)²+1)` kernel of Lee 2009 is replaced
  by a Gaussian with the same total weight, applied along rows and then columns
  within a band around the contact.
  Code: `CollisionResponder::propagateSeparable`.

## Running from the Visual Studio Code terminal

Requirements: Visual Studio 2026 (C++ workload), CMake, a built CHAI3D 3.3, and
Python 3 with `matplotlib` and `numpy` (only needed for the plots).

Open the `VirtualPottery_Project` folder in VS Code, open a terminal
(`` Ctrl+` ``, PowerShell) and run:

```powershell
# 1. Configure (first time only). Drop -Dchai3d_DIR if CMake finds CHAI3D by itself.
cmake -S . -B build -G "Visual Studio 18 2026" -A x64 `
      -Dchai3d_DIR="C:/Users/Admin/Desktop/Project/chai3d/chai3d-3.3.0/out/build/Visual Studio Community 2026 Release - amd64"

# 2. Build
cmake --build build --config Release

# 3. Run
.\bin\win-x64\VirtualPottery.exe                # Method 2 (default)
.\bin\win-x64\VirtualPottery.exe --method 0     # Baseline
.\bin\win-x64\VirtualPottery.exe --open         # start with the clay opened (hollow)

# 4. Compare the two methods (about 1 minute, exits by itself), then plot
.\bin\win-x64\VirtualPottery.exe --bench
python Evaluation\plot_comparison.py
```

After changing the code, only steps 2 and 3 need to be repeated. On another
machine, change the `chai3d_DIR` path to where CHAI3D was built.

## Controls

Without a haptic device the mouse takes its place: the tool follows the cursor,
the scroll wheel sets the depth, and holding the right button spins the wheel.

| Key | Action |
|---|---|
| `0` · `2` (or the buttons) | Baseline · Method 2 |
| `SPACE` · `↑` `↓` | spin the wheel · wheel speed |
| `[` `]` · `,` `.` | tool size · grid resolution |
| `R` · `O` | reset the clay · open/close (hollow) |
| `K` `V` `D` `C` (`SHIFT` to lower) | stiffness · viscosity · damping · friction |
| `P` · `W` | propagation on/off · wireframe |
| `F` · `F1` · `Q` | fullscreen · key list · quit |

## Results

`--bench` drives a scripted tool through the whole system in 12 runs: pushing
into the clay with the wheel stopped (as in Lee 2008, Fig. 8) and cutting a
groove with the wheel turning (as in Lee 2009), at several resolutions. Per-tick
data is in `Results/*.csv`; the full table is in
[Results/summary.md](Results/summary.md).

| | Baseline | Method 2 |
|---|---|---|
| Largest force step before filtering (groove 400×360) | 32.0 mN | 6.2 mN |
| Delay added by the force filter | 23 ms | 0 |
| Peak force change between 200×180 and 800×360 grids | 5.0 % | 1.3 % |
| Physics step, groove 400×360 | 349 µs | 115 µs |
| Physics step, groove 1600×720 | 1515 µs | 715 µs |
| Time inside the 1 kHz loop, 1600×720 (mean) | 1516 µs | 0.5 µs |
| Actual servo rate, 1600×720 | 640 Hz | 1000 Hz |

Timings depend on the machine and vary between runs.

| Figure | Content |
|---|---|
| ![](Results/fig1_push_force_depth_contacts.png) | Force, depth and colliding elements while pushing in |
| ![](Results/fig2_force_detail.png) | 100 ms of force at tick level while cutting a groove |
| ![](Results/fig3_force_steps.png) | Largest force change between two ticks |
| ![](Results/fig4_resolution.png) | The same push on a coarse and a fine grid |
| ![](Results/fig5_timing.png) | Physics step time and time inside the 1 kHz loop |

Method 2's peak force is lower because the volume force grows with the square of
the depth; it can be tuned with the `K` key.

## Project structure

```
VirtualPottery_Project/
├── Core_Data_Structures/   sector elements, grid, wheel, method settings
├── Haptic_Thread/          haptic device, collision detection and response, local force model
├── Graphic_Thread/         data sync to the GPU, scene and UI drawing, shaders
├── Evaluation/             benchmark and plotting script
├── Results/                CSV data, figures, summary.md
└── main.cpp                three threads (servo 1 kHz, physics 250 Hz, graphics) and input
```

## Limitations

**Model**
- Each element stores only a radius per (angle, height), so the clay is a
  surface of revolution with local bumps. Overhangs, folds, cracks and pieces
  coming off cannot be represented, and the wall thickness of a hollow body is
  fixed.
- The clay is viscous but has no elasticity, volume conservation or moisture.
  Material pushed in disappears instead of moving elsewhere.
- The tool is always a sphere; real pottery tools (ribs, wire, loop tools) are
  not modelled.

**Haptics**
- Not yet tested with a real PHANToM. All numbers come from a scripted tool and
  the mouse, so `k_V`, `μ`, `A_ref` and the physics rate still need tuning with
  the device in hand.
- Method 2's local model does not know how the clay will deform within the next
  4 ms; that part only updates at 250 Hz (with cross-fading).
- Method 2's propagation kernel is a Gaussian with the same total weight as the
  Lee 2009 kernel, not that kernel itself.
- At the same stiffness setting Method 2 feels softer than the baseline at small
  depths, so switching methods also changes how stiff the clay feels.

**Code**
- No unit tests; correctness is checked only through the benchmark and by eye.
- `main.cpp` keeps the system state in global variables, which makes the
  threads harder to reason about and the code harder to reuse.
- The benchmark output folder is compiled in as an absolute path, so moving the
  project requires a rebuild.
- Rendering uses OpenGL 2.1 and GLSL 1.20 (fixed-function state, client arrays),
  which is deprecated.
- Only built and run on Windows; the Linux and macOS branches in
  `CMakeLists.txt` are untested.
- CHAI3D 3.3 deletes twice in `~cViewport()`, so the viewport is deliberately
  not deleted on exit.
- The finished pot cannot be saved or exported.

## Future work

- Test with a real PHANToM and calibrate the parameters with users; run a user
  study comparing how the two methods feel.
- Add elasticity and volume conservation, so clay pushed in by the tool rises or
  bulges elsewhere as real clay does.
- Support other tool shapes (rib, wire, loop) and two-handed interaction with
  two devices.
- Predict the deformation inside the local model (for example by also
  linearising the deformation), to reduce the remaining 4 ms lag.
- Move propagation and collision detection to the GPU (compute shaders) to allow
  finer grids.
- Export the pot as OBJ/STL for 3D printing, and save/load sessions.
- Port rendering to modern OpenGL, add unit tests, and replace the global state
  in `main.cpp` with a system class.
- Make the results folder configurable at run time and test the Linux build.
