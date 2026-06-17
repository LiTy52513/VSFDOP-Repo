# VSF-DOP

This repository provides a compact C++17 implementation of the voxelised surface footprint and discrete orientation polytope (VSF-DOP) method for three-dimensional matrix-fracture intersection detection in non-matching grids.

The code package is intentionally limited to the geometric preprocessing stage. It does not include EDFM solvers, flow equations, well models, plotting scripts, benchmark runners, or visualization export utilities.

## Code-Package Status

The repository is organised as a minimal open-source code package for reproducible research and reuse:

- C++17 library with a stable public API.
- CMake build system with no external dependencies.
- In-memory executable example.
- CTest verification target.
- MIT license.
- Citation metadata and code/data availability notes.

## Contents

- `include/vsfdop/geometry.hpp`: basic 3D geometry types.
- `include/vsfdop/vsf_dop.hpp`: public VSF-DOP API.
- `src/vsf_dop.cpp`: VSF indexing, 14-DOP culling, and exact intersection detection.
- `examples/basic_vsf_dop.cpp`: minimal in-memory example.

## Build

```powershell
cmake -S . -B build
cmake --build build --config Release
```

On single-configuration generators, the executable is usually written to `build/basic_vsf_dop`. On Visual Studio generators, it is usually written to `build/Release/basic_vsf_dop.exe`.

## Test

```powershell
cmake -S . -B build -DVSFDOP_BUILD_TESTS=ON
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

The current verification target checks a deterministic in-memory matrix-fracture case and validates the VSF candidate count, DOP-retained count, exact-check count, and final true-pair count.

## Run the Example

```powershell
.\build\Release\basic_vsf_dop.exe
```

Expected output fields:

- `VSF candidates`: matrix-cell candidates returned by the voxelised surface footprint query.
- `DOP retained`: candidates retained after 14-DOP projection culling.
- `Exact checks`: candidates processed by exact geometric intersection detection.
- `True pairs`: final matrix-fracture intersecting pairs.

## Minimal API

```cpp
#include <vsfdop/vsf_dop.hpp>

vsfdop::VsfDopOptions options;
options.voxelResolutionX = 8;
options.voxelResolutionY = 8;
options.voxelResolutionZ = 4;

vsfdop::VsfDopResult result =
    vsfdop::detectMatrixFractureIntersections(matrixCells, fractureFacets, options);
```

The input matrix cells are convex polyhedra described by vertices and face connectivity. Fracture facets are planar polygons and are internally triangulated by a fan triangulation.

## Public Interface

- `vsfdop::VsfDopOptions`
  - `voxelResolutionX`, `voxelResolutionY`, `voxelResolutionZ`: background voxel-grid resolution.
  - `dopType`: DOP variant. The current release supports 14-DOP.
  - `epsilon`: geometric tolerance used in filtering and intersection tests.
- `vsfdop::VsfDopResult`
  - `pairs`: final matrix-fracture intersecting pairs.
  - `vsfCandidateCount`: candidates returned by VSF filtering before DOP culling.
  - `dopRetainedCount`: candidates retained after DOP culling.
  - `exactCheckCount`: candidates processed by exact intersection detection.
  - `wallTimeMs`: wall-clock time for the full matrix-fracture detection call.

## Scope

This code package is designed for reproducible method inspection. It contains only the core 3D VSF-DOP functionality and a minimal executable example.

## Reproducibility Notes

The example is intentionally small and self-contained so that readers can build and run it without external mesh files. It demonstrates the full VSF-DOP pipeline but is not intended to reproduce the large benchmark figures in the associated study.

For scholarly use, cite a versioned repository release rather than a moving branch. A recommended workflow is:

1. Use a tagged release, for example `v0.1.0`.
2. Archive the release with Zenodo if a DOI is needed for citation or reproducibility.

## License

This code is released under the MIT License. See `LICENSE`.
