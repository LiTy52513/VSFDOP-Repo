#pragma once

#include <vector>

#include "vsfdop/geometry.hpp"

namespace vsfdop {

struct VsfDopOptions {
    int voxelResolutionX{16};
    int voxelResolutionY{16};
    int voxelResolutionZ{16};
    int dopType{14};
    double epsilon{1.0e-9};
};

struct VsfDopResult {
    std::vector<IntersectionPair> pairs;
    long long vsfCandidateCount{0};
    long long dopRetainedCount{0};
    long long exactCheckCount{0};
    double wallTimeMs{0.0};
};

VsfDopResult detectMatrixFractureIntersections(
    const std::vector<MatrixCell>& matrixCells,
    const std::vector<FractureFacet>& fractureFacets,
    const VsfDopOptions& options = {});

} // namespace vsfdop

