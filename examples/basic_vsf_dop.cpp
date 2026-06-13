#include <iomanip>
#include <iostream>
#include <vector>

#include <vsfdop/vsf_dop.hpp>

namespace {

vsfdop::MatrixCell makeHexCell(int id, double x0, double x1, double y0, double y1, double z0, double z1) {
    vsfdop::MatrixCell cell;
    cell.id = id;
    cell.vertices = {
        {x0, y0, z0}, {x1, y0, z0}, {x1, y1, z0}, {x0, y1, z0},
        {x0, y0, z1}, {x1, y0, z1}, {x1, y1, z1}, {x0, y1, z1}};
    cell.faces = {
        {0, 3, 2, 1},
        {4, 5, 6, 7},
        {0, 1, 5, 4},
        {1, 2, 6, 5},
        {2, 3, 7, 6},
        {3, 0, 4, 7}};
    cell.updateBounds();
    return cell;
}

std::vector<vsfdop::MatrixCell> makeRegularGrid(int nx, int ny, int nz) {
    std::vector<vsfdop::MatrixCell> cells;
    cells.reserve(static_cast<std::size_t>(nx * ny * nz));
    int id = 0;
    for (int k = 0; k < nz; ++k) {
        for (int j = 0; j < ny; ++j) {
            for (int i = 0; i < nx; ++i) {
                cells.push_back(makeHexCell(
                    id++,
                    static_cast<double>(i), static_cast<double>(i + 1),
                    static_cast<double>(j), static_cast<double>(j + 1),
                    static_cast<double>(k), static_cast<double>(k + 1)));
            }
        }
    }
    return cells;
}

} // namespace

int main() {
    const std::vector<vsfdop::MatrixCell> matrixCells = makeRegularGrid(4, 4, 2);

    vsfdop::FractureFacet facet;
    facet.id = 1;
    facet.vertices = {
        {0.25, 0.25, 0.15},
        {3.75, 1.15, 0.85},
        {3.15, 3.60, 1.55},
        {0.45, 3.20, 1.05}};

    vsfdop::VsfDopOptions options;
    options.voxelResolutionX = 4;
    options.voxelResolutionY = 4;
    options.voxelResolutionZ = 2;
    options.dopType = 14;
    options.epsilon = 1.0e-9;

    const vsfdop::VsfDopResult result =
        vsfdop::detectMatrixFractureIntersections(matrixCells, {facet}, options);

    std::cout << "VSF candidates: " << result.vsfCandidateCount << '\n';
    std::cout << "DOP retained:   " << result.dopRetainedCount << '\n';
    std::cout << "Exact checks:   " << result.exactCheckCount << '\n';
    std::cout << "True pairs:     " << result.pairs.size() << '\n';
    std::cout << "Wall time (ms): " << std::fixed << std::setprecision(3) << result.wallTimeMs << '\n';

    for (const vsfdop::IntersectionPair& pair : result.pairs) {
        std::cout << "  matrixCell=" << pair.matrixCellId
                  << ", fractureFacet=" << pair.fractureFacetId << '\n';
    }

    return result.pairs.empty() ? 1 : 0;
}

