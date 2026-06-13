#include "vsfdop/vsf_dop.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <set>
#include <stdexcept>
#include <unordered_map>

namespace vsfdop {
namespace {

constexpr int kDopDirectionCount = 7;

double component(const Vec3& value, int axis) {
    if (axis == 0) {
        return value.x;
    }
    if (axis == 1) {
        return value.y;
    }
    return value.z;
}

double projection14(const Vec3& point, int axis) {
    switch (axis) {
    case 0: return point.x;
    case 1: return point.y;
    case 2: return point.z;
    case 3: return point.x + point.y + point.z;
    case 4: return point.x - point.y + point.z;
    case 5: return point.x + point.y - point.z;
    case 6: return point.x - point.y - point.z;
    default: return 0.0;
    }
}

struct Dop14 {
    std::array<double, kDopDirectionCount> minValues{};
    std::array<double, kDopDirectionCount> maxValues{};

    Dop14() {
        reset();
    }

    void reset() {
        minValues.fill(std::numeric_limits<double>::infinity());
        maxValues.fill(-std::numeric_limits<double>::infinity());
    }

    void addPoint(const Vec3& point) {
        for (int axis = 0; axis < kDopDirectionCount; ++axis) {
            const double value = projection14(point, axis);
            minValues[axis] = std::min(minValues[axis], value);
            maxValues[axis] = std::max(maxValues[axis], value);
        }
    }

    static Dop14 fromTriangle(const Triangle& triangle) {
        Dop14 dop;
        dop.addPoint(triangle.a);
        dop.addPoint(triangle.b);
        dop.addPoint(triangle.c);
        return dop;
    }

    bool overlaps(const AABB& box, double epsilon) const {
        const std::array<Vec3, 8> corners = {
            Vec3{box.min.x, box.min.y, box.min.z},
            Vec3{box.min.x, box.min.y, box.max.z},
            Vec3{box.min.x, box.max.y, box.min.z},
            Vec3{box.min.x, box.max.y, box.max.z},
            Vec3{box.max.x, box.min.y, box.min.z},
            Vec3{box.max.x, box.min.y, box.max.z},
            Vec3{box.max.x, box.max.y, box.min.z},
            Vec3{box.max.x, box.max.y, box.max.z}};

        for (int axis = 0; axis < kDopDirectionCount; ++axis) {
            double boxMin = std::numeric_limits<double>::infinity();
            double boxMax = -std::numeric_limits<double>::infinity();
            for (const Vec3& corner : corners) {
                const double value = projection14(corner, axis);
                boxMin = std::min(boxMin, value);
                boxMax = std::max(boxMax, value);
            }
            if (maxValues[axis] < boxMin - epsilon || minValues[axis] > boxMax + epsilon) {
                return false;
            }
        }
        return true;
    }
};

bool axisSeparatesTriangleAndBox(
    const Vec3& axis,
    const std::array<Vec3, 3>& triangle,
    const Vec3& boxHalfSize,
    double epsilon) {

    if (normSquared(axis) <= epsilon * epsilon) {
        return false;
    }

    double triMin = dot(axis, triangle[0]);
    double triMax = triMin;
    for (int i = 1; i < 3; ++i) {
        const double value = dot(axis, triangle[i]);
        triMin = std::min(triMin, value);
        triMax = std::max(triMax, value);
    }

    const double radius =
        boxHalfSize.x * std::abs(axis.x) +
        boxHalfSize.y * std::abs(axis.y) +
        boxHalfSize.z * std::abs(axis.z);

    return triMin > radius + epsilon || triMax < -radius - epsilon;
}

bool triangleIntersectsAABB(const Triangle& triangle, const AABB& box, double epsilon) {
    const AABB triangleBox = boundsOf(triangle);
    if (!triangleBox.overlaps(box, epsilon)) {
        return false;
    }

    const Vec3 center = box.center();
    const Vec3 halfSize = box.halfSize() + Vec3{epsilon, epsilon, epsilon};
    const std::array<Vec3, 3> tri = {
        triangle.a - center,
        triangle.b - center,
        triangle.c - center};

    for (int axis = 0; axis < 3; ++axis) {
        double triMin = component(tri[0], axis);
        double triMax = triMin;
        for (int i = 1; i < 3; ++i) {
            const double value = component(tri[i], axis);
            triMin = std::min(triMin, value);
            triMax = std::max(triMax, value);
        }
        const double radius = component(halfSize, axis);
        if (triMin > radius + epsilon || triMax < -radius - epsilon) {
            return false;
        }
    }

    const std::array<Vec3, 3> edges = {
        tri[1] - tri[0],
        tri[2] - tri[1],
        tri[0] - tri[2]};
    const Vec3 normal = cross(edges[0], edges[1]);
    if (axisSeparatesTriangleAndBox(normal, tri, halfSize, epsilon)) {
        return false;
    }

    const std::array<Vec3, 3> boxAxes = {
        Vec3{1.0, 0.0, 0.0},
        Vec3{0.0, 1.0, 0.0},
        Vec3{0.0, 0.0, 1.0}};
    for (const Vec3& edge : edges) {
        for (const Vec3& boxAxis : boxAxes) {
            if (axisSeparatesTriangleAndBox(cross(edge, boxAxis), tri, halfSize, epsilon)) {
                return false;
            }
        }
    }

    return true;
}

int dominantAxis(const Vec3& normal) {
    const double ax = std::abs(normal.x);
    const double ay = std::abs(normal.y);
    const double az = std::abs(normal.z);
    if (ax >= ay && ax >= az) {
        return 0;
    }
    if (ay >= ax && ay >= az) {
        return 1;
    }
    return 2;
}

struct Point2 {
    double x{0.0};
    double y{0.0};
};

Point2 projectTo2D(const Vec3& point, int droppedAxis) {
    if (droppedAxis == 0) {
        return Point2{point.y, point.z};
    }
    if (droppedAxis == 1) {
        return Point2{point.x, point.z};
    }
    return Point2{point.x, point.y};
}

double orient2D(const Point2& a, const Point2& b, const Point2& c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool onSegment2D(const Point2& a, const Point2& b, const Point2& point, double epsilon) {
    return std::abs(orient2D(a, b, point)) <= epsilon &&
           point.x >= std::min(a.x, b.x) - epsilon &&
           point.x <= std::max(a.x, b.x) + epsilon &&
           point.y >= std::min(a.y, b.y) - epsilon &&
           point.y <= std::max(a.y, b.y) + epsilon;
}

bool segmentsIntersect2D(const Point2& a, const Point2& b, const Point2& c, const Point2& d, double epsilon) {
    const double o1 = orient2D(a, b, c);
    const double o2 = orient2D(a, b, d);
    const double o3 = orient2D(c, d, a);
    const double o4 = orient2D(c, d, b);

    if (((o1 > epsilon && o2 < -epsilon) || (o1 < -epsilon && o2 > epsilon)) &&
        ((o3 > epsilon && o4 < -epsilon) || (o3 < -epsilon && o4 > epsilon))) {
        return true;
    }

    return onSegment2D(a, b, c, epsilon) ||
           onSegment2D(a, b, d, epsilon) ||
           onSegment2D(c, d, a, epsilon) ||
           onSegment2D(c, d, b, epsilon);
}

bool pointOnTriangle(const Vec3& point, const Triangle& triangle, double epsilon) {
    const Vec3 v0 = triangle.b - triangle.a;
    const Vec3 v1 = triangle.c - triangle.a;
    const Vec3 normal = cross(v0, v1);
    const double normalLength = norm(normal);
    if (normalLength <= epsilon) {
        return false;
    }

    if (std::abs(dot(point - triangle.a, normal)) > epsilon * normalLength) {
        return false;
    }

    const Vec3 v2 = point - triangle.a;
    const double d00 = dot(v0, v0);
    const double d01 = dot(v0, v1);
    const double d11 = dot(v1, v1);
    const double d20 = dot(v2, v0);
    const double d21 = dot(v2, v1);
    const double denom = d00 * d11 - d01 * d01;
    if (std::abs(denom) <= epsilon) {
        return false;
    }

    const double v = (d11 * d20 - d01 * d21) / denom;
    const double w = (d00 * d21 - d01 * d20) / denom;
    const double u = 1.0 - v - w;
    return u >= -epsilon && v >= -epsilon && w >= -epsilon;
}

bool segmentIntersectsTriangle(const Vec3& start, const Vec3& end, const Triangle& triangle, double epsilon) {
    const Vec3 normal = cross(triangle.b - triangle.a, triangle.c - triangle.a);
    const double normalLength = norm(normal);
    if (normalLength <= epsilon) {
        return false;
    }

    const Vec3 direction = end - start;
    const double denominator = dot(normal, direction);
    const double signedStart = dot(normal, start - triangle.a);

    if (std::abs(denominator) > epsilon) {
        const double t = -signedStart / denominator;
        if (t < -epsilon || t > 1.0 + epsilon) {
            return false;
        }
        const Vec3 point = start + direction * t;
        return pointOnTriangle(point, triangle, epsilon);
    }

    if (std::abs(signedStart) > epsilon * normalLength ||
        std::abs(dot(normal, end - triangle.a)) > epsilon * normalLength) {
        return false;
    }

    if (pointOnTriangle(start, triangle, epsilon) || pointOnTriangle(end, triangle, epsilon)) {
        return true;
    }

    const int droppedAxis = dominantAxis(normal);
    const Point2 s0 = projectTo2D(start, droppedAxis);
    const Point2 s1 = projectTo2D(end, droppedAxis);
    const std::array<Point2, 3> t = {
        projectTo2D(triangle.a, droppedAxis),
        projectTo2D(triangle.b, droppedAxis),
        projectTo2D(triangle.c, droppedAxis)};

    return segmentsIntersect2D(s0, s1, t[0], t[1], epsilon) ||
           segmentsIntersect2D(s0, s1, t[1], t[2], epsilon) ||
           segmentsIntersect2D(s0, s1, t[2], t[0], epsilon);
}

Vec3 centroidOf(const MatrixCell& cell) {
    Vec3 center;
    for (const Vec3& vertex : cell.vertices) {
        center += vertex;
    }
    return cell.vertices.empty() ? center : center / static_cast<double>(cell.vertices.size());
}

Vec3 outwardFaceNormal(const MatrixCell& cell, const std::vector<std::size_t>& face, const Vec3& cellCentroid) {
    if (face.size() < 3) {
        return Vec3{};
    }
    Vec3 normal = cross(
        cell.vertices[face[1]] - cell.vertices[face[0]],
        cell.vertices[face[2]] - cell.vertices[face[0]]);
    if (dot(normal, cellCentroid - cell.vertices[face[0]]) > 0.0) {
        normal = -normal;
    }
    return normalized(normal);
}

bool pointInsideConvexCell(const Vec3& point, const MatrixCell& cell, double epsilon) {
    const Vec3 cellCentroid = centroidOf(cell);
    for (const auto& face : cell.faces) {
        if (face.size() < 3) {
            continue;
        }
        const Vec3 normal = outwardFaceNormal(cell, face, cellCentroid);
        if (dot(normal, point - cell.vertices[face[0]]) > epsilon) {
            return false;
        }
    }
    return true;
}

std::vector<Triangle> triangulateFacet(const FractureFacet& facet) {
    std::vector<Triangle> triangles;
    if (facet.vertices.size() < 3) {
        return triangles;
    }
    triangles.reserve(facet.vertices.size() - 2);
    for (std::size_t i = 1; i + 1 < facet.vertices.size(); ++i) {
        triangles.push_back(Triangle{facet.vertices[0], facet.vertices[i], facet.vertices[i + 1]});
    }
    return triangles;
}

std::vector<Triangle> triangulateFace(const MatrixCell& cell, const std::vector<std::size_t>& face) {
    std::vector<Triangle> triangles;
    if (face.size() < 3) {
        return triangles;
    }
    triangles.reserve(face.size() - 2);
    for (std::size_t i = 1; i + 1 < face.size(); ++i) {
        triangles.push_back(Triangle{cell.vertices[face[0]], cell.vertices[face[i]], cell.vertices[face[i + 1]]});
    }
    return triangles;
}

std::vector<std::pair<std::size_t, std::size_t>> uniqueCellEdges(const MatrixCell& cell) {
    std::set<std::pair<std::size_t, std::size_t>> edges;
    for (const auto& face : cell.faces) {
        for (std::size_t i = 0; i < face.size(); ++i) {
            const std::size_t a = face[i];
            const std::size_t b = face[(i + 1) % face.size()];
            edges.insert(std::minmax(a, b));
        }
    }
    return {edges.begin(), edges.end()};
}

bool segmentIntersectsFace(const Vec3& start, const Vec3& end, const MatrixCell& cell, const std::vector<std::size_t>& face, double epsilon) {
    for (const Triangle& faceTriangle : triangulateFace(cell, face)) {
        if (segmentIntersectsTriangle(start, end, faceTriangle, epsilon)) {
            return true;
        }
    }
    return false;
}

bool triangleIntersectsCell(const Triangle& triangle, const MatrixCell& cell, double epsilon) {
    if (!boundsOf(triangle).overlaps(cell.bounds, epsilon)) {
        return false;
    }

    if (pointInsideConvexCell(triangle.a, cell, epsilon) ||
        pointInsideConvexCell(triangle.b, cell, epsilon) ||
        pointInsideConvexCell(triangle.c, cell, epsilon)) {
        return true;
    }

    for (const Vec3& vertex : cell.vertices) {
        if (pointOnTriangle(vertex, triangle, epsilon)) {
            return true;
        }
    }

    const std::array<std::pair<Vec3, Vec3>, 3> triangleEdges = {
        std::make_pair(triangle.a, triangle.b),
        std::make_pair(triangle.b, triangle.c),
        std::make_pair(triangle.c, triangle.a)};

    for (const auto& edge : triangleEdges) {
        for (const auto& face : cell.faces) {
            if (segmentIntersectsFace(edge.first, edge.second, cell, face, epsilon)) {
                return true;
            }
        }
    }

    for (const auto& edge : uniqueCellEdges(cell)) {
        if (segmentIntersectsTriangle(cell.vertices[edge.first], cell.vertices[edge.second], triangle, epsilon)) {
            return true;
        }
    }

    return false;
}

struct VoxelGrid {
    AABB bounds{AABB::empty()};
    Vec3 voxelSize;
    int nx{0};
    int ny{0};
    int nz{0};
    double epsilon{1.0e-9};
    std::unordered_map<int, std::vector<std::size_t>> bins;

    int flatIndex(int i, int j, int k) const {
        return i + nx * (j + ny * k);
    }

    int clampIndex(int value, int upper) const {
        return std::max(0, std::min(value, upper - 1));
    }

    void getIJK(const Vec3& point, int& i, int& j, int& k) const {
        i = clampIndex(static_cast<int>(std::floor((point.x - bounds.min.x) / voxelSize.x)), nx);
        j = clampIndex(static_cast<int>(std::floor((point.y - bounds.min.y) / voxelSize.y)), ny);
        k = clampIndex(static_cast<int>(std::floor((point.z - bounds.min.z) / voxelSize.z)), nz);
    }

    AABB voxelBox(int i, int j, int k) const {
        const Vec3 minPoint{
            bounds.min.x + static_cast<double>(i) * voxelSize.x,
            bounds.min.y + static_cast<double>(j) * voxelSize.y,
            bounds.min.z + static_cast<double>(k) * voxelSize.z};
        return AABB{minPoint, minPoint + voxelSize};
    }

    void build(const std::vector<MatrixCell>& cells, const VsfDopOptions& options) {
        nx = std::max(1, options.voxelResolutionX);
        ny = std::max(1, options.voxelResolutionY);
        nz = std::max(1, options.voxelResolutionZ);
        epsilon = options.epsilon;
        bins.clear();
        bounds = AABB::empty();

        for (const MatrixCell& cell : cells) {
            bounds.expand(cell.bounds);
        }

        bounds.pad(std::max(epsilon, 1.0e-8));
        Vec3 extent = bounds.max - bounds.min;
        if (std::abs(extent.x) <= epsilon) {
            bounds.max.x += 1.0;
            extent.x = 1.0;
        }
        if (std::abs(extent.y) <= epsilon) {
            bounds.max.y += 1.0;
            extent.y = 1.0;
        }
        if (std::abs(extent.z) <= epsilon) {
            bounds.max.z += 1.0;
            extent.z = 1.0;
        }

        voxelSize = Vec3{
            extent.x / static_cast<double>(nx),
            extent.y / static_cast<double>(ny),
            extent.z / static_cast<double>(nz)};

        for (std::size_t cellIndex = 0; cellIndex < cells.size(); ++cellIndex) {
            int iMin = 0;
            int jMin = 0;
            int kMin = 0;
            int iMax = 0;
            int jMax = 0;
            int kMax = 0;
            getIJK(cells[cellIndex].bounds.min, iMin, jMin, kMin);
            getIJK(cells[cellIndex].bounds.max, iMax, jMax, kMax);

            for (int k = kMin; k <= kMax; ++k) {
                for (int j = jMin; j <= jMax; ++j) {
                    for (int i = iMin; i <= iMax; ++i) {
                        bins[flatIndex(i, j, k)].push_back(cellIndex);
                    }
                }
            }
        }
    }

    std::vector<std::size_t> queryTriangle(const Triangle& triangle) const {
        std::vector<std::size_t> rawCandidates;
        const AABB triangleBox = boundsOf(triangle);

        int iMin = 0;
        int jMin = 0;
        int kMin = 0;
        int iMax = 0;
        int jMax = 0;
        int kMax = 0;
        getIJK(triangleBox.min, iMin, jMin, kMin);
        getIJK(triangleBox.max, iMax, jMax, kMax);

        for (int k = kMin; k <= kMax; ++k) {
            for (int j = jMin; j <= jMax; ++j) {
                for (int i = iMin; i <= iMax; ++i) {
                    if (!triangleIntersectsAABB(triangle, voxelBox(i, j, k), epsilon)) {
                        continue;
                    }
                    const auto iter = bins.find(flatIndex(i, j, k));
                    if (iter != bins.end()) {
                        rawCandidates.insert(rawCandidates.end(), iter->second.begin(), iter->second.end());
                    }
                }
            }
        }

        std::sort(rawCandidates.begin(), rawCandidates.end());
        rawCandidates.erase(std::unique(rawCandidates.begin(), rawCandidates.end()), rawCandidates.end());
        return rawCandidates;
    }
};

std::vector<MatrixCell> prepareCells(const std::vector<MatrixCell>& inputCells) {
    std::vector<MatrixCell> cells = inputCells;
    for (MatrixCell& cell : cells) {
        cell.updateBounds();
    }
    return cells;
}

} // namespace

VsfDopResult detectMatrixFractureIntersections(
    const std::vector<MatrixCell>& matrixCells,
    const std::vector<FractureFacet>& fractureFacets,
    const VsfDopOptions& options) {

    if (options.dopType != 14) {
        throw std::invalid_argument("VSF-DOP currently supports only dopType = 14.");
    }

    VsfDopResult result;
    const auto startTime = std::chrono::high_resolution_clock::now();

    if (matrixCells.empty() || fractureFacets.empty()) {
        return result;
    }

    const std::vector<MatrixCell> cells = prepareCells(matrixCells);
    VoxelGrid grid;
    grid.build(cells, options);

    std::set<std::pair<int, int>> uniquePairs;

    for (const FractureFacet& facet : fractureFacets) {
        for (const Triangle& triangle : triangulateFacet(facet)) {
            const std::vector<std::size_t> vsfCandidates = grid.queryTriangle(triangle);
            result.vsfCandidateCount += static_cast<long long>(vsfCandidates.size());

            const Dop14 dop = Dop14::fromTriangle(triangle);
            for (const std::size_t cellIndex : vsfCandidates) {
                const MatrixCell& cell = cells[cellIndex];
                if (!dop.overlaps(cell.bounds, options.epsilon)) {
                    continue;
                }

                ++result.dopRetainedCount;
                ++result.exactCheckCount;
                if (triangleIntersectsCell(triangle, cell, options.epsilon)) {
                    const std::pair<int, int> key{cell.id, facet.id};
                    if (uniquePairs.insert(key).second) {
                        result.pairs.push_back(IntersectionPair{cell.id, facet.id});
                    }
                }
            }
        }
    }

    const auto endTime = std::chrono::high_resolution_clock::now();
    result.wallTimeMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();
    return result;
}

} // namespace vsfdop

