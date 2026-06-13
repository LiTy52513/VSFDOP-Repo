#pragma once

#include <array>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace vsfdop {

struct Vec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    constexpr Vec3() = default;
    constexpr Vec3(double xValue, double yValue, double zValue)
        : x(xValue), y(yValue), z(zValue) {}

    Vec3& operator+=(const Vec3& rhs) {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    Vec3& operator-=(const Vec3& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }
};

inline Vec3 operator+(Vec3 lhs, const Vec3& rhs) {
    lhs += rhs;
    return lhs;
}

inline Vec3 operator-(Vec3 lhs, const Vec3& rhs) {
    lhs -= rhs;
    return lhs;
}

inline Vec3 operator-(const Vec3& value) {
    return Vec3{-value.x, -value.y, -value.z};
}

inline Vec3 operator*(const Vec3& value, double scale) {
    return Vec3{value.x * scale, value.y * scale, value.z * scale};
}

inline Vec3 operator*(double scale, const Vec3& value) {
    return value * scale;
}

inline Vec3 operator/(const Vec3& value, double scale) {
    return Vec3{value.x / scale, value.y / scale, value.z / scale};
}

inline double dot(const Vec3& lhs, const Vec3& rhs) {
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

inline Vec3 cross(const Vec3& lhs, const Vec3& rhs) {
    return Vec3{
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x};
}

inline double normSquared(const Vec3& value) {
    return dot(value, value);
}

inline double norm(const Vec3& value) {
    return std::sqrt(normSquared(value));
}

inline Vec3 normalized(const Vec3& value, double epsilon = 1.0e-12) {
    const double length = norm(value);
    return length > epsilon ? value / length : Vec3{};
}

struct AABB {
    Vec3 min;
    Vec3 max;

    static AABB empty() {
        const double inf = std::numeric_limits<double>::infinity();
        return AABB{Vec3{inf, inf, inf}, Vec3{-inf, -inf, -inf}};
    }

    void expand(const Vec3& point) {
        min.x = std::min(min.x, point.x);
        min.y = std::min(min.y, point.y);
        min.z = std::min(min.z, point.z);
        max.x = std::max(max.x, point.x);
        max.y = std::max(max.y, point.y);
        max.z = std::max(max.z, point.z);
    }

    void expand(const AABB& other) {
        expand(other.min);
        expand(other.max);
    }

    void pad(double epsilon) {
        min = min - Vec3{epsilon, epsilon, epsilon};
        max = max + Vec3{epsilon, epsilon, epsilon};
    }

    bool overlaps(const AABB& other, double epsilon = 0.0) const {
        return min.x <= other.max.x + epsilon && max.x + epsilon >= other.min.x &&
               min.y <= other.max.y + epsilon && max.y + epsilon >= other.min.y &&
               min.z <= other.max.z + epsilon && max.z + epsilon >= other.min.z;
    }

    Vec3 center() const {
        return (min + max) * 0.5;
    }

    Vec3 halfSize() const {
        return (max - min) * 0.5;
    }
};

struct Triangle {
    Vec3 a;
    Vec3 b;
    Vec3 c;
};

inline AABB boundsOf(const Triangle& triangle) {
    AABB box = AABB::empty();
    box.expand(triangle.a);
    box.expand(triangle.b);
    box.expand(triangle.c);
    return box;
}

struct MatrixCell {
    int id{0};
    std::vector<Vec3> vertices;
    std::vector<std::vector<std::size_t>> faces;
    AABB bounds{AABB::empty()};

    void updateBounds() {
        bounds = AABB::empty();
        for (const Vec3& vertex : vertices) {
            bounds.expand(vertex);
        }
    }
};

struct FractureFacet {
    int id{0};
    std::vector<Vec3> vertices;
};

struct IntersectionPair {
    int matrixCellId{0};
    int fractureFacetId{0};
};

} // namespace vsfdop
