//-----------------------------------------------------------------------------
// File : s3d_math.h
// Desc : Math Library.
// Copyright(c) Project Asura. All right reserved.
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include <cstdint>
#include <cmath>
#include <limits>

#include <immintrin.h>

#ifdef _MSC_VER
#include <intrin.h> // for __lzcnt
#endif



namespace s3d {

constexpr float     kMaxBound = std::numeric_limits<float>::max();
constexpr float     kMinBound = std::numeric_limits<float>::lowest();
constexpr uint32_t  kInvalid  = std::numeric_limits<uint32_t>::max();

template<typename T>
__forceinline T Min(T lhs, T rhs) noexcept
{ return (lhs < rhs) ? lhs : rhs; }

template<typename T>
__forceinline T Max(T lhs, T rhs) noexcept
{ return (lhs > rhs) ? lhs : rhs; }

template<typename T>
__forceinline T Clamp(T value, T mini, T maxi) noexcept
{ return Max( mini, Min( maxi, value ) ); }

template<typename T>
__forceinline T Lerp(T a, T b, T t) noexcept
{ return (b - a) * t + a; }

__forceinline uint32_t ExpandBits(uint32_t v) noexcept
{
    v = (v * 0x00010001u) & 0xFF0000FFu;
    v = (v * 0x00000101u) & 0x0F00F00Fu;
    v = (v * 0x00000011u) & 0xC30C30C3u;
    v = (v * 0x00000005u) & 0x49249249u;
    return v;
}

__forceinline uint64_t ExpandBits(uint64_t v) noexcept
{
    v = (v * 0x000100000001u) & 0xFFFF00000000FFFFu;
    v = (v * 0x000000010001u) & 0x00FF0000FF0000FFu;
    v = (v * 0x000000000101u) & 0xF00F00F00F00F00Fu;
    v = (v * 0x000000000011u) & 0x30C30C30C30C30C3u;
    v = (v * 0x000000000005u) & 0x9249249249249249u;

    return v;
}

__forceinline uint32_t Morton3D(float x, float y, float z) noexcept
{
    x = s3d::Clamp(x * 1024.0f, 0.0f, 1023.0f);
    y = s3d::Clamp(y * 1024.0f, 0.0f, 1023.0f);
    z = s3d::Clamp(z * 1024.0f, 0.0f, 1023.0f);
    uint32_t xx = ExpandBits((uint32_t)x);
    uint32_t yy = ExpandBits((uint32_t)y);
    uint32_t zz = ExpandBits((uint32_t)z);
    return xx * 4 + yy * 2 + zz;
}

__forceinline uint64_t Morton3D_64(float x, float y, float z) noexcept
{
    double dx = s3d::Clamp(x * 1048576.0, 0.0, 1048575.0);
    double dy = s3d::Clamp(y * 1048576.0, 0.0, 1048575.0);
    double dz = s3d::Clamp(z * 1048576.0, 0.0, 1048575.0);
    uint64_t xx = ExpandBits((uint64_t)dx);
    uint64_t yy = ExpandBits((uint64_t)dy);
    uint64_t zz = ExpandBits((uint64_t)dz);
    return xx * 4 + yy * 2 + zz;
}

// count leading zero
__forceinline uint32_t Clz(uint32_t value)
{
#if defined(_MSC_VER)
    return __lzcnt(value);
#elif defined(__GNUC__)
    return __builtin_clz(value);
#else
    uint32_t n = 0;
    if ((value & 0xFFFF0000) == 0) {n  = 16; value <<= 16;}
    if ((value & 0xFF000000) == 0) {n +=  8; value <<=  8;}
    if ((value & 0xF0000000) == 0) {n +=  4; value <<=  4;}
    if ((value & 0xC0000000) == 0) {n +=  2; value <<=  2;}
    if ((value & 0x80000000) == 0) {n +=  1;}
    return n;
#endif
}


///////////////////////////////////////////////////////////////////////////////
// Vector2 structure
///////////////////////////////////////////////////////////////////////////////
template<typename T>
struct Vector2
{
    T x;
    T y;
 
    __forceinline Vector2() noexcept
    { /* DO_NOTHING */ }

    __forceinline explicit Vector2(T nx, T ny) noexcept
    : x(nx), y(ny)
    { /* DO_NOTHING */ }

    __forceinline Vector2<T> operator + (const Vector2<T>& value) const noexcept
    { return Vector2<T>(x+value.x, y+value.y); }

    __forceinline Vector2<T> operator - (const Vector2<T>& value) const noexcept
    { return Vector2<T>(x-value.x, y-value.y); }

    __forceinline Vector2<T> operator / (const Vector2<T>& value) const noexcept
    { return Vector2<T>(x/value.x, y/value.y); }

    __forceinline Vector2<T> operator * (const Vector2<T>& value) const noexcept
    { return Vector2<T>(x*value.x, y*value.y); }

    __forceinline Vector2<T> operator - () const noexcept
    { return Vector2<T>(-x, -y); }

    __forceinline Vector2<T> operator * (T value) const noexcept
    { return Vector2<T>(x*value, y*value); }

    __forceinline Vector2<T>& operator += (const Vector2<T>& value) noexcept
    {
        x += value.x;
        y += value.y;
        return *this;
    }

    __forceinline Vector2<T>& operator -= (const Vector2<T>& value) noexcept
    {
        x -= value.x;
        y -= value.y;
        return *this;
    }

    __forceinline Vector2<T>& operator *= (T value) noexcept
    {
        x *= value;
        y *= value;
        return *this;
    }

    __forceinline Vector2<T>& operator /= (T value) noexcept
    {
        x /= value;
        y /= value;
        return *this;
    }

    __forceinline const T& operator[] (int index) const noexcept
    { return *(&x + index); }

    __forceinline static T Dot(const Vector2<T>& lhs, const Vector2<T>& rhs) noexcept
    { return lhs.x*rhs.x + lhs.y*rhs.y; }

    __forceinline static T Length(const Vector2<T>& value) noexcept
    { return sqrt(Dot(value, value)); }

    __forceinline static Vector2<T> Normalize(const Vector2<T>& value) noexcept
    {
        const auto mag = Dot(value, value);
        const auto inv = (mag > 0.0) ? 1.0/sqrt(mag) : 1.0;
        return Vector2<T>(value.x*inv, value.y*inv);
    }

    __forceinline static Vector2<T> Min(const Vector2<T>& lhs, const Vector2<T>& rhs) noexcept
    {
        return Vector2<T>(
            s3d::Min(lhs.x, rhs.x),
            s3d::Min(lhs.y, rhs.y));
    }

    __forceinline static Vector2<T> Max(const Vector2<T>& lhs, const Vector2<T>& rhs) noexcept
    {
        return Vector2<T>(
            s3d::Max(lhs.x, rhs.x),
            s3d::Max(lhs.y, rhs.y));
    }
};


///////////////////////////////////////////////////////////////////////////////
// Vector3 structure
///////////////////////////////////////////////////////////////////////////////
template<typename T>
struct Vector3
{
    T x;
    T y;
    T z;
 
    __forceinline Vector3() noexcept
    { /* DO_NOTHING */ }

    __forceinline explicit Vector3(T nx, T ny, T nz) noexcept
    : x(nx), y(ny), z(nz)
    { /* DO_NOTHING */ }

    __forceinline Vector3<T> operator + (const Vector3<T>& value) const noexcept
    { return Vector3<T>(x+value.x, y+value.y, z+value.z); }

    __forceinline Vector3<T> operator - (const Vector3<T>& value) const noexcept
    { return Vector3<T>(x-value.x, y-value.y, z-value.z); }

    __forceinline Vector3<T> operator / (const Vector3<T>& value) const noexcept
    { return Vector3<T>(x/value.x, y/value.y, z/value.z); }

    __forceinline Vector3<T> operator * (const Vector3<T>& value) const noexcept
    { return Vector3<T>(x*value.x, y*value.y, z*value.z); }

    __forceinline Vector3<T> operator - () const noexcept
    { return Vector3<T>(-x, -y, -z); }

    __forceinline Vector3<T> operator * (T value) const noexcept
    { return Vector3<T>(x*value, y*value, z*value); }

    __forceinline Vector3<T> operator / (T value) const noexcept
    { return Vector3<T>(x/value, y/value, z/value); }

    __forceinline Vector3<T>& operator += (const Vector3<T>& value) noexcept
    {
        x += value.x;
        y += value.y;
        z += value.z;
        return *this;
    }

    __forceinline Vector3<T>& operator -= (const Vector3<T>& value) noexcept
    {
        x -= value.x;
        y -= value.y;
        z -= value.z;
        return *this;
    }

    __forceinline Vector3<T>& operator *= (T value) noexcept
    {
        x *= value;
        y *= value;
        z *= value;
        return *this;
    }

    __forceinline Vector3<T>& operator /= (T value) noexcept
    {
        x /= value;
        y /= value;
        z /= value;
        return *this;
    }

    __forceinline const T& operator[] (int index) const noexcept
    { return *(&x + index); }

    __forceinline static T Dot(const Vector3<T>& lhs, const Vector3<T>& rhs) noexcept
    { return lhs.x*rhs.x + lhs.y*rhs.y + lhs.z*rhs.z; }

    __forceinline static T Length(const Vector3<T>& value) noexcept
    { return sqrt(Dot(value, value)); }

    __forceinline static Vector3<T> Cross(const Vector3<T>& lhs, const Vector3<T>& rhs) noexcept
    {
        return Vector3<T>(
            (lhs.y*rhs.z) - (lhs.z*rhs.y),
            (lhs.z*rhs.x) - (lhs.x*rhs.z),
            (lhs.x*rhs.y) - (lhs.y*rhs.x));
    }

    __forceinline static Vector3<T> Normalize(const Vector3<T>& value) noexcept
    {
        const auto mag = Dot(value, value);
        const auto inv = (mag > T(0.0)) ? T(1.0/sqrt(mag)) : T(1.0);
        return Vector3<T>(value.x*inv, value.y*inv, value.z*inv);
    }

    __forceinline static Vector3<T> Min(const Vector3<T>& lhs, const Vector3<T>& rhs) noexcept
    {
        return Vector3<T>(
            s3d::Min(lhs.x, rhs.x),
            s3d::Min(lhs.y, rhs.y),
            s3d::Min(lhs.z, rhs.z));
    }

    __forceinline static Vector3<T> Max(const Vector3<T>& lhs, const Vector3<T>& rhs) noexcept
    {
        return Vector3<T>(
            s3d::Max(lhs.x, rhs.x),
            s3d::Max(lhs.y, rhs.y),
            s3d::Max(lhs.z, rhs.z));
    }
};

template<typename T>
__forceinline T Max2(const Vector2<T>& value)
{ return s3d::Max(value.x, value.y); }

template<typename T>
__forceinline T Min2(const Vector2<T>& value)
{ return s3d::Min(value.x, value.y); }

template<typename T>
__forceinline T Max3(const Vector3<T>& value)
{ return s3d::Max( s3d::Max(value.x, value.y), value.z ); }

template<typename T>
__forceinline T Min3(const Vector3<T>& value)
{ return s3d::Min( s3d::Min(value.x, value.y), value.z ); }

using Vector2i  = Vector2<int>;
using Vector2u  = Vector2<uint32_t>;
using Vector2li = Vector2<int64_t>;
using Vector2lu = Vector2<uint64_t>;
using Vector2f  = Vector2<float>;
using Vector2d  = Vector2<double>;

using Vector3i  = Vector3<int>;
using Vector3u  = Vector3<uint32_t>;
using Vector3li = Vector3<int64_t>;
using Vector3lu = Vector3<uint64_t>;
using Vector3f  = Vector3<float>;
using Vector3d  = Vector3<double>;


///////////////////////////////////////////////////////////////////////////////
// AABB structure
///////////////////////////////////////////////////////////////////////////////
struct AABB
{
    Vector3f mini;
    Vector3f maxi;

    __forceinline AABB() noexcept
    { /* DO_NOTHING */ }

    __forceinline explicit AABB(std::nullptr_t) noexcept
    : mini(kMaxBound, kMaxBound, kMaxBound)
    , maxi(kMinBound, kMinBound, kMinBound)
    { /* DO_NOTHING */ }

    __forceinline explicit AABB(const Vector3f& _min, const Vector3f& _max) noexcept
    : mini(_min)
    , maxi(_max)
    { /* DO_NOTHING */ }

    __forceinline explicit AABB(const Vector3f& value) noexcept
    : mini(value)
    , maxi(value)
    { /* DO_NOTHING */ }

    __forceinline explicit AABB(const float* _min, const float* _max) noexcept
    : mini(_min[0], _min[1], _min[2])
    , maxi(_max[0], _max[1], _max[2])
    { /* DO_NOTHING */ }

    __forceinline AABB(const AABB& value) noexcept
    : mini(value.mini)
    , maxi(value.maxi)
    { /* DO_NOTHING */ }

    __forceinline Vector3f GetCenter() const noexcept
    { return (mini + maxi) * 0.5f; }

    __forceinline void Merge(const AABB& value) noexcept
    {
        mini = Vector3f::Min(mini, value.mini);
        maxi = Vector3f::Max(maxi, value.maxi);
    }

    __forceinline void Merge(const Vector3f& value) noexcept
    {
        mini = Vector3f::Min(mini, value);
        maxi = Vector3f::Max(maxi, value);
    }

    __forceinline Vector3f Normalize(const Vector3f& p) const noexcept
    { return (p - mini) / (maxi - mini); }

    __forceinline const Vector3f& operator[] (int index) const
    { return *(&mini + index); }

    __forceinline bool Slab(const Vector3f& rayPos, const Vector3f& invRayDir) const noexcept
    {
        const auto t0 = (mini - rayPos) * invRayDir;
        const auto t1 = (maxi - rayPos) * invRayDir;
        const auto tmin = Vector3f::Min(t0, t1);
        const auto tmax = Vector3f::Max(t0, t1);
        return Max3(tmin) <= Min3(tmax);
    }

    __forceinline bool Intersect(const Vector3f& rayPos, const Vector3f& invRayDir, const float length) const noexcept
    {
        Vector3f v;
        v.x = ((0 < invRayDir.x ? mini.x : maxi.x) - rayPos.x) * invRayDir.x;
        v.y = ((0 < invRayDir.y ? mini.y : maxi.y) - rayPos.y) * invRayDir.y;
        v.z = ((0 < invRayDir.z ? mini.z : maxi.z) - rayPos.z) * invRayDir.z;

        const auto tmin = Max3(v);

        v.x = ((0 < invRayDir.x ? maxi.x : mini.x) - rayPos.x) * invRayDir.x;
        v.y = ((0 < invRayDir.y ? maxi.y : mini.y) - rayPos.y) * invRayDir.y;
        v.z = ((0 < invRayDir.z ? maxi.z : mini.z) - rayPos.z) * invRayDir.z;

        const auto tmax = Min3(v);
        return (tmin <= tmax) && (0.0f < tmax) && (tmin < length);
    }

    __forceinline void Clear() noexcept
    {
        mini.x = mini.y = mini.z = kMaxBound;
        maxi.x = maxi.y = maxi.z = kMinBound;
    }
};

__forceinline AABB MakeBox(const float* position, size_t count) noexcept 
{
    if (count == 0 || position == nullptr)
    { return AABB(nullptr); }

    AABB result;

    result.mini.x = result.maxi.x = position[0];
    result.mini.y = result.maxi.y = position[1];
    result.mini.z = result.maxi.z = position[2];

    for(size_t i=1; i<count; ++i)
    {
        result.mini.x = s3d::Min(result.mini.x, position[i * 3 + 0]);
        result.mini.y = s3d::Min(result.mini.y, position[i * 3 + 1]);
        result.mini.z = s3d::Min(result.mini.z, position[i * 3 + 2]);

        result.maxi.x = s3d::Max(result.maxi.x, position[i * 3 + 0]);
        result.maxi.y = s3d::Max(result.maxi.y, position[i * 3 + 1]);
        result.maxi.z = s3d::Max(result.maxi.z, position[i * 3 + 2]);
    }

    return result;
}

__forceinline bool IntersectTriangle
(
    const Vector3f& rayPos,
    const Vector3f& rayDir,
    const Vector3f& v0,
    const Vector3f& v1,
    const Vector3f& v2,
    const float     tmin,
    const float     tmax,
    float&          dist,
    float&          u,
    float&          v
)
{
    auto e1  = v1 - v0;
    auto e2  = v2 - v0;
    auto P   = Vector3f::Cross(rayDir, e2);
    auto det = Vector3f::Dot(e1, P);
    if (det == 0.0f)
    { return false; }

    auto inv_det = 1.0f / det;
    auto T  = rayPos - v0;
    auto fu = Vector3f::Dot(T, P) * inv_det;
    if (fu < 0.0f || fu > 1.0f)
    { return false; }

    auto Q  = Vector3f::Cross(T, e1);
    auto fv = Vector3f::Dot(rayDir, Q) * inv_det;
    if (fv < 0.0f || (fu + fv) > 1.0f)
    { return false; }

    auto t = Vector3f::Dot(e2, Q) * inv_det;
    if (t < tmin || tmax <= t || t > dist)
    { return false; }

    dist = t;
    u    = fu;
    v    = fv;
    return true;
}


} // namespace s3d
