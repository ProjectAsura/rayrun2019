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
#include <cfloat>

#include <immintrin.h>



namespace s3d {

__forceinline float Min(float lhs, float rhs) noexcept
{ return (lhs < rhs) ? lhs : rhs; }

__forceinline float Max(float lhs, float rhs) noexcept
{ return (lhs > rhs) ? lhs : rhs; }

///////////////////////////////////////////////////////////////////////////////
// Vector3 structure
///////////////////////////////////////////////////////////////////////////////
struct Vector3
{
    float x;
    float y;
    float z;
 
    __forceinline Vector3() noexcept
    { /* DO_NOTHING */ }

    __forceinline explicit Vector3(float nx, float ny, float nz) noexcept
    : x(nx), y(ny), z(nz)
    { /* DO_NOTHING */ }

    __forceinline Vector3 operator + (const Vector3& value) const noexcept
    { return Vector3(x+value.x, y+value.y, z+value.z); }

    __forceinline Vector3 operator - (const Vector3& value) const noexcept
    { return Vector3(x-value.x, y-value.y, z-value.z); }

    __forceinline Vector3 operator * (const Vector3& value) const noexcept
    { return Vector3(x*value.x, y*value.y, z*value.z); }

    __forceinline Vector3 operator - () const noexcept
    { return Vector3(-x, -y, -z); }

    __forceinline Vector3 operator * (float value) const noexcept
    { return Vector3(x*value, y*value, z*value); }

    __forceinline const float& operator[] (int index) const noexcept
    { return *(&x + index); }

    __forceinline static float Dot(const Vector3& lhs, const Vector3& rhs) noexcept
    { return lhs.x*rhs.x + lhs.y*rhs.y + lhs.z*rhs.z; }

    __forceinline static float Length(const Vector3& value) noexcept
    { return sqrt(Dot(value, value)); }

    __forceinline static Vector3 Cross(const Vector3& lhs, const Vector3& rhs) noexcept
    {
        return Vector3(
            (lhs.y*rhs.z) - (lhs.z*rhs.y),
            (lhs.z*rhs.x) - (lhs.x*rhs.z),
            (lhs.x*rhs.y) - (lhs.y*rhs.x));
    }

    __forceinline static Vector3 Normalize(const Vector3& value) noexcept
    {
        const auto mag = Dot(value, value);
        const auto inv = (mag > 0.0f) ? 1.0f/sqrt(mag) : 1.0f;
        return Vector3(value.x*inv, value.y*inv, value.z*inv);
    }

    __forceinline static Vector3 Min(const Vector3& lhs, const Vector3& rhs) noexcept
    {
        return Vector3(
            s3d::Min(lhs.x, rhs.x),
            s3d::Min(lhs.y, rhs.y),
            s3d::Min(lhs.z, rhs.z));
    }

    __forceinline static Vector3 Max(const Vector3& lhs, const Vector3& rhs) noexcept
    {
        return Vector3(
            s3d::Max(lhs.x, rhs.x),
            s3d::Max(lhs.y, rhs.y),
            s3d::Max(lhs.z, rhs.z));
    }
};

__forceinline float Max3(const Vector3& value)
{ return s3d::Max( s3d::Max(value.x, value.y), value.z ); }

__forceinline float Min3(const Vector3& value)
{ return s3d::Min( s3d::Min(value.x, value.y), value.z ); }


///////////////////////////////////////////////////////////////////////////////
// AABB structure
///////////////////////////////////////////////////////////////////////////////
struct AABB
{
    Vector3 mini;
    Vector3 maxi;

    __forceinline AABB() noexcept
    { /* DO_NOTHING */ }

    __forceinline explicit AABB(std::nullptr_t) noexcept
    : mini( FLT_MAX,  FLT_MAX,  FLT_MAX)
    , maxi(-FLT_MAX, -FLT_MAX, -FLT_MAX)
    { /* DO_NOTHING */ }

    __forceinline explicit AABB(const Vector3& _min, const Vector3& _max) noexcept
    : mini(_min)
    , maxi(_max)
    { /* DO_NOTHING */ }

    __forceinline explicit AABB(const float* _min, const float* _max) noexcept
    : mini(_min[0], _min[1], _min[2])
    , maxi(_max[0], _max[1], _max[2])
    { /* DO_NOTHING */ }

    __forceinline Vector3 GetCenter() const noexcept
    { return (mini + maxi) * 0.5f; }

    __forceinline void Merge(const AABB& value) noexcept
    {
        mini = Vector3::Min(mini, value.mini);
        maxi = Vector3::Max(maxi, value.maxi);
    }

    __forceinline void Merge(const Vector3& value) noexcept
    {
        mini = Vector3::Min(mini, value);
        maxi = Vector3::Max(maxi, value);
    }

    __forceinline const Vector3& operator[] (int index) const
    { return *(&mini + index); }

    __forceinline bool Slab(const Vector3& rayPos, const Vector3& invRayDir) const noexcept
    {
        const auto t0 = (mini - rayPos) * invRayDir;
        const auto t1 = (maxi - rayPos) * invRayDir;
        const auto tmin = Vector3::Min(t0, t1);
        const auto tmax = Vector3::Max(t0, t1);
        return Max3(tmin) <= Min3(tmax);
    }

    __forceinline void Clear() noexcept
    {
        mini.x = mini.y = mini.z =  FLT_MAX;
        maxi.x = maxi.y = maxi.z = -FLT_MAX;
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

struct Triangle
{
    Vector3 p[3];
    Vector3 n[3];
    AABB    box;

    __forceinline Triangle() noexcept
    { /* DO_NOTHING */ }

    inline Triangle
    (
        const float*    position,
        const float*    normals,
        const uint32_t* indices,
        size_t          currentFace
    ) noexcept
    {
        {
            auto v0 = indices[currentFace + 0];
            auto v1 = indices[currentFace + 2];
            auto v2 = indices[currentFace + 4];

            p[0].x = position[v0 + 0];
            p[0].y = position[v0 + 1];
            p[0].z = position[v0 + 2];

            p[1].x = position[v1 + 0];
            p[1].y = position[v1 + 1];
            p[1].z = position[v1 + 2];

            p[2].x = position[v2 + 0];
            p[2].y = position[v2 + 1];
            p[2].z = position[v2 + 2];
        }

        {
            auto n0 = indices[currentFace + 1];
            auto n1 = indices[currentFace + 3];
            auto n2 = indices[currentFace + 5];

            n[0].x = normals[n0 + 0];
            n[0].y = normals[n0 + 1];
            n[0].z = normals[n0 + 2];

            n[1].x = normals[n1 + 0];
            n[1].y = normals[n1 + 1];
            n[1].z = normals[n1 + 2];

            n[2].x = normals[n2 + 0];
            n[2].y = normals[n2 + 1];
            n[2].z = normals[n2 + 2];
        }
    }
};


} // namespace s3d
