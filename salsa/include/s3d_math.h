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

    __forceinline Vector3(float nx, float ny, float nz) noexcept
    : x(nx), y(ny), z(nz)
    { /* DO_NOTHING */ }

    __forceinline Vector3 operator + (const Vector3& value) const noexcept
    { return Vector3(x+value.x, y+value.y, z+value.z); }

    __forceinline Vector3 operator - (const Vector3& value) const noexcept
    { return Vector3(x-value.x, y-value.y, z-value.z); }

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

///////////////////////////////////////////////////////////////////////////////
// RayBase structure
// ※レイトレ徒競走側と共通部分.
///////////////////////////////////////////////////////////////////////////////
struct RayBase
{
    Vector3 ray_pos;        //!< レイの原点.
    Vector3 ray_dir;        //!< レイの方向ベクトル.
    float   t_near;         //!< 交差判定用最小値.
    float   t_far;          //!< 交差判定用最大値.
    bool    intersect;      //!< 交差判定ありなら true.
    Vector3 hit_pos;        //!< 交差した位置座標.
    Vector3 normal;         //!< シェーディング用の法線.
    int32_t face_id;        //!< 交差した面番号.
};

///////////////////////////////////////////////////////////////////////////////
// Ray structure
// ※個人利用のデータ.
///////////////////////////////////////////////////////////////////////////////
struct Ray : public RayBase
{
    Vector3 inv_dir;        //!< レイの方向ベクトルの逆数.
};




} // namespace s3d
