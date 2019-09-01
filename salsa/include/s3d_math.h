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


namespace s3d {

constexpr float     kMaxBound = std::numeric_limits<float>::max();
constexpr float     kMinBound = std::numeric_limits<float>::lowest();
constexpr uint32_t  kInvalid  = std::numeric_limits<uint32_t>::max();

template<typename T> struct Vector2;
template<typename T> struct Vector3;
template<typename T> struct Vector4;
template<typename T> struct Matrix;

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
    return (xx << 2) + (yy << 1) + zz;
}

__forceinline uint64_t Morton3D_64(float x, float y, float z) noexcept
{
    double dx = s3d::Clamp(x * 1048576.0, 0.0, 1048575.0);
    double dy = s3d::Clamp(y * 1048576.0, 0.0, 1048575.0);
    double dz = s3d::Clamp(z * 1048576.0, 0.0, 1048575.0);
    uint64_t xx = ExpandBits((uint64_t)dx);
    uint64_t yy = ExpandBits((uint64_t)dy);
    uint64_t zz = ExpandBits((uint64_t)dz);
    return (xx << 2) + (yy << 1) + zz;
}

///////////////////////////////////////////////////////////////////////////////
// Vector2 structure
///////////////////////////////////////////////////////////////////////////////
template<typename T>
struct Vector2
{
    T x;
    T y;
 
    Vector2() noexcept = default;

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

    static Vector2<T> Transform(const Vector2<T>& lhs, const Matrix<T>& rhs) noexcept;
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
 
    Vector3() noexcept = default;

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

    static Vector3<T> Transform      (const Vector3<T>& lhs, const Matrix<T>& rhs) noexcept;
    static Vector3<T> TransformNormal(const Vector3<T>& lhs, const Matrix<T>& rhs) noexcept;
    static Vector3<T> TransformCoord (const Vector3<T>& lhs, const Matrix<T>& rhs) noexcept;
};

///////////////////////////////////////////////////////////////////////////////
// Vector4 structure
///////////////////////////////////////////////////////////////////////////////
template<typename T>
struct Vector4
{
    T x;
    T y;
    T z;
    T w;
 
    Vector4() noexcept = default;

    __forceinline explicit Vector4(T nx, T ny, T nz, T nw) noexcept
    : x(nx), y(ny), z(nz), w(nw)
    { /* DO_NOTHING */ }

    __forceinline Vector4<T> operator + (const Vector4<T>& value) const noexcept
    { return Vector4<T>(x+value.x, y+value.y, z+value.z, w+value.w); }

    __forceinline Vector4<T> operator - (const Vector4<T>& value) const noexcept
    { return Vector4<T>(x-value.x, y-value.y, z-value.z, w-value.w); }

    __forceinline Vector4<T> operator / (const Vector4<T>& value) const noexcept
    { return Vector4<T>(x/value.x, y/value.y, z/value.z, w/value.w); }

    __forceinline Vector4<T> operator * (const Vector4<T>& value) const noexcept
    { return Vector4<T>(x*value.x, y*value.y, z*value.z, w*value.w); }

    __forceinline Vector4<T> operator - () const noexcept
    { return Vector4<T>(-x, -y, -z, -w); }

    __forceinline Vector4<T> operator * (T value) const noexcept
    { return Vector4<T>(x*value, y*value, z*value, w*value); }

    __forceinline Vector4<T> operator / (T value) const noexcept
    { return Vector4<T>(x/value, y/value, z/value, w/value); }

    __forceinline Vector4<T>& operator += (const Vector4<T>& value) noexcept
    {
        x += value.x;
        y += value.y;
        z += value.z;
        w += value.w;
        return *this;
    }

    __forceinline Vector4<T>& operator -= (const Vector4<T>& value) noexcept
    {
        x -= value.x;
        y -= value.y;
        z -= value.z;
        w -= value.w;
        return *this;
    }

    __forceinline Vector4<T>& operator *= (T value) noexcept
    {
        x *= value;
        y *= value;
        z *= value;
        w *= value;
        return *this;
    }

    __forceinline Vector4<T>& operator /= (T value) noexcept
    {
        x /= value;
        y /= value;
        z /= value;
        w /= value;
        return *this;
    }

    __forceinline const T& operator[] (int index) const noexcept
    { return *(&x + index); }

    __forceinline static T Dot(const Vector3<T>& lhs, const Vector3<T>& rhs) noexcept
    { return lhs.x*rhs.x + lhs.y*rhs.y + lhs.z*rhs.z + lhs.w*rhs.w; }

    __forceinline static T Length(const Vector4<T>& value) noexcept
    { return sqrt(Dot(value, value)); }

    __forceinline static Vector4<T> Normalize(const Vector4<T>& value) noexcept
    {
        const auto mag = Dot(value, value);
        const auto inv = (mag > T(0.0)) ? T(1.0/sqrt(mag)) : T(1.0);
        return Vector4<T>(value.x*inv, value.y*inv, value.z*inv, value.w*inv);
    }

    __forceinline static Vector4<T> Min(const Vector4<T>& lhs, const Vector4<T>& rhs) noexcept
    {
        return Vector4<T>(
            s3d::Min(lhs.x, rhs.x),
            s3d::Min(lhs.y, rhs.y),
            s3d::Min(lhs.z, rhs.z),
            s3d::Min(lhs.w, rhs.w));
    }

    __forceinline static Vector4<T> Max(const Vector4<T>& lhs, const Vector4<T>& rhs) noexcept
    {
        return Vector4<T>(
            s3d::Max(lhs.x, rhs.x),
            s3d::Max(lhs.y, rhs.y),
            s3d::Max(lhs.z, rhs.z),
            s3d::Max(lhs.w, rhs.w));
    }

    static Vector4<T> Transform(const Vector4<T>& lhs, const Matrix<T>& rhs) noexcept;
};

///////////////////////////////////////////////////////////////////////////////
// Matrix structure
///////////////////////////////////////////////////////////////////////////////
template<typename T>
struct Matrix
{
    union 
    {
        struct 
        {
            T _11, _12, _13, _14;
            T _21, _22, _23, _24;
            T _31, _32, _33, _34;
            T _41, _42, _43, _44;
        };
        Vector4<T> row[4];
        T          v[16];
    };

    Matrix() noexcept = default;

    __forceinline Matrix
    (
        T m11, T m12, T m13, T m14,
        T m21, T m22, T m23, T m24,
        T m31, T m32, T m33, T m34,
        T m41, T m42, T m43, T m44
    ) noexcept
    : _11(m11), _12(m12), _13(m13), _14(m14)
    , _21(m21), _22(m22), _23(m23), _24(m24)
    , _31(m31), _32(m32), _33(m33), _34(m34)
    , _41(m41), _42(m42), _43(m43), _44(m44)
    { /* DO_NOTHING */ }

    __forceinline Matrix
    (
        const Vector4<T>& r0,
        const Vector4<T>& r1,
        const Vector4<T>& r2,
        const Vector4<T>& r3
    )
    {
        row[0] = r0;
        row[1] = r1;
        row[2] = r2;
        row[3] = r3;
    }

    __forceinline Matrix operator - () const noexcept
    {
        return Matrix<T>(
            -_11, -_12, -_13, -_14,
            -_21, -_22, -_23, -_24,
            -_31, -_32, -_33, -_34,
            -_41, -_42, -_43, -_44);
    }

    __forceinline Matrix<T> operator * (const Matrix<T>& value) const noexcept
    {
        auto r0 = Vector4<T>::Transform( row[0], value );
        auto r1 = Vector4<T>::Transform( row[1], value );
        auto r2 = Vector4<T>::Transform( row[2], value );
        auto r3 = Vector4<T>::Transform( row[3], value );
        return Matrix( r0, r1, r2, r3 );
    }

    __forceinline Matrix<T> operator * (float value) const noexcept
    {
        return Matrix<T>(
            _11 * value, _12 * value, _13 * value, _14 * value,
            _21 * value, _22 * value, _23 * value, _24 * value,
            _31 * value, _32 * value, _33 * value, _34 * value,
            _41 * value, _42 * value, _43 * value, _44 * value);
    }

    __forceinline T Det() const noexcept
    {
        return (
            ( _11 * _22 * _33 * _44 ) + ( _11 * _23 * _34 * _42 ) +
            ( _11 * _24 * _32 * _43 ) + ( _12 * _21 * _34 * _43 ) +
            ( _12 * _23 * _31 * _44 ) + ( _12 * _24 * _33 * _41 ) +
            ( _13 * _21 * _32 * _44 ) + ( _13 * _22 * _34 * _41 ) +
            ( _13 * _24 * _31 * _42 ) + ( _14 * _21 * _33 * _42 ) +
            ( _14 * _22 * _31 * _43 ) + ( _14 * _23 * _32 * _41 ) -
            ( _11 * _22 * _34 * _43 ) - ( _11 * _23 * _32 * _44 ) -
            ( _11 * _24 * _33 * _42 ) - ( _12 * _21 * _33 * _44 ) -
            ( _12 * _23 * _34 * _41 ) - ( _12 * _24 * _31 * _43 ) -
            ( _13 * _21 * _34 * _42 ) - ( _13 * _22 * _31 * _44 ) -
            ( _13 * _24 * _32 * _41 ) - ( _14 * _21 * _32 * _43 ) -
            ( _14 * _22 * _33 * _41 ) - ( _14 * _23 * _31 * _42 )
        );
    }

    inline static Matrix<T> Invert(const Matrix<T>& value)
    {
        Matrix<T> result;
        auto det = value.Det();

        result._11 = ( value._22 * value._33 * value._44 ) + ( value._23 * value._34 * value._42 ) + ( value._24 * value._32 * value._43 )
                   - ( value._22 * value._34 * value._43 ) - ( value._23 * value._32 * value._44 ) - ( value._24 * value._33 * value._42 ) / det;
        result._12 = ( value._12 * value._34 * value._43 ) + ( value._13 * value._32 * value._44 ) + ( value._14 * value._33 * value._42 )
                   - ( value._12 * value._33 * value._44 ) - ( value._13 * value._34 * value._42 ) - ( value._14 * value._32 * value._43 ) / det;
        result._13 = ( value._12 * value._23 * value._44 ) + ( value._13 * value._24 * value._42 ) + ( value._14 * value._22 * value._43 )
                   - ( value._12 * value._24 * value._43 ) - ( value._13 * value._22 * value._44 ) - ( value._14 * value._23 * value._42 ) / det;
        result._14 = ( value._12 * value._24 * value._33 ) + ( value._13 * value._22 * value._34 ) + ( value._14 * value._23 * value._32 )
                   - ( value._12 * value._23 * value._34 ) - ( value._13 * value._24 * value._32 ) - ( value._14 * value._22 * value._33 ) / det;

        result._21 = ( value._21 * value._34 * value._43 ) + ( value._23 * value._31 * value._44 ) + ( value._24 * value._33 * value._41 )
                   - ( value._21 * value._33 * value._44 ) - ( value._23 * value._34 * value._41 ) - ( value._24 * value._31 * value._43 ) / det;
        result._22 = ( value._11 * value._33 * value._44 ) + ( value._13 * value._34 * value._41 ) + ( value._14 * value._31 * value._43 )
                   - ( value._11 * value._34 * value._43 ) - ( value._13 * value._31 * value._44 ) - ( value._14 * value._33 * value._41 ) / det;
        result._23 = ( value._11 * value._24 * value._43 ) + ( value._13 * value._21 * value._44 ) + ( value._14 * value._23 * value._41 )
                   - ( value._11 * value._23 * value._44 ) - ( value._13 * value._24 * value._41 ) - ( value._14 * value._21 * value._43 ) / det;
        result._24 = ( value._11 * value._23 * value._34 ) + ( value._13 * value._24 * value._31 ) + ( value._14 * value._21 * value._33 )
                   - ( value._11 * value._24 * value._33 ) - ( value._13 * value._21 * value._34 ) - ( value._14 * value._23 * value._31 ) / det;

        result._31 = ( value._21 * value._32 * value._44 ) + ( value._22 * value._34 * value._41 ) + ( value._24 * value._31 * value._42 )
                   - ( value._21 * value._34 * value._42 ) - ( value._22 * value._31 * value._44 ) - ( value._24 * value._32 * value._41 ) / det;
        result._32 = ( value._11 * value._34 * value._42 ) + ( value._12 * value._31 * value._44 ) + ( value._14 * value._32 * value._41 )
                   - ( value._11 * value._32 * value._44 ) - ( value._12 * value._34 * value._41 ) - ( value._14 * value._31 * value._42 ) / det;
        result._33 = ( value._11 * value._22 * value._44 ) + ( value._12 * value._24 * value._41 ) + ( value._14 * value._21 * value._42 )
                   - ( value._11 * value._24 * value._42 ) - ( value._12 * value._21 * value._44 ) - ( value._14 * value._22 * value._41 ) / det;
        result._34 = ( value._11 * value._24 * value._32 ) + ( value._12 * value._21 * value._34 ) + ( value._14 * value._22 * value._31 )
                   - ( value._11 * value._22 * value._34 ) - ( value._12 * value._24 * value._31 ) - ( value._14 * value._21 * value._32 ) / det;

        result._41 = ( value._21 * value._33 * value._42 ) + ( value._22 * value._31 * value._43 ) + ( value._23 * value._32 * value._41 )
                   - ( value._21 * value._32 * value._43 ) - ( value._22 * value._33 * value._41 ) - ( value._23 * value._31 * value._42 ) / det;
        result._42 = ( value._11 * value._32 * value._43 ) + ( value._12 * value._33 * value._41 ) + ( value._13 * value._31 * value._42 )
                   - ( value._11 * value._33 * value._42 ) - ( value._12 * value._31 * value._43 ) - ( value._13 * value._32 * value._41 ) / det;
        result._43 = ( value._11 * value._23 * value._42 ) + ( value._12 * value._21 * value._43 ) + ( value._13 * value._22 * value._41 )
                   - ( value._11 * value._22 * value._43 ) - ( value._12 * value._23 * value._41 ) - ( value._13 * value._21 * value._42 ) / det;
        result._44 = ( value._11 * value._22 * value._33 ) + ( value._12 * value._23 * value._31 ) + ( value._13 * value._21 * value._32 )
                   - ( value._11 * value._23 * value._32 ) - ( value._12 * value._21 * value._33 ) - ( value._13 * value._22 * value._31 ) / det;

        return result;
    }

    __forceinline static Matrix<T> Transpose(const Matrix<T>& value) noexcept
    {
        return Matrix<T>(
            value._11, value._21, value._31, value._41,
            value._12, value._22, value._32, value._42,
            value._13, value._23, value._33, value._43,
            value._14, value._24, value._34, value._44);
    }

    __forceinline static Matrix<T> CreateIdentity() noexcept
    {
        return Matrix<T>(
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1);
    }
};

template<typename T>
__forceinline T Max2(const Vector2<T>& value) noexcept
{ return s3d::Max(value.x, value.y); }

template<typename T>
__forceinline T Min2(const Vector2<T>& value) noexcept
{ return s3d::Min(value.x, value.y); }

template<typename T>
__forceinline T Max3(const Vector3<T>& value) noexcept
{ return s3d::Max( s3d::Max(value.x, value.y), value.z ); }

template<typename T>
__forceinline T Min3(const Vector3<T>& value) noexcept
{ return s3d::Min( s3d::Min(value.x, value.y), value.z ); }

template<typename T>
__forceinline T Max4(const Vector4<T>& value) noexcept
{ return s3d::Max( s3d::Max(value.x, value.y), s3d::Max(value.z, value.w) ); }

template<typename T>
__forceinline T Min4(const Vector4<T>& value) noexcept
{ return s3d::Min( s3d::Min(value.x, value.y), s3d::Min(value.z, value.w) ); }

template<typename T>
__forceinline Vector2<T> Vector2<T>::Transform(const Vector2<T>& lhs, const Matrix<T>& rhs) noexcept
{
    return Vector2<T>(
        ( lhs.x * rhs._11 ) + ( lhs.y * rhs._21 ) + rhs._41,
        ( lhs.x * rhs._12 ) + ( lhs.y * rhs._22 ) + rhs._42);
}

template<typename T>
__forceinline Vector3<T> Vector3<T>::Transform(const Vector3<T>& lhs, const Matrix<T>& rhs) noexcept
{
    return Vector3<T>(
        ( lhs.x * rhs._11 ) + ( lhs.y * rhs._21 ) + ( lhs.z * rhs._31 ) + rhs._41,
        ( lhs.x * rhs._12 ) + ( lhs.y * rhs._22 ) + ( lhs.z * rhs._32 ) + rhs._42,
        ( lhs.x * rhs._13 ) + ( lhs.y * rhs._23 ) + ( lhs.z * rhs._33 ) + rhs._43
    );
}

template<typename T>
__forceinline Vector3<T> Vector3<T>::TransformNormal(const Vector3<T>& lhs, const Matrix<T>& rhs) noexcept
{
    return Vector3(
        ( lhs.x * rhs._11 ) + ( lhs.y * rhs._21 ) + ( lhs.z * rhs._31 ),
        ( lhs.x * rhs._12 ) + ( lhs.y * rhs._22 ) + ( lhs.z * rhs._32 ),
        ( lhs.x * rhs._13 ) + ( lhs.y * rhs._23 ) + ( lhs.z * rhs._33 ));
}

template<typename T>
__forceinline Vector3<T> Vector3<T>::TransformCoord(const Vector3<T>& lhs, const Matrix<T>& rhs) noexcept
{
    auto X = ( lhs.x * rhs._11 ) + ( lhs.y * rhs._21 ) + ( lhs.z * rhs._31 ) + rhs._41;
    auto Y = ( lhs.x * rhs._12 ) + ( lhs.y * rhs._22 ) + ( lhs.z * rhs._32 ) + rhs._42;
    auto Z = ( lhs.x * rhs._13 ) + ( lhs.y * rhs._23 ) + ( lhs.z * rhs._33 ) + rhs._43;
    auto W = ( lhs.x * rhs._14 ) + ( lhs.y * rhs._24 ) + ( lhs.z * rhs._34 ) + rhs._44;
    auto inv = T(1) / W;
    return Vector3<T>(X * inv, Y * inv, Z * inv);
}

template<typename T>
__forceinline Vector4<T> Vector4<T>::Transform(const Vector4<T>& lhs, const Matrix<T>& rhs) noexcept
{
    return Vector4<T>(
        ( lhs.x * rhs._11 ) + ( lhs.y * rhs._21 ) + ( lhs.z * rhs._31 ) + ( lhs.w * rhs._41 ),
        ( lhs.x * rhs._12 ) + ( lhs.y * rhs._22 ) + ( lhs.z * rhs._32 ) + ( lhs.w * rhs._42 ),
        ( lhs.x * rhs._13 ) + ( lhs.y * rhs._23 ) + ( lhs.z * rhs._33 ) + ( lhs.w * rhs._43 ),
        ( lhs.x * rhs._14 ) + ( lhs.y * rhs._24 ) + ( lhs.z * rhs._34 ) + ( lhs.w * rhs._44 ));
}


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

using Vector4i  = Vector4<int>;
using Vector4u  = Vector4<uint32_t>;
using Vector4li = Vector4<int64_t>;
using Vector4lu = Vector4<uint64_t>;
using Vector4f  = Vector4<float>;
using Vector4d  = Vector4<double>;


///////////////////////////////////////////////////////////////////////////////
// AABB structure
///////////////////////////////////////////////////////////////////////////////
struct AABB
{
    Vector3f mini;
    Vector3f maxi;

    AABB() noexcept = default;

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
        result.mini.x = s3d::Min<float>(result.mini.x, position[i * 3 + 0]);
        result.mini.y = s3d::Min<float>(result.mini.y, position[i * 3 + 1]);
        result.mini.z = s3d::Min<float>(result.mini.z, position[i * 3 + 2]);

        result.maxi.x = s3d::Max<float>(result.maxi.x, position[i * 3 + 0]);
        result.maxi.y = s3d::Max<float>(result.maxi.y, position[i * 3 + 1]);
        result.maxi.z = s3d::Max<float>(result.maxi.z, position[i * 3 + 2]);
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
    const auto e1  = v1 - v0;
    const auto e2  = v2 - v0;
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

__forceinline void TangentSpace(const Vector3f& N, Vector3f& T, Vector3f& B)
{
    // Tom Duff, James Burgess, Per Christensen, Christophe Hery, Andrew Kensler, Max Liani, and Ryusuke Villemin
    // "Building an Orthonormal Bais, Revisited",
    // Journal of Computer Graphics Techniques Vol.6, No.1, 2017.
    // Listing 3.参照.
    auto s = (N.z >= 0.0f) ? 1.0f : -1.0f;
    auto a = -1.0f / (s + N.z);
    auto b = N.x * N.y * a;
    T = Vector3f(1.0f + s * N.x * N.x * a, s * b, -s * N.x);
    B = Vector3f(b, s + N.y * N.y * a, -N.y);
}

///////////////////////////////////////////////////////////////////////////////
// PCG class
///////////////////////////////////////////////////////////////////////////////
class PCG
{
public:
    PCG() noexcept
    { SetSeed( 123456789 ); }

    PCG(uint64_t seed) noexcept
    { SetSeed(seed); }

    PCG(const PCG& value) noexcept
    : m_State(value.m_State)
    { /* DO_NOTHING */ }

    __forceinline void SetSeed(uint64_t seed) noexcept
    {
        m_State = seed + kIncrement;
        GetAsU32();
    }

    __forceinline uint32_t GetAsU32() noexcept
    {
        auto old_state = m_State;
        m_State = old_state * kMultiplier + kIncrement;
        auto xorshifted = uint32_t(((old_state >> 18) ^ old_state) >> 27);
        auto rot        = uint32_t(old_state >> 59);
        return uint32_t((xorshifted >> rot) | (xorshifted << ((~rot +1) & 31)));
    }

    __forceinline float GetAsF32() noexcept
    { return static_cast<float>( GetAsU32() ) / 0xffffffffui32; }

    __forceinline PCG& operator = (const PCG& value) noexcept
    {
        m_State = value.m_State;
        return *this;
    }

private:
    static const uint64_t   kMultiplier = 6364136223846793005u;
    static const uint64_t   kIncrement  = 1442695040888963407u;
    uint64_t                m_State     = 0x4d595df4d0f33173;
};


} // namespace s3d
