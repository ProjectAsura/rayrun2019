//-----------------------------------------------------------------------------
// File : s3d_bvh.h
// Desc : Bounding Volume Hierarchy.
// Copyright(c) Project Asura. All right reserved.
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include <s3d_math.h>
#include <vector>


namespace s3d {

///////////////////////////////////////////////////////////////////////////////
// Node structure
///////////////////////////////////////////////////////////////////////////////
struct Node
{
    AABB        Box;    //!< バウンディングボックス.
    uint32_t    L;      //!< 子ノード左. (末尾が0x1なら葉ノード). 
    uint32_t    R;      //!< 子ノード右. (末尾が0x1なら葉ノード).

    __forceinline Node() noexcept
    : Box(nullptr)
    , L  (kInvalid)
    , R  (kInvalid)
    { /* DO_NOTHING */ }
};

///////////////////////////////////////////////////////////////////////////////
// VertexIndex structure
///////////////////////////////////////////////////////////////////////////////
struct alignas(8) VertexIndex
{
    uint32_t    P;     //!< 位置座標の番号.
    uint32_t    N;     //!< 法線ベクトルの番号.
};

///////////////////////////////////////////////////////////////////////////////
// Ray structure
///////////////////////////////////////////////////////////////////////////////
struct Ray
{
    Vector3f    pos;
    Vector3f    dir;
    Vector3f    inv_dir;
    float       tmin;
    float       tmax;
};

///////////////////////////////////////////////////////////////////////////////
// HitRecord structure
///////////////////////////////////////////////////////////////////////////////
struct HitRecord
{
    bool        hit;        //!< 交差したら true.
    float       dist;       //!< 距離.
    float       u;          //!< 重心座標(yに適用).
    float       v;          //!< 重心座標(zに適用).
    int         face_id;    //!< 交差した面の番号.
};

///////////////////////////////////////////////////////////////////////////////
// LBVH structure
///////////////////////////////////////////////////////////////////////////////
struct LBVH
{
    uint32_t                    Root            = kInvalid;
    const Vector3f*             Positions       = nullptr;
    const Vector3f*             Normals         = nullptr;
    const VertexIndex*          Indices         = nullptr;
    size_t                      PositionCount   = 0;
    size_t                      NormalCount     = 0;
    size_t                      IndexCount      = 0;
    std::vector<Node>           Nodes;

    void Build();
    void Destruct();
    void TraverseIterative(const Ray& ray, HitRecord& record) const;

    __forceinline void IsHit(const Ray& ray, HitRecord& record, uint32_t face_id) const noexcept
    {
        const auto  id = face_id * 3;
        if (s3d::IntersectTriangle(
            ray.pos,
            ray.dir,
            Positions[Indices[id + 0].P],
            Positions[Indices[id + 1].P],
            Positions[Indices[id + 2].P],
            ray.tmin,
            ray.tmax,
            record.dist,
            record.u,
            record.v))
        {
            record.face_id = int32_t(face_id);
            record.hit     = true;
        }
    }

    __forceinline Vector3f CalcPosition(uint32_t face_id, float u, float v, float w) const noexcept
    {
        const auto id = face_id * 3;
        return Positions[Indices[id + 0].P] * w
             + Positions[Indices[id + 1].P] * u
             + Positions[Indices[id + 2].P] * v;
    }

    __forceinline Vector3f CalcNormal(uint32_t face_id, float u, float v, float w) const noexcept
    {
        const auto id = face_id * 3;
        return Normals[Indices[id + 0].N] * w
             + Normals[Indices[id + 1].N] * u
             + Normals[Indices[id + 2].N] * v;
    }
};

} // namespace s3d
