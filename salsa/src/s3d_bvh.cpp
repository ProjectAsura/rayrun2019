//-----------------------------------------------------------------------------
// File : s3d_bvh.cpp
// Desc : Bounding Volume Hierarchy.
// Copyright(c) Project Asura. All right reserved.
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include <s3d_bvh.h>


namespace s3d {

BVH2::BVH2()
{ /* DO_NOTHING */ }

BVH2::~BVH2()
{ /* DO_NOTHING */ }

bool BVH2::Build
(
    const float*    positions,
    const float*    normals,
    const uint32_t* indices,
    size_t          faceCount
)
{
    if (positions == nullptr || normals == nullptr || indices == nullptr || faceCount == 0)
    { return false; }

    // 三角形リスト構築.
    {
        m_Triangles.resize(faceCount);
        for(size_t i=0, idx=0; i<faceCount; i++, idx+=6)
        {
            // 位置座標設定.
            {
                auto v0 = indices[idx + 0];
                auto v1 = indices[idx + 2];
                auto v2 = indices[idx + 4];

                m_Triangles[i].p[0].x = positions[v0 + 0];
                m_Triangles[i].p[0].y = positions[v0 + 1];
                m_Triangles[i].p[0].z = positions[v0 + 2];

                m_Triangles[i].p[1].x = positions[v1 + 0];
                m_Triangles[i].p[1].y = positions[v1 + 1];
                m_Triangles[i].p[1].z = positions[v1 + 2];

                m_Triangles[i].p[2].x = positions[v2 + 0];
                m_Triangles[i].p[2].y = positions[v2 + 1];
                m_Triangles[i].p[2].z = positions[v2 + 2];
            }

            // 法線ベクトル設定.
            {
                auto n0 = indices[idx + 1];
                auto n1 = indices[idx + 3];
                auto n2 = indices[idx + 5];

                m_Triangles[i].n[0].x = normals[n0 + 0];
                m_Triangles[i].n[0].y = normals[n0 + 1];
                m_Triangles[i].n[0].z = normals[n0 + 2];

                m_Triangles[i].n[1].x = normals[n1 + 0];
                m_Triangles[i].n[1].y = normals[n1 + 1];
                m_Triangles[i].n[1].z = normals[n1 + 2];

                m_Triangles[i].n[2].x = normals[n2 + 0];
                m_Triangles[i].n[2].y = normals[n2 + 1];
                m_Triangles[i].n[2].z = normals[n2 + 2];
            }

            // AABB設定.
            m_Triangles[i].box.maxi = m_Triangles[i].box.mini = m_Triangles[i].p[0];
            m_Triangles[i].box.Merge(m_Triangles[i].p[1]);
            m_Triangles[i].box.Merge(m_Triangles[i].p[2]);
        }
    }

    /* TODO : 高速化のために再帰を使わない */

    return false;
}

Node2 BVH2::BuildNode()
{
    Node2 result;


    return result;
}



} // namespace s3d
