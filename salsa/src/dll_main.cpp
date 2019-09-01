//-----------------------------------------------------------------------------
// File : dll_main.cpp
// Desc : DLL Main Entry Point.
// Copyright(c) Project Asura. All right reserved.
//-----------------------------------------------------------------------------

#if 1

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include <cstdint>
#include <Windows.h>
#include "../../src/rayrun.hpp"
#include <s3d_bvh.h>
#include <ppl.h>


//-----------------------------------------------------------------------------
// Using Statements
//-----------------------------------------------------------------------------
using namespace concurrency;


//-----------------------------------------------------------------------------
// Gloval Variables.
//-----------------------------------------------------------------------------
s3d::LBVH gLBVH;        // Linear BVH.

//-----------------------------------------------------------------------------
//      DLLメインエントリーポイントです.
//-----------------------------------------------------------------------------
BOOL APIENTRY DllMain(HMODULE, DWORD, LPVOID)
{ return TRUE; }

//-----------------------------------------------------------------------------
//      競技で定められている事前処理関数(スレッドによる規定は書いてない).
//-----------------------------------------------------------------------------
void preprocess
(
    const float*    vertices,       // 頂点座標配列.
    size_t          vertexCount,    // 頂点数.
    const float*    normals,        // 法線配列.
    size_t          normalCount,    // 法線数.
    const uint32_t* indices,        // 頂点インデックス(v0, n0, v1, n1, v2, n2...のように格納される, また三角形はすでに行われているものとする).
    size_t          faceCount       // 頂点インデックス数.
)
{
    gLBVH.PositionCount = vertexCount;
    gLBVH.Positions     = reinterpret_cast<const s3d::Vector3f*>(vertices);

    gLBVH.NormalCount   = normalCount;
    gLBVH.Normals       = reinterpret_cast<const s3d::Vector3f*>(normals);

    gLBVH.IndexCount    = faceCount * 3;
    gLBVH.Indices       = reinterpret_cast<const s3d::VertexIndex*>(indices);

    gLBVH.Build();
}

//------------------------------------------------------------------------------
//      競技で定められている交差判定関数(マルチスレッドで呼び出される).
//------------------------------------------------------------------------------
void intersect
(
    Ray*    rays,           // レイ配列.
    size_t  rayCount,       // レイ数.
    bool    /*hitAny*/      // 交差が1つ以上あることが確定した段階で戻るか?
)
{
    // ここはparallel_for化してもそんなに早くならなかった(むしろ，ちょっと遅くなる).
    for(size_t i=0; i<rayCount; ++i)
    {
        // 無効なレイは処理しない.
        if (!rays[i].valid)
        {
            rays[i].isisect = false;
            continue;
        }

        s3d::Ray ray;
        ray.pos.x = rays[i].pos[0];
        ray.pos.y = rays[i].pos[1];
        ray.pos.z = rays[i].pos[2];

        ray.dir.x = rays[i].dir[0];
        ray.dir.y = rays[i].dir[1];
        ray.dir.z = rays[i].dir[2];

        ray.inv_dir.x = 1.0f / ray.dir.x;
        ray.inv_dir.y = 1.0f / ray.dir.y;
        ray.inv_dir.z = 1.0f / ray.dir.z;

        ray.tmin = rays[i].tnear;
        ray.tmax = rays[i].tfar;

        s3d::HitRecord record;
        record.hit  = false;
        record.dist = rays[i].tfar;

        gLBVH.TraverseIterative(ray, record);
        rays[i].isisect = record.hit;

        // 交差していた場合のみ計算を行う.
        if (record.hit)
        { 
            auto w = 1.0f - record.u - record.v;
            auto pos = gLBVH.CalcPosition(record.face_id, record.u, record.v, w);
            rays[i].isect[0] = pos.x;
            rays[i].isect[1] = pos.y;
            rays[i].isect[2] = pos.z;

            auto nrm = gLBVH.CalcNormal(record.face_id, record.u, record.v, w);
            rays[i].ns[0] = nrm.x;
            rays[i].ns[1] = nrm.y;
            rays[i].ns[2] = nrm.z;
        }
    }
}

#endif