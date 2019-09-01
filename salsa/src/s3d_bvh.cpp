//-----------------------------------------------------------------------------
// File : s3d_bvh.cpp
// Desc : Bounding Volume Hierarchy.
// Copyright(c) Project Asura. All right reserved.
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include <s3d_bvh.h>
#include <execution>
#include <ppl.h>



//-----------------------------------------------------------------------------
// Using Statements
//-----------------------------------------------------------------------------
using namespace concurrency;


namespace {

// delta function in sec3 of the paper
// "Fast and Simple Agglomerative LBVH Construction"
__forceinline uint32_t Delta(const std::vector<s3d::Vector2u> &leaves, const uint32_t id)
{ return leaves[id + 1].y ^ leaves[id].y; }

} // namespace


namespace s3d {

///////////////////////////////////////////////////////////////////////////////
// LBVH structure
///////////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------------------
//      構築処理を行います.
//-----------------------------------------------------------------------------
void LBVH::Build()
{
    AABB box;
    box.Clear();

    // 全体のバウンディングボックスを求める.
    for(size_t i=0; i<PositionCount; ++i)
    { box.Merge(Positions[i]); }

    // ポリゴン数.
    const auto T = uint32_t(IndexCount / 3);

    // allocate pair <reference, morton code>
    std::vector<Vector2u> leaves;
    leaves.resize(T);

    // モートンコードを設定.
    parallel_for<uint32_t>(0, T, [&](uint32_t i)
    {
        const auto id = i * 3;
        const auto centroid = (Positions[Indices[id + 0].P] + Positions[Indices[id + 1].P] + Positions[Indices[id + 2].P]) / 3.0f;
        const auto unitcube = box.Normalize(centroid);
        leaves[i].x = i;
        leaves[i].y = Morton3D(unitcube.x, unitcube.y, unitcube.z);
    });

    // 降順でモートンコードをソートする.
    std::sort(std::execution::par, std::begin(leaves), std::end(leaves), [&](const Vector2u& lhs, const Vector2u& rhs)
    {
        return (lhs.y < rhs.y);
    });

    // ノードの数.
    const auto N = T - 1;
    Nodes.resize(N);

    // otherBounds in algorithm 1 of the paper
    // "Massively Parallel Construction of Radix Tree Forests for the Efficient Sampling of Discrete Probability Distributions"
    // https://arxiv.org/pdf/1901.05423.pdf
    std::vector<std::atomic<uint32_t>> other_bounds(N);
    parallel_for<size_t>(0, N, [&](size_t i)
    {
        other_bounds[i].store(kInvalid);
    });

    parallel_for<uint32_t>(0, T, [&](uint32_t i)
    {
        // 現在のリーフ/ノードID.
        auto current = i;

        // 範囲.
        auto L = current;
        auto R = current;

        // 葉ノードのAABB
        const auto id = leaves[i].x * 3;
        AABB aabb (Positions[Indices[id + 0].P]);
        aabb.Merge(Positions[Indices[id + 1].P]);
        aabb.Merge(Positions[Indices[id + 2].P]);
        aabb.mini -= box.mini;
        aabb.maxi -= box.mini;

        // リーフまたはノードか?.
        auto is_leaf = true;
        while(1)
        {
            // 全体の範囲がカバーされたら, おしまい.
            if (0 == L && R == N)
            {
                Root = current;
                break;
            }

            // リーフ/ノード番号.
            // 下位1ビットはリーフノード判定ビットとして利用するため, 1bitシフト.
            const auto index = (is_leaf) ? (leaves[current].x << 1) + 1 : current << 1;

            uint32_t previous, parent;
            if (0 == L || (R != N && Delta(leaves, R) < Delta(leaves, L - 1)) )
            {
                // 右が親で，"左"は変化しない.
                parent = R;
                previous = other_bounds[parent].exchange(L);
                if (kInvalid != previous)
                { R = previous; }
                Nodes[parent].L = index;
            }
            else
            {
                // 親が左で，"右"は変換しない.
                parent = L - 1;
                previous = other_bounds[parent].exchange(R);
                if (kInvalid != previous)
                { L = previous; }
                Nodes[parent].R = index;
            }

            // マージする.
            Nodes[parent].Box.Merge(aabb);

            // このスレッドを終了する.
            if (kInvalid == previous)
            { break; }

            current = parent;
            aabb    = Nodes[current].Box;
            is_leaf = false;
        }
    });

    // ずらした分を戻す.
    parallel_for<size_t>(0, N, [&](size_t i)
    {
        Nodes[i].Box.mini += box.mini;
        Nodes[i].Box.maxi += box.mini;
    });
}

//-----------------------------------------------------------------------------
//      データを破棄します.
//-----------------------------------------------------------------------------
void LBVH::Destruct()
{
    Nodes.clear();
    Nodes.shrink_to_fit();

    PositionCount = 0;
    Positions = nullptr;

    NormalCount = 0;
    Normals = nullptr;

    IndexCount = 0;
    Indices = nullptr;
}

//-----------------------------------------------------------------------------
//      ノードを巡回し交差判定を取ります.
//-----------------------------------------------------------------------------
void LBVH::TraverseIterative(const Ray& ray, HitRecord& record) const
{
    uint32_t visit_stack[64];
    uint32_t stack_ptr = 1;

    // ルートノードだけpush
    visit_stack[0] = Root;

    // スタックが空になるまで処理.
    while(stack_ptr > 0)
    {
        --stack_ptr; // pop.
        const auto  idx  = visit_stack[stack_ptr];
        const auto& node = Nodes[idx];

        if (!node.Box.Intersect(ray.pos, ray.inv_dir, record.dist))
        { continue; }

        const auto idxL = node.L >> 1;
        const auto idxR = node.R >> 1;

        if (node.L & 0x1)
            IsHit(ray, record, idxL);
        else
            visit_stack[stack_ptr++] = idxL; // push.

        if (node.R & 0x1)
            IsHit(ray, record, idxR);
        else
            visit_stack[stack_ptr++] = idxR; // push.
    }
}

} // namespace s3d
