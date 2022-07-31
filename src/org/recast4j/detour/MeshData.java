/*
Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
recast4j copyright (c) 2015-2019 Piotr Piastucki piotr@jtilia.org

This software is provided 'as-is', without any express or implied
warranty.  In no event will the authors be held liable for any damages
arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:
1. The origin of this software must not be misrepresented; you must not
 claim that you wrote the original software. If you use this software
 in a product, an acknowledgment in the product documentation would be
 appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
 misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
package org.recast4j.detour;

import org.joml.Vector3f;

import static jdk.nashorn.internal.objects.NativeMath.max;
import static jdk.nashorn.internal.objects.NativeMath.min;
import static org.joml.Math.clamp;
import static org.recast4j.Vectors.copy;

public class MeshData {

    /**
     * The tile header.
     */
    public MeshHeader header;
    /**
     * The tile vertices. [Size: MeshHeader::vertCount]
     */
    public float[] vertices;
    /**
     * The tile polygons. [Size: MeshHeader::polyCount]
     */
    public Poly[] polygons;
    /**
     * The tile's detail sub-meshes. [Size: MeshHeader::detailMeshCount]
     */
    public PolyDetail[] detailMeshes;
    /**
     * The detail mesh's unique vertices. [(x, y, z) * MeshHeader::detailVertCount]
     */
    public float[] detailVertices;
    /**
     * The detail mesh's triangles. [(vertA, vertB, vertC) * MeshHeader::detailTriCount] See DetailTriEdgeFlags and
     * NavMesh::getDetailTriEdgeFlags.
     */
    public int[] detailTriangles;
    /**
     * The tile bounding volume nodes. [Size: MeshHeader::bvNodeCount] (Will be null if bounding volumes are disabled.)
     */
    public BVNode[] bvTree;
    /**
     * The tile off-mesh connections. [Size: MeshHeader::offMeshConCount]
     */
    public OffMeshConnection[] offMeshCons;

    public static MeshData build(NavMeshDataCreateParams params, int tileX, int tileY) {
        MeshData data = NavMeshBuilder.createNavMeshData(params);
        if (data != null) {
            data.header.x = tileX;
            data.header.y = tileY;
        }
        return data;
    }

    public static void build(MeshData data) {
        data.bvTree = new BVNode[data.header.polyCount * 2];
        data.header.bvNodeCount = data.bvTree.length == 0 ? 0
                : createBVTree(data, data.bvTree, data.header.bvQuantizationFactor);
    }

    private static int createBVTree(MeshData data, BVNode[] BVNodes, float quantFactor) {
        BVNode[] items = new BVNode[data.header.polyCount];
        for (int i = 0; i < data.header.polyCount; i++) {
            BVNode it = new BVNode();
            items[i] = it;
            it.index = i;
            Vector3f bmin = new Vector3f();
            copy(bmin, data.vertices, data.polygons[i].vertices[0] * 3);
            Vector3f bmax = new Vector3f(bmin);
            for (int j = 1; j < data.polygons[i].vertCount; j++) {
                int idx = data.polygons[i].vertices[j] * 3;
                min(bmin, data.vertices, idx);
                max(bmax, data.vertices, idx);
            }
            it.minX = clamp((int) ((bmin.x - data.header.bmin.x) * quantFactor), 0, 0x7fffffff);
            it.minY = clamp((int) ((bmin.y - data.header.bmin.y) * quantFactor), 0, 0x7fffffff);
            it.minZ = clamp((int) ((bmin.z - data.header.bmin.z) * quantFactor), 0, 0x7fffffff);
            it.maxX = clamp((int) ((bmax.x - data.header.bmin.x) * quantFactor), 0, 0x7fffffff);
            it.maxY = clamp((int) ((bmax.y - data.header.bmin.y) * quantFactor), 0, 0x7fffffff);
            it.maxZ = clamp((int) ((bmax.z - data.header.bmin.z) * quantFactor), 0, 0x7fffffff);
        }
        return NavMeshBuilder.subdivide(items, 0, data.header.polyCount, 0, BVNodes);
    }

}