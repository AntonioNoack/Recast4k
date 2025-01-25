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
package org.recast4j.detour

import jdk.nashorn.internal.objects.NativeMath
import org.joml.Vector3f
import org.recast4j.Vectors
import org.recast4j.detour.NavMeshBuilder.createNavMeshData
import org.recast4j.detour.NavMeshBuilder.subdivide

class MeshData : MeshHeader() {

    val header get() = this

    /**
     * The tile vertices. [Size: MeshHeader::vertCount]
     */
    lateinit var vertices: FloatArray

    /**
     * The tile polygons. [Size: MeshHeader::polyCount]
     */
    lateinit var polygons: Array<Poly>

    /**
     * The tile's detail sub-meshes. [Size: MeshHeader::detailMeshCount]
     */
    var detailMeshes: Array<PolyDetail>? = null

    /**
     * The detail mesh's unique vertices. [(x, y, z) * MeshHeader::detailVertCount]
     */
    lateinit var detailVertices: FloatArray

    /**
     * The detail mesh's triangles. [(vertA, vertB, vertC) * MeshHeader::detailTriCount] See DetailTriEdgeFlags and
     * NavMesh::getDetailTriEdgeFlags. Unsigned bytes; 4 per each detail triangle (???)
     */
    lateinit var detailTriangles: ByteArray

    /**
     * The tile bounding volume nodes. [Size: MeshHeader::bvNodeCount] (Will be null if bounding volumes are disabled.)
     */
    var bvTree: Array<BVNode>? = null

    /**
     * The tile off-mesh connections. [Size: MeshHeader::offMeshConCount]
     */
    lateinit var offMeshCons: Array<OffMeshConnection>

    companion object {
        fun build(params: NavMeshDataCreateParams, tileX: Int, tileY: Int): MeshData? {
            val data = createNavMeshData(params)
            if (data != null) {
                data.x = tileX
                data.y = tileY
            }
            return data
        }

        fun build(data: MeshData) {
            data.bvTree = Array(data.polyCount * 2) { BVNode() }
            data.bvNodeCount =
                if (data.bvTree!!.isEmpty()) 0 else
                    createBVTree(data, data.bvTree!!, data.bvQuantizationFactor)
        }

        private fun createBVTree(data: MeshData, BVNodes: Array<BVNode>, quantFactor: Float): Int {
            val items = Array(data.polyCount) { BVNode() }
            for (i in 0 until data.polyCount) {
                val node = items[i]
                node.index = i
                val bmin = Vector3f()
                bmin.set(data.vertices, data.polygons[i].vertices[0] * 3)
                val bmax = Vector3f(bmin)
                for (j in 1 until data.polygons[i].vertCount) {
                    val idx = data.polygons[i].vertices[j] * 3
                    NativeMath.min(bmin, data.vertices, idx)
                    NativeMath.max(bmax, data.vertices, idx)
                }
                val header = data
                val headerMin = header.bmin
                node.minX = quantitize(bmin.x, headerMin.x, quantFactor)
                node.minY = quantitize(bmin.y, headerMin.y, quantFactor)
                node.minZ = quantitize(bmin.z, headerMin.z, quantFactor)
                node.maxX = quantitize(bmax.x, headerMin.x, quantFactor)
                node.maxY = quantitize(bmax.y, headerMin.y, quantFactor)
                node.maxZ = quantitize(bmax.z, headerMin.z, quantFactor)
            }
            return subdivide(items, 0, data.polyCount, 0, BVNodes)
        }

        private fun quantitize(bmin: Float, headerMin: Float, quantFactor: Float): Int {
            return Vectors.clamp(((bmin - headerMin) * quantFactor).toInt(), 0, 0x7fffffff)
        }
    }
}