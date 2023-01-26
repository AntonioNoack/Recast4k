/*
Recast4J Copyright (c) 2015 Piotr Piastucki piotr@jtilia.org

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
package org.recast4j.detour.io

import org.recast4j.detour.MeshData
import org.recast4j.detour.MeshHeader
import java.io.IOException
import java.io.OutputStream
import java.nio.ByteOrder

class MeshDataWriter : DetourWriter() {
    @Throws(IOException::class)
    fun write(stream: OutputStream, data: MeshData, order: ByteOrder) {
        val header = data.header
        write(stream, header!!.magic, order)
        write(stream, MeshHeader.DT_NAVMESH_VERSION_RECAST4J_LAST, order)
        write(stream, header.x, order)
        write(stream, header.y, order)
        write(stream, header.layer, order)
        write(stream, header.userId, order)
        write(stream, header.polyCount, order)
        write(stream, header.vertCount, order)
        write(stream, header.maxLinkCount, order)
        write(stream, header.detailMeshCount, order)
        write(stream, header.detailVertCount, order)
        write(stream, header.detailTriCount, order)
        write(stream, header.bvNodeCount, order)
        write(stream, header.offMeshConCount, order)
        write(stream, header.offMeshBase, order)
        write(stream, header.walkableHeight, order)
        write(stream, header.walkableRadius, order)
        write(stream, header.walkableClimb, order)
        write(stream, header.bmin, order)
        write(stream, header.bmax, order)
        write(stream, header.bvQuantizationFactor, order)
        writeVertices(stream, data.vertices, header.vertCount, order)
        writePolys(stream, data, order)
        writePolyDetails(stream, data, order)
        writeVertices(stream, data.detailVertices, header.detailVertCount, order)
        writeDTris(stream, data)
        writeBVTree(stream, data, order)
        writeOffMeshCons(stream, data, order)
    }

    @Throws(IOException::class)
    private fun writeVertices(stream: OutputStream, vertices: FloatArray, count: Int, order: ByteOrder) {
        for (i in 0 until count * 3) {
            write(stream, vertices[i], order)
        }
    }

    @Throws(IOException::class)
    private fun writePolys(stream: OutputStream, data: MeshData, order: ByteOrder) {
        for (i in 0 until data.header!!.polyCount) {
            for (j in data.polygons[i].vertices.indices) {
                write(stream, data.polygons[i].vertices[j].toShort(), order)
            }
            for (j in data.polygons[i].neighborData.indices) {
                write(stream, data.polygons[i].neighborData[j].toShort(), order)
            }
            write(stream, data.polygons[i].flags.toShort(), order)
            stream.write(data.polygons[i].vertCount)
            stream.write(data.polygons[i].areaAndType)
        }
    }

    @Throws(IOException::class)
    private fun writePolyDetails(stream: OutputStream, data: MeshData, order: ByteOrder) {
        for (i in 0 until data.header!!.detailMeshCount) {
            write(stream, data.detailMeshes!![i].vertBase, order)
            write(stream, data.detailMeshes!![i].triBase, order)
            stream.write(data.detailMeshes!![i].vertCount)
            stream.write(data.detailMeshes!![i].triCount)
        }
    }

    @Throws(IOException::class)
    private fun writeDTris(stream: OutputStream, data: MeshData) {
        for (i in 0 until data.header!!.detailTriCount * 4) {
            stream.write(data.detailTriangles[i])
        }
    }

    @Throws(IOException::class)
    private fun writeBVTree(stream: OutputStream, data: MeshData, order: ByteOrder) {
        for (i in 0 until data.header!!.bvNodeCount) {
            write(stream, data.bvTree!![i].minX, order)
            write(stream, data.bvTree!![i].minY, order)
            write(stream, data.bvTree!![i].minZ, order)
            write(stream, data.bvTree!![i].maxX, order)
            write(stream, data.bvTree!![i].maxY, order)
            write(stream, data.bvTree!![i].maxZ, order)
            write(stream, data.bvTree!![i].index, order)
        }
    }

    @Throws(IOException::class)
    private fun writeOffMeshCons(stream: OutputStream, data: MeshData, order: ByteOrder) {
        for (i in 0 until data.header!!.offMeshConCount) {
            write(stream, data.offMeshCons[i].posA, order)
            write(stream, data.offMeshCons[i].posB, order)
            write(stream, data.offMeshCons[i].rad, order)
            write(stream, data.offMeshCons[i].poly.toShort(), order)
            stream.write(data.offMeshCons[i].flags)
            stream.write(data.offMeshCons[i].side)
            write(stream, data.offMeshCons[i].userId, order)
        }
    }
}