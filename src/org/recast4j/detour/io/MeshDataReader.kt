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

import org.recast4j.detour.*
import java.io.IOException
import java.io.InputStream
import java.nio.ByteBuffer
import java.nio.ByteOrder

class MeshDataReader {

    fun read(stream: InputStream, maxVertPerPoly: Int): MeshData {
        val buf = IOUtils.toByteBuffer(stream)
        return read(buf, maxVertPerPoly, false)
    }

    fun read(buf: ByteBuffer, maxVertPerPoly: Int): MeshData {
        return read(buf, maxVertPerPoly, false)
    }

    fun read32Bit(stream: InputStream, maxVertPerPoly: Int): MeshData {
        val buf = IOUtils.toByteBuffer(stream)
        return read(buf, maxVertPerPoly, true)
    }

    fun read32Bit(buf: ByteBuffer, maxVertPerPoly: Int): MeshData {
        return read(buf, maxVertPerPoly, true)
    }

    fun read(buf: ByteBuffer, maxVertPerPoly: Int, is32Bit: Boolean): MeshData {
        val data = MeshData()
        val header = MeshHeader()
        data.header = header
        header.magic = buf.int
        if (header.magic != MeshHeader.DT_NAVMESH_MAGIC) {
            header.magic = IOUtils.swapEndianness(header.magic)
            if (header.magic != MeshHeader.DT_NAVMESH_MAGIC) {
                throw IOException("Invalid magic")
            }
            buf.order(if (buf.order() == ByteOrder.BIG_ENDIAN) ByteOrder.LITTLE_ENDIAN else ByteOrder.BIG_ENDIAN)
        }
        header.version = buf.int
        if (header.version != MeshHeader.DT_NAVMESH_VERSION) {
            if (header.version < MeshHeader.DT_NAVMESH_VERSION_RECAST4J_FIRST
                || header.version > MeshHeader.DT_NAVMESH_VERSION_RECAST4J_LAST
            ) {
                throw IOException("Invalid version " + header.version)
            }
        }
        val cCompatibility = header.version == MeshHeader.DT_NAVMESH_VERSION
        header.x = buf.int
        header.y = buf.int
        header.layer = buf.int
        header.userId = buf.int
        header.polyCount = buf.int
        header.vertCount = buf.int
        header.maxLinkCount = buf.int
        header.detailMeshCount = buf.int
        header.detailVertCount = buf.int
        header.detailTriCount = buf.int
        header.bvNodeCount = buf.int
        header.offMeshConCount = buf.int
        header.offMeshBase = buf.int
        header.walkableHeight = buf.float
        header.walkableRadius = buf.float
        header.walkableClimb = buf.float
        header.bmin.set(buf.float, buf.float, buf.float)
        header.bmax.set(buf.float, buf.float, buf.float)
        header.bvQuantizationFactor = buf.float
        data.vertices = readVertices(buf, header.vertCount)
        data.polygons = readPolys(buf, header, maxVertPerPoly)
        if (cCompatibility) {
            buf.position(buf.position() + header.maxLinkCount * getSizeofLink(is32Bit))
        }
        data.detailMeshes = readPolyDetails(buf, header, cCompatibility)
        data.detailVertices = readVertices(buf, header.detailVertCount)
        data.detailTriangles = readDTris(buf, header)
        data.bvTree = readBVTree(buf, header)
        data.offMeshCons = readOffMeshCons(buf, header)
        return data
    }

    private fun readVertices(buf: ByteBuffer, count: Int): FloatArray {
        val vertices = FloatArray(count * 3)
        for (i in vertices.indices) {
            vertices[i] = buf.float
        }
        return vertices
    }

    private fun readPolys(buf: ByteBuffer, header: MeshHeader, maxVertPerPoly: Int): Array<Poly> {
        val polys = Array(header.polyCount) { Poly(it, maxVertPerPoly) }
        for (i in polys.indices) {
            if (header.version < MeshHeader.DT_NAVMESH_VERSION_RECAST4J_NO_POLY_FIRSTLINK) {
                buf.int // polys[i].getFirst()Link
            }
            for (j in polys[i].vertices.indices) {
                polys[i].vertices[j] = buf.short.toInt() and 0xFFFF
            }
            for (j in polys[i].neighborData.indices) {
                polys[i].neighborData[j] = buf.short.toInt() and 0xFFFF
            }
            polys[i].flags = buf.short.toInt() and 0xFFFF
            polys[i].vertCount = buf.get().toInt() and 0xFF
            polys[i].areaAndType = buf.get().toInt() and 0xFF
        }
        return polys
    }

    private fun readPolyDetails(buf: ByteBuffer, header: MeshHeader, cCompatibility: Boolean): Array<PolyDetail> {
        val polys = Array(header.detailMeshCount) { PolyDetail() }
        for (i in polys.indices) {
            polys[i].vertBase = buf.int
            polys[i].triBase = buf.int
            polys[i].vertCount = buf.get().toInt() and 0xFF
            polys[i].triCount = buf.get().toInt() and 0xFF
            if (cCompatibility) {
                buf.short // C struct padding
            }
        }
        return polys
    }

    private fun readDTris(buf: ByteBuffer, header: MeshHeader): IntArray {
        val tris = IntArray(4 * header.detailTriCount)
        for (i in tris.indices) {
            tris[i] = buf.get().toInt() and 0xFF
        }
        return tris
    }

    private fun readBVTree(buf: ByteBuffer, header: MeshHeader): Array<BVNode> {
        val nodes = Array(header.bvNodeCount) { BVNode() }
        for (i in nodes.indices) {
            val n = nodes[i]
            n.minX = buf.int
            n.minY = buf.int
            n.minZ = buf.int
            n.maxX = buf.int
            n.maxY = buf.int
            n.maxZ = buf.int
            n.index = buf.int
        }
        return nodes
    }

    private fun readOffMeshCons(buf: ByteBuffer, header: MeshHeader): Array<OffMeshConnection> {
        val cons = Array(header.offMeshConCount) { OffMeshConnection() }
        for (i in cons.indices) {
            cons[i].posA.set(buf.float, buf.float, buf.float)
            cons[i].posB.set(buf.float, buf.float, buf.float)
            cons[i].rad = buf.float
            cons[i].poly = buf.short.toInt() and 0xffff
            cons[i].flags = buf.get().toInt() and 0xff
            cons[i].side = buf.get().toInt() and 0xff
            cons[i].userId = buf.int
        }
        return cons
    }

    companion object {
        const val LINK_SIZEOF = 16
        const val LINK_SIZEOF32BIT = 12
        fun getSizeofLink(is32Bit: Boolean): Int {
            return if (is32Bit) LINK_SIZEOF32BIT else LINK_SIZEOF
        }
    }
}