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

import org.recast4j.Vectors
import org.recast4j.detour.NavMesh
import org.recast4j.detour.NavMeshParams
import java.io.IOException
import java.io.InputStream
import java.nio.ByteBuffer
import java.nio.ByteOrder

class MeshSetReader {
    private val meshReader = MeshDataReader()
    private val paramReader = NavMeshParamReader()
    @Throws(IOException::class)
    fun read(`is`: InputStream?, maxVertPerPoly: Int): NavMesh {
        return read(IOUtils.toByteBuffer(`is`), maxVertPerPoly, false)
    }

    @Throws(IOException::class)
    fun read(bb: ByteBuffer, maxVertPerPoly: Int): NavMesh {
        return read(bb, maxVertPerPoly, false)
    }

    @Throws(IOException::class)
    fun read32Bit(`is`: InputStream?, maxVertPerPoly: Int): NavMesh {
        return read(IOUtils.toByteBuffer(`is`), maxVertPerPoly, true)
    }

    @Throws(IOException::class)
    fun read32Bit(bb: ByteBuffer, maxVertPerPoly: Int): NavMesh {
        return read(bb, maxVertPerPoly, true)
    }

    @Throws(IOException::class)
    fun read(`is`: InputStream?): NavMesh {
        return read(IOUtils.toByteBuffer(`is`))
    }

    @Throws(IOException::class)
    fun read(bb: ByteBuffer): NavMesh {
        return read(bb, -1, false)
    }

    @Throws(IOException::class)
    fun read(bb: ByteBuffer, maxVertPerPoly: Int, is32Bit: Boolean): NavMesh {
        val header = readHeader(bb, maxVertPerPoly)
        if (header.maxVerticesPerPoly <= 0) {
            throw IOException("Invalid number of vertices per poly " + header.maxVerticesPerPoly)
        }
        val cCompatibility = header.version == NavMeshSetHeader.NAVMESHSET_VERSION
        val mesh = NavMesh(header.params, header.maxVerticesPerPoly)
        readTiles(bb, is32Bit, header, cCompatibility, mesh)
        return mesh
    }

    @Throws(IOException::class)
    private fun readHeader(bb: ByteBuffer, maxVerticesPerPoly: Int): NavMeshSetHeader {
        val header = NavMeshSetHeader()
        header.magic = bb.int
        if (header.magic != NavMeshSetHeader.NAVMESHSET_MAGIC) {
            header.magic = IOUtils.swapEndianness(header.magic)
            if (header.magic != NavMeshSetHeader.NAVMESHSET_MAGIC) {
                throw IOException("Invalid magic " + header.magic)
            }
            bb.order(if (bb.order() == ByteOrder.BIG_ENDIAN) ByteOrder.LITTLE_ENDIAN else ByteOrder.BIG_ENDIAN)
        }
        header.version = bb.int
        if (header.version != NavMeshSetHeader.NAVMESHSET_VERSION && header.version != NavMeshSetHeader.NAVMESHSET_VERSION_RECAST4J_1 && header.version != NavMeshSetHeader.NAVMESHSET_VERSION_RECAST4J) {
            throw IOException("Invalid version " + header.version)
        }
        header.numTiles = bb.int
        header.params = paramReader.read(bb)
        header.maxVerticesPerPoly = maxVerticesPerPoly
        if (header.version == NavMeshSetHeader.NAVMESHSET_VERSION_RECAST4J) {
            header.maxVerticesPerPoly = bb.int
        }
        return header
    }

    @Throws(IOException::class)
    private fun readTiles(
        bb: ByteBuffer,
        is32Bit: Boolean,
        header: NavMeshSetHeader,
        cCompatibility: Boolean,
        mesh: NavMesh
    ) {
        // Read tiles.
        for (i in 0 until header.numTiles) {
            val tileHeader = NavMeshTileHeader()
            if (is32Bit) {
                tileHeader.tileRef = convert32BitRef(bb.int, header.params)
            } else {
                tileHeader.tileRef = bb.long
            }
            tileHeader.dataSize = bb.int
            if (tileHeader.tileRef == 0L || tileHeader.dataSize == 0) {
                break
            }
            if (cCompatibility && !is32Bit) {
                bb.int // C struct padding
            }
            val data = meshReader.read(bb, mesh.maxVerticesPerPoly, is32Bit)
            mesh.addTile(data, i, tileHeader.tileRef)
        }
    }

    private fun convert32BitRef(ref: Int, params: NavMeshParams): Long {
        val m_tileBits = Vectors.ilog2(Vectors.nextPow2(params.maxTiles))
        val m_polyBits = Vectors.ilog2(Vectors.nextPow2(params.maxPolys))
        // Only allow 31 salt bits, since the salt mask is calculated using 32bit uint and it will overflow.
        val m_saltBits = Math.min(31, 32 - m_tileBits - m_polyBits)
        val saltMask = (1 shl m_saltBits) - 1
        val tileMask = (1 shl m_tileBits) - 1
        val polyMask = (1 shl m_polyBits) - 1
        val salt = ref shr m_polyBits + m_tileBits and saltMask
        val it = ref shr m_polyBits and tileMask
        val ip = ref and polyMask
        return NavMesh.encodePolyId(salt, it, ip)
    }
}