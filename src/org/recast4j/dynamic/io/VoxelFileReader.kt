/*
recast4j copyright (c) 2021 Piotr Piastucki piotr@jtilia.org

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
package org.recast4j.dynamic.io

import org.joml.Vector3f
import org.recast4j.detour.io.IOUtils
import java.io.IOException
import java.io.InputStream
import java.nio.ByteBuffer
import java.nio.ByteOrder

class VoxelFileReader {

    fun read(stream: InputStream): VoxelFile {
        val buf = IOUtils.toByteBuffer(stream)
        val file = VoxelFile()
        var magic = buf.int
        if (magic != VoxelFile.MAGIC) {
            magic = IOUtils.swapEndianness(magic)
            if (magic != VoxelFile.MAGIC) {
                throw IOException("Invalid magic")
            }
            buf.order(if (buf.order() == ByteOrder.BIG_ENDIAN) ByteOrder.LITTLE_ENDIAN else ByteOrder.BIG_ENDIAN)
        }
        file.version = buf.int
        val isExportedFromAstar = file.version and VoxelFile.VERSION_EXPORTER_MASK == 0
        file.walkableRadius = buf.float
        file.walkableHeight = buf.float
        file.walkableClimb = buf.float
        file.walkableSlopeAngle = buf.float
        file.cellSize = buf.float
        file.maxSimplificationError = buf.float
        file.maxEdgeLen = buf.float
        file.minRegionArea = buf.float.toInt().toFloat()
        if (!isExportedFromAstar) {
            file.regionMergeArea = buf.float
            file.verticesPerPoly = buf.int
            file.buildMeshDetail = buf.get().toInt() != 0
            file.detailSampleDistance = buf.float
            file.detailSampleMaxError = buf.float
        } else {
            file.regionMergeArea = 6 * file.minRegionArea
            file.verticesPerPoly = 6
            file.buildMeshDetail = true
            file.detailSampleDistance = file.maxEdgeLen * 0.5f
            file.detailSampleMaxError = file.maxSimplificationError * 0.8f
        }
        file.useTiles = buf.get().toInt() != 0
        file.tileSizeX = buf.int
        file.tileSizeZ = buf.int
        file.rotation.set(buf.float, buf.float, buf.float)
        file.bounds[0] = buf.float
        file.bounds[1] = buf.float
        file.bounds[2] = buf.float
        file.bounds[3] = buf.float
        file.bounds[4] = buf.float
        file.bounds[5] = buf.float
        if (isExportedFromAstar) {
            // bounds are saved as center + size
            file.bounds[0] -= 0.5f * file.bounds[3]
            file.bounds[1] -= 0.5f * file.bounds[4]
            file.bounds[2] -= 0.5f * file.bounds[5]
            file.bounds[3] += file.bounds[0]
            file.bounds[4] += file.bounds[1]
            file.bounds[5] += file.bounds[2]
        }
        val tileCount = buf.int
        for (tile in 0 until tileCount) {
            val tileX = buf.int
            val tileZ = buf.int
            val width = buf.int
            val depth = buf.int
            val borderSize = buf.int
            val boundsMin = Vector3f(buf.float, buf.float, buf.float)
            val boundsMax = Vector3f(buf.float, buf.float, buf.float)
            if (isExportedFromAstar) {
                // bounds are local
                boundsMin.x += file.bounds[0]
                boundsMin.y += file.bounds[1]
                boundsMin.z += file.bounds[2]
                boundsMax.x += file.bounds[0]
                boundsMax.y += file.bounds[1]
                boundsMax.z += file.bounds[2]
            }
            val cellSize = buf.float
            val cellHeight = buf.float
            val voxelSize = buf.int
            val position = buf.position()
            val bytes = ByteArray(voxelSize)
            buf.get(bytes)
            val data = ByteBuffer.wrap(bytes)
            data.order(buf.order())
            file.addTile(
                VoxelTile(
                    tileX, tileZ, width, depth,
                    boundsMin, boundsMax, cellSize, cellHeight,
                    borderSize, data
                )
            )
            buf.position(position + voxelSize)
        }
        return file
    }
}