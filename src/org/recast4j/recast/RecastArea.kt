/*
Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
recast4J Copyright (c) 2015-2019 Piotr Piastucki piotr@jtilia.org

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
package org.recast4j.recast

import org.joml.Vector3f
import org.recast4j.Vectors
import java.util.*
import kotlin.math.max
import kotlin.math.min

class RecastArea {

    /**
     * This filter is usually applied after applying area id's using functions
     * such as #rcMarkBoxArea, #rcMarkConvexPolyArea, and #rcMarkCylinderArea.
     *
     * @see rcCompactHeightfield
     */
    fun medianFilterWalkableArea(ctx: Telemetry?, chf: CompactHeightfield): Boolean {
        val w = chf.width
        val h = chf.height
        ctx?.startTimer(TelemetryType.MEDIAN_AREA)
        val areas = IntArray(chf.spanCount)
        val nei = IntArray(9)
        for (y in 0 until h) {
            for (x in 0 until w) {
                val c = x + y * w
                for (i in chf.index[c] until chf.endIndex[c]) {
                    val s = chf.spans[i]
                    if (chf.areas[i] == RecastConstants.RC_NULL_AREA) {
                        areas[i] = chf.areas[i]
                        continue
                    }
                    nei.fill(chf.areas[i])
                    for (dir in 0..3) {
                        medianFilterWalkableArea(chf, s, dir, x, y, w, nei)
                    }
                    Arrays.sort(nei)
                    areas[i] = nei[4] // median
                }
            }
        }
        System.arraycopy(areas, 0, chf.areas, 0, chf.spanCount)
        ctx?.stopTimer(TelemetryType.MEDIAN_AREA)
        return true
    }

    fun medianFilterWalkableArea(
        chf: CompactHeightfield, s: CompactSpan,
        dir: Int, x: Int, y: Int, w: Int, nei: IntArray
    ) {
        if (RecastCommon.getCon(s, dir) == RecastConstants.RC_NOT_CONNECTED) return
        val ax = x + RecastCommon.getDirOffsetX(dir)
        val ay = y + RecastCommon.getDirOffsetY(dir)
        val ai = chf.index[ax + ay * w] + RecastCommon.getCon(s, dir)
        if (chf.areas[ai] != RecastConstants.RC_NULL_AREA) nei[dir * 2] = chf.areas[ai]
        val asp = chf.spans[ai]
        val dir2 = dir + 1 and 0x3
        if (RecastCommon.getCon(asp, dir2) != RecastConstants.RC_NOT_CONNECTED) {
            val ax2 = ax + RecastCommon.getDirOffsetX(dir2)
            val ay2 = ay + RecastCommon.getDirOffsetY(dir2)
            val ai2 = chf.index[ax2 + ay2 * w] + RecastCommon.getCon(asp, dir2)
            if (chf.areas[ai2] != RecastConstants.RC_NULL_AREA) nei[dir * 2 + 1] = chf.areas[ai2]
        }
    }

    fun markBoxArea(
        ctx: Telemetry?,
        bmin: Vector3f,
        bmax: Vector3f,
        areaMod: AreaModification,
        chf: CompactHeightfield
    ) {
        // The value of spacial parameters are in world units.
        ctx?.startTimer(TelemetryType.MARK_BOX_AREA)
        var minx = ((bmin.x - chf.bmin.x) / chf.cellSize).toInt()
        val miny = ((bmin.y - chf.bmin.y) / chf.cellHeight).toInt()
        var minz = ((bmin.z - chf.bmin.z) / chf.cellSize).toInt()
        var maxx = ((bmax.x - chf.bmin.x) / chf.cellSize).toInt()
        val maxy = ((bmax.y - chf.bmin.y) / chf.cellHeight).toInt()
        var maxz = ((bmax.z - chf.bmin.z) / chf.cellSize).toInt()
        minx = max(minx, 0)
        maxx = min(maxx, chf.width - 1)
        minz = max(minz, 0)
        maxz = min(maxz, chf.height - 1)
        for (z in minz..maxz) {
            for (x in minx..maxx) {
                val c = x + z * chf.width
                for (i in chf.index[c] until chf.endIndex[c]) {
                    val s = chf.spans[i]
                    if (s.y in miny..maxy) {
                        if (chf.areas[i] != RecastConstants.RC_NULL_AREA) {
                            chf.areas[i] = areaMod.apply(chf.areas[i])
                        }
                    }
                }
            }
        }
        ctx?.stopTimer(TelemetryType.MARK_BOX_AREA)
    }

    /**
     * The value of spacial parameters are in world units.
     *
     * @see rcCompactHeightfield, rcMedianFilterWalkableArea
     */
    fun markCylinderArea(
        ctx: Telemetry?, pos: Vector3f, r: Float, h: Float, areaMod: AreaModification,
        chf: CompactHeightfield
    ) {
        ctx?.startTimer(TelemetryType.MARK_CYLINDER_AREA)
        val bmin = Vector3f()
        val bmax = Vector3f()
        bmin.x = pos.x - r
        bmin.y = pos.y
        bmin.z = pos.z - r
        bmax.x = pos.x + r
        bmax.y = pos.y + h
        bmax.z = pos.z + r
        val r2 = r * r
        var minx = ((bmin.x - chf.bmin.x) / chf.cellSize).toInt()
        val miny = ((bmin.y - chf.bmin.y) / chf.cellHeight).toInt()
        var minz = ((bmin.z - chf.bmin.z) / chf.cellSize).toInt()
        var maxx = ((bmax.x - chf.bmin.x) / chf.cellSize).toInt()
        var maxy = ((bmax.y - chf.bmin.y) / chf.cellHeight).toInt()
        val maxz = ((bmax.z - chf.bmin.z) / chf.cellSize).toInt()
        minx = max(minx, 0)
        minz = max(minz, 0)
        maxx = min(maxx, chf.width - 1)
        maxy = min(maxy, chf.height - 1)
        for (z in minz..maxz) {
            for (x in minx..maxx) {
                val c = x + z * chf.width
                for (i in chf.index[c] until chf.endIndex[c]) {
                    if (chf.areas[i] == RecastConstants.RC_NULL_AREA) {
                        continue
                    }
                    val s = chf.spans[i]
                    if (s.y in miny..maxy) {
                        val sx = chf.bmin.x + (x + 0.5f) * chf.cellSize
                        val sz = chf.bmin.z + (z + 0.5f) * chf.cellSize
                        val dx = sx - pos.x
                        val dz = sz - pos.z
                        if (dx * dx + dz * dz < r2) {
                            chf.areas[i] = areaMod.apply(chf.areas[i])
                        }
                    }
                }
            }
        }
        ctx?.stopTimer(TelemetryType.MARK_CYLINDER_AREA)
    }

    companion object {
        /**
         * Basically, any spans that are closer to a boundary or obstruction than the specified radius are marked as unwalkable.
         * This method is usually called immediately after the heightfield has been built.
         */
        fun erodeWalkableArea(ctx: Telemetry?, radius: Int, chf: CompactHeightfield) {
            val w = chf.width
            val h = chf.height
            ctx?.startTimer(TelemetryType.ERODE_AREA)
            val dist = IntArray(chf.spanCount)
            dist.fill(255)
            // Mark boundary cells.
            for (y in 0 until h) {
                for (x in 0 until w) {
                    val c = x + y * w
                    for (i in chf.index[c] until chf.endIndex[c]) {
                        if (chf.areas[i] == RecastConstants.RC_NULL_AREA) {
                            dist[i] = 0
                        } else {
                            val s = chf.spans[i]
                            var nc = 0
                            for (dir in 0..3) {
                                if (RecastCommon.getCon(s, dir) != RecastConstants.RC_NOT_CONNECTED) {
                                    val nx = x + RecastCommon.getDirOffsetX(dir)
                                    val ny = y + RecastCommon.getDirOffsetY(dir)
                                    val nidx = chf.index[nx + ny * w] + RecastCommon.getCon(s, dir)
                                    if (chf.areas[nidx] != RecastConstants.RC_NULL_AREA) {
                                        nc++
                                    }
                                }
                            }
                            // At least one missing neighbour.
                            if (nc != 4) dist[i] = 0
                        }
                    }
                }
            }

            // Pass 1
            for (y in 0 until h) {
                for (x in 0 until w) {
                    val c = x + y * w
                    for (i in chf.index[c] until chf.endIndex[c]) {
                        val s = chf.spans[i]
                        if (RecastCommon.getCon(s, 0) != RecastConstants.RC_NOT_CONNECTED) {
                            // (-1,0)
                            val ax = x + RecastCommon.getDirOffsetX(0)
                            val ay = y + RecastCommon.getDirOffsetY(0)
                            val ai = chf.index[ax + ay * w] + RecastCommon.getCon(s, 0)
                            val asp = chf.spans[ai]
                            var nd = min(dist[ai] + 2, 255)
                            if (nd < dist[i]) dist[i] = nd

                            // (-1,-1)
                            if (RecastCommon.getCon(asp, 3) != RecastConstants.RC_NOT_CONNECTED) {
                                val aax = ax + RecastCommon.getDirOffsetX(3)
                                val aay = ay + RecastCommon.getDirOffsetY(3)
                                val aai = chf.index[aax + aay * w] + RecastCommon.getCon(asp, 3)
                                nd = min(dist[aai] + 3, 255)
                                if (nd < dist[i]) dist[i] = nd
                            }
                        }
                        if (RecastCommon.getCon(s, 3) != RecastConstants.RC_NOT_CONNECTED) {
                            // (0,-1)
                            val ax = x + RecastCommon.getDirOffsetX(3)
                            val ay = y + RecastCommon.getDirOffsetY(3)
                            val ai = chf.index[ax + ay * w] + RecastCommon.getCon(s, 3)
                            val asp = chf.spans[ai]
                            var nd = min(dist[ai] + 2, 255)
                            if (nd < dist[i]) dist[i] = nd

                            // (1,-1)
                            if (RecastCommon.getCon(asp, 2) != RecastConstants.RC_NOT_CONNECTED) {
                                val aax = ax + RecastCommon.getDirOffsetX(2)
                                val aay = ay + RecastCommon.getDirOffsetY(2)
                                val aai = chf.index[aax + aay * w] + RecastCommon.getCon(asp, 2)
                                nd = min(dist[aai] + 3, 255)
                                if (nd < dist[i]) dist[i] = nd
                            }
                        }
                    }
                }
            }

            // Pass 2
            for (y in h - 1 downTo 0) {
                for (x in w - 1 downTo 0) {
                    val c = x + y * w
                    for (i in chf.index[c] until chf.endIndex[c]) {
                        val s = chf.spans[i]
                        if (RecastCommon.getCon(s, 2) != RecastConstants.RC_NOT_CONNECTED) {
                            // (1,0)
                            val ax = x + RecastCommon.getDirOffsetX(2)
                            val ay = y + RecastCommon.getDirOffsetY(2)
                            val ai = chf.index[ax + ay * w] + RecastCommon.getCon(s, 2)
                            val asp = chf.spans[ai]
                            var nd = min(dist[ai] + 2, 255)
                            if (nd < dist[i]) dist[i] = nd

                            // (1,1)
                            if (RecastCommon.getCon(asp, 1) != RecastConstants.RC_NOT_CONNECTED) {
                                val aax = ax + RecastCommon.getDirOffsetX(1)
                                val aay = ay + RecastCommon.getDirOffsetY(1)
                                val aai = chf.index[aax + aay * w] + RecastCommon.getCon(asp, 1)
                                nd = min(dist[aai] + 3, 255)
                                if (nd < dist[i]) dist[i] = nd
                            }
                        }
                        if (RecastCommon.getCon(s, 1) != RecastConstants.RC_NOT_CONNECTED) {
                            // (0,1)
                            val ax = x + RecastCommon.getDirOffsetX(1)
                            val ay = y + RecastCommon.getDirOffsetY(1)
                            val ai = chf.index[ax + ay * w] + RecastCommon.getCon(s, 1)
                            val asp = chf.spans[ai]
                            var nd = min(dist[ai] + 2, 255)
                            if (nd < dist[i]) dist[i] = nd

                            // (-1,1)
                            if (RecastCommon.getCon(asp, 0) != RecastConstants.RC_NOT_CONNECTED) {
                                val aax = ax + RecastCommon.getDirOffsetX(0)
                                val aay = ay + RecastCommon.getDirOffsetY(0)
                                val aai = chf.index[aax + aay * w] + RecastCommon.getCon(asp, 0)
                                nd = min(dist[aai] + 3, 255)
                                if (nd < dist[i]) dist[i] = nd
                            }
                        }
                    }
                }
            }
            val thr = radius * 2
            for (i in 0 until chf.spanCount) if (dist[i] < thr) chf.areas[i] = RecastConstants.RC_NULL_AREA
            ctx?.stopTimer(TelemetryType.ERODE_AREA)
        }

        fun pointInPoly(vertices: FloatArray, p: Vector3f): Boolean {
            var c = false
            var i: Int
            var j: Int
            i = 0
            j = vertices.size - 3
            while (i < vertices.size) {
                if (vertices[i + 2] > p.z != vertices[j + 2] > p.z &&
                    p.x < (vertices[j] - vertices[i]) * (p.z - vertices[i + 2]) / (vertices[j + 2] - vertices[i + 2]) + vertices[i]
                ) {
                    c = !c
                }
                j = i
                i += 3
            }
            return c
        }

        /**
         * The value of spacial parameters are in world units.
         *
         * The y-values of the polygon vertices are ignored. So the polygon is effectively
         * projected onto the xz-plane at @p hmin, then extruded to @p hmax.
         *
         * @see rcCompactHeightfield, rcMedianFilterWalkableArea
         */
        fun markConvexPolyArea(
            ctx: Telemetry?,
            vertices: FloatArray,
            hmin: Float,
            hmax: Float,
            areaMod: AreaModification,
            chf: CompactHeightfield
        ) {
            ctx?.startTimer(TelemetryType.MARK_CONVEXPOLY_AREA)
            val bmin = Vector3f()
            val bmax = Vector3f()
            bmin.set(vertices, 0)
            bmax.set(vertices, 0)
            var i = 3
            while (i < vertices.size) {
                Vectors.min(bmin, vertices, i)
                Vectors.max(bmax, vertices, i)
                i += 3
            }
            bmin.y = hmin
            bmax.y = hmax
            var minx = ((bmin.x - chf.bmin.x) / chf.cellSize).toInt()
            val miny = ((bmin.y - chf.bmin.y) / chf.cellHeight).toInt()
            var minz = ((bmin.z - chf.bmin.z) / chf.cellSize).toInt()
            var maxx = ((bmax.x - chf.bmin.x) / chf.cellSize).toInt()
            val maxy = ((bmax.y - chf.bmin.y) / chf.cellHeight).toInt()
            var maxz = ((bmax.z - chf.bmin.z) / chf.cellSize).toInt()
            minx = max(minx, 0)
            maxx = min(maxx, chf.width - 1)
            minz = max(minz, 0)
            maxz = min(maxz, chf.height - 1)
            for (z in minz..maxz) {
                for (x in minx..maxx) {
                    val c = x + z * chf.width
                    for (j in chf.index[c] until chf.endIndex[c]) {
                        val s = chf.spans[j]
                        if (chf.areas[j] == RecastConstants.RC_NULL_AREA) {
                            continue
                        }
                        if (s.y in miny..maxy) {
                            val p = Vector3f()
                            p.x = chf.bmin.x + (x + 0.5f) * chf.cellSize
                            p.y = 0f
                            p.z = chf.bmin.z + (z + 0.5f) * chf.cellSize
                            if (pointInPoly(vertices, p)) {
                                chf.areas[j] = areaMod.apply(chf.areas[j])
                            }
                        }
                    }
                }
            }
            ctx?.stopTimer(TelemetryType.MARK_CONVEXPOLY_AREA)
        }
    }
}