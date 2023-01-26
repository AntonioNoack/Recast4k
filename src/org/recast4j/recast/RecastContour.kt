/*
Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
recast4j Copyright (c) 2015-2019 Piotr Piastucki piotr@jtilia.org

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

import org.recast4j.IntArrayList
import java.util.*

object RecastContour {
    private fun getCornerHeight(
        x: Int,
        y: Int,
        i: Int,
        dir: Int,
        chf: CompactHeightfield,
        isBorderVertex: BooleanArray
    ): Int {
        val s = chf.spans[i]
        var ch = s.y
        val dirp = dir + 1 and 0x3
        val regs = intArrayOf(0, 0, 0, 0)

        // Combine region and area codes in order to prevent
        // border vertices which are in between two areas to be removed.
        regs[0] = chf.spans[i].reg or (chf.areas[i] shl 16)
        if (RecastCommon.getCon(s, dir) != RecastConstants.RC_NOT_CONNECTED) {
            val ax = x + RecastCommon.getDirOffsetX(dir)
            val ay = y + RecastCommon.getDirOffsetY(dir)
            val ai = chf.cells[ax + ay * chf.width].index + RecastCommon.getCon(s, dir)
            val `as` = chf.spans[ai]
            ch = Math.max(ch, `as`.y)
            regs[1] = chf.spans[ai].reg or (chf.areas[ai] shl 16)
            if (RecastCommon.getCon(`as`, dirp) != RecastConstants.RC_NOT_CONNECTED) {
                val ax2 = ax + RecastCommon.getDirOffsetX(dirp)
                val ay2 = ay + RecastCommon.getDirOffsetY(dirp)
                val ai2 = chf.cells[ax2 + ay2 * chf.width].index + RecastCommon.getCon(`as`, dirp)
                val as2 = chf.spans[ai2]
                ch = Math.max(ch, as2.y)
                regs[2] = chf.spans[ai2].reg or (chf.areas[ai2] shl 16)
            }
        }
        if (RecastCommon.getCon(s, dirp) != RecastConstants.RC_NOT_CONNECTED) {
            val ax = x + RecastCommon.getDirOffsetX(dirp)
            val ay = y + RecastCommon.getDirOffsetY(dirp)
            val ai = chf.cells[ax + ay * chf.width].index + RecastCommon.getCon(s, dirp)
            val `as` = chf.spans[ai]
            ch = Math.max(ch, `as`.y)
            regs[3] = chf.spans[ai].reg or (chf.areas[ai] shl 16)
            if (RecastCommon.getCon(`as`, dir) != RecastConstants.RC_NOT_CONNECTED) {
                val ax2 = ax + RecastCommon.getDirOffsetX(dir)
                val ay2 = ay + RecastCommon.getDirOffsetY(dir)
                val ai2 = chf.cells[ax2 + ay2 * chf.width].index + RecastCommon.getCon(`as`, dir)
                val as2 = chf.spans[ai2]
                ch = Math.max(ch, as2.y)
                regs[2] = chf.spans[ai2].reg or (chf.areas[ai2] shl 16)
            }
        }

        // Check if the vertex is special edge vertex, these vertices will be removed later.
        for (a in 0..3) {
            val b = a + 1 and 0x3
            val c = a + 2 and 0x3
            val d = a + 3 and 0x3

            // The vertex is a border vertex there are two same exterior cells in a row,
            // followed by two interior cells and none of the regions are out of bounds.
            val twoSameExts = regs[a] and regs[b] and RecastConstants.RC_BORDER_REG != 0 && regs[a] == regs[b]
            val twoInts = regs[c] or regs[d] and RecastConstants.RC_BORDER_REG == 0
            val intsSameArea = regs[c] shr 16 == regs[d] shr 16
            val noZeros = regs[a] != 0 && regs[b] != 0 && regs[c] != 0 && regs[d] != 0
            if (twoSameExts && twoInts && intsSameArea && noZeros) {
                isBorderVertex[0] = true
                break
            }
        }
        return ch
    }

    private fun walkContour(
        x: Int,
        y: Int,
        i: Int,
        chf: CompactHeightfield,
        flags: IntArray,
        points: IntArrayList
    ) {
        // Choose the first non-connected edge
        var x = x
        var y = y
        var i = i
        var dir = 0
        while (flags[i] and (1 shl dir) == 0) dir++
        val startDir = dir
        val starti = i
        val area = chf.areas[i]
        var iter = 0
        while (++iter < 40000) {
            if (flags[i] and (1 shl dir) != 0) {
                // Choose the edge corner
                val isBorderVertex = booleanArrayOf(false)
                var isAreaBorder = false
                var px = x
                val py = getCornerHeight(x, y, i, dir, chf, isBorderVertex)
                var pz = y
                when (dir) {
                    0 -> pz++
                    1 -> {
                        px++
                        pz++
                    }
                    2 -> px++
                }
                var r = 0
                val s = chf.spans[i]
                if (RecastCommon.getCon(s, dir) != RecastConstants.RC_NOT_CONNECTED) {
                    val ax = x + RecastCommon.getDirOffsetX(dir)
                    val ay = y + RecastCommon.getDirOffsetY(dir)
                    val ai = chf.cells[ax + ay * chf.width].index + RecastCommon.getCon(s, dir)
                    r = chf.spans[ai].reg
                    if (area != chf.areas[ai]) isAreaBorder = true
                }
                if (isBorderVertex[0]) r = r or RecastConstants.RC_BORDER_VERTEX
                if (isAreaBorder) r = r or RecastConstants.RC_AREA_BORDER
                points.add(px)
                points.add(py)
                points.add(pz)
                points.add(r)
                flags[i] = flags[i] and (1 shl dir).inv() // Remove visited edges
                dir = dir + 1 and 0x3 // Rotate CW
            } else {
                var ni = -1
                val nx = x + RecastCommon.getDirOffsetX(dir)
                val ny = y + RecastCommon.getDirOffsetY(dir)
                val s = chf.spans[i]
                if (RecastCommon.getCon(s, dir) != RecastConstants.RC_NOT_CONNECTED) {
                    val nc = chf.cells[nx + ny * chf.width]
                    ni = nc.index + RecastCommon.getCon(s, dir)
                }
                if (ni == -1) {
                    // Should not happen.
                    return
                }
                x = nx
                y = ny
                i = ni
                dir = dir + 3 and 0x3 // Rotate CCW
            }
            if (starti == i && startDir == dir) {
                break
            }
        }
    }

    private fun distancePtSeg(x: Int, z: Int, px: Int, pz: Int, qx: Int, qz: Int): Float {
        val pqx = (qx - px).toFloat()
        val pqz = (qz - pz).toFloat()
        var dx = (x - px).toFloat()
        var dz = (z - pz).toFloat()
        val d = pqx * pqx + pqz * pqz
        var t = pqx * dx + pqz * dz
        if (d > 0) t /= d
        if (t < 0) t = 0f else if (t > 1) t = 1f
        dx = px + t * pqx - x
        dz = pz + t * pqz - z
        return dx * dx + dz * dz
    }

    private fun simplifyContour(
        points: IntArrayList, simplified: IntArrayList, maxError: Float, maxEdgeLen: Int,
        buildFlags: Int
    ) {
        // Add initial points.
        var hasConnections = false
        run {
            var i = 0
            while (i < points.size) {
                if (points[i + 3] and RecastConstants.RC_CONTOUR_REG_MASK != 0) {
                    hasConnections = true
                    break
                }
                i += 4
            }
        }
        if (hasConnections) {
            // The contour has some portals to other regions.
            // Add a new point to every location where the region changes.
            var i = 0
            val ni = points.size / 4
            while (i < ni) {
                val ii = (i + 1) % ni
                val differentRegs = points[i * 4 + 3] and RecastConstants.RC_CONTOUR_REG_MASK != (points[ii * 4 + 3]
                        and RecastConstants.RC_CONTOUR_REG_MASK)
                val areaBorders = points[i * 4 + 3] and RecastConstants.RC_AREA_BORDER != (points[ii * 4 + 3]
                        and RecastConstants.RC_AREA_BORDER)
                if (differentRegs || areaBorders) {
                    simplified.add(points[i * 4])
                    simplified.add(points[i * 4 + 1])
                    simplified.add(points[i * 4 + 2])
                    simplified.add(i)
                }
                ++i
            }
        }
        if (simplified.size == 0) {
            // If there is no connections at all,
            // create some initial points for the simplification process.
            // Find lower-left and upper-right vertices of the contour.
            var llx = points[0]
            var lly = points[1]
            var llz = points[2]
            var lli = 0
            var urx = points[0]
            var ury = points[1]
            var urz = points[2]
            var uri = 0
            var i = 0
            while (i < points.size) {
                val x = points[i]
                val y = points[i + 1]
                val z = points[i + 2]
                if (x < llx || x == llx && z < llz) {
                    llx = x
                    lly = y
                    llz = z
                    lli = i / 4
                }
                if (x > urx || x == urx && z > urz) {
                    urx = x
                    ury = y
                    urz = z
                    uri = i / 4
                }
                i += 4
            }
            simplified.add(llx)
            simplified.add(lly)
            simplified.add(llz)
            simplified.add(lli)
            simplified.add(urx)
            simplified.add(ury)
            simplified.add(urz)
            simplified.add(uri)
        }
        // Add points until all raw points are within
        // error tolerance to the simplified shape.
        val pn = points.size / 4
        run {
            var i = 0
            while (i < simplified.size / 4) {
                val ii = (i + 1) % (simplified.size / 4)
                var ax = simplified[i * 4]
                var az = simplified[i * 4 + 2]
                val ai = simplified[i * 4 + 3]
                var bx = simplified[ii * 4]
                var bz = simplified[ii * 4 + 2]
                val bi = simplified[ii * 4 + 3]

                // Find maximum deviation from the segment.
                var maxd = 0f
                var maxi = -1
                var ci: Int
                var cinc: Int
                var endi: Int

                // Traverse the segment in lexilogical order so that the
                // max deviation is calculated similarly when traversing
                // opposite segments.
                if (bx > ax || bx == ax && bz > az) {
                    cinc = 1
                    ci = (ai + cinc) % pn
                    endi = bi
                } else {
                    cinc = pn - 1
                    ci = (bi + cinc) % pn
                    endi = ai
                    var temp = ax
                    ax = bx
                    bx = temp
                    temp = az
                    az = bz
                    bz = temp
                }
                // Tessellate only outer edges or edges between areas.
                if (points[ci * 4 + 3] and RecastConstants.RC_CONTOUR_REG_MASK == 0 || points[ci * 4 + 3] and RecastConstants.RC_AREA_BORDER != 0) {
                    while (ci != endi) {
                        val d = distancePtSeg(points[ci * 4], points[ci * 4 + 2], ax, az, bx, bz)
                        if (d > maxd) {
                            maxd = d
                            maxi = ci
                        }
                        ci = (ci + cinc) % pn
                    }
                }
                // If the max deviation is larger than accepted error,
                // add new point, else continue to next segment.
                if (maxi != -1 && maxd > maxError * maxError) {
                    // Add the point.
                    simplified.add((i + 1) * 4, points[maxi * 4])
                    simplified.add((i + 1) * 4 + 1, points[maxi * 4 + 1])
                    simplified.add((i + 1) * 4 + 2, points[maxi * 4 + 2])
                    simplified.add((i + 1) * 4 + 3, maxi)
                } else {
                    ++i
                }
            }
        }
        // Split too long edges.
        if (maxEdgeLen > 0 && buildFlags and (RecastConstants.RC_CONTOUR_TESS_WALL_EDGES or RecastConstants.RC_CONTOUR_TESS_AREA_EDGES) != 0) {
            var i = 0
            while (i < simplified.size / 4) {
                val ii = (i + 1) % (simplified.size / 4)
                val ax = simplified[i * 4]
                val az = simplified[i * 4 + 2]
                val ai = simplified[i * 4 + 3]
                val bx = simplified[ii * 4]
                val bz = simplified[ii * 4 + 2]
                val bi = simplified[ii * 4 + 3]

                // Find maximum deviation from the segment.
                var maxi = -1
                val ci = (ai + 1) % pn

                // Tessellate only outer edges or edges between areas.
                var tess = (buildFlags and RecastConstants.RC_CONTOUR_TESS_WALL_EDGES != 0
                        && points[ci * 4 + 3] and RecastConstants.RC_CONTOUR_REG_MASK == 0)
                // Wall edges.
                // Edges between areas.
                if (buildFlags and RecastConstants.RC_CONTOUR_TESS_AREA_EDGES != 0 && points[ci * 4 + 3] and RecastConstants.RC_AREA_BORDER != 0) tess =
                    true
                if (tess) {
                    val dx = bx - ax
                    val dz = bz - az
                    if (dx * dx + dz * dz > maxEdgeLen * maxEdgeLen) {
                        // Round based on the segments in lexilogical order so that the
                        // max tesselation is consistent regardles in which direction
                        // segments are traversed.
                        val n = if (bi < ai) bi + pn - ai else bi - ai
                        if (n > 1) {
                            maxi = if ((bx > ax || bx == ax) && bz > az) (ai + n / 2) % pn else (ai + (n + 1) / 2) % pn
                        }
                    }
                }

                // If the max deviation is larger than accepted error,
                // add new point, else continue to next segment.
                if (maxi != -1) {
                    // Add the point.
                    simplified.add((i + 1) * 4, points[maxi * 4])
                    simplified.add((i + 1) * 4 + 1, points[maxi * 4 + 1])
                    simplified.add((i + 1) * 4 + 2, points[maxi * 4 + 2])
                    simplified.add((i + 1) * 4 + 3, maxi)
                } else {
                    ++i
                }
            }
        }
        for (i in 0 until simplified.size / 4) {
            // The edge vertex flag is take from the current raw point,
            // and the neighbour region is take from the next raw point.
            val ai = (simplified[i * 4 + 3] + 1) % pn
            val bi = simplified[i * 4 + 3]
            simplified[i * 4 + 3] =
                (points[ai * 4 + 3] and (RecastConstants.RC_CONTOUR_REG_MASK or RecastConstants.RC_AREA_BORDER)
                        or (points[bi * 4 + 3] and RecastConstants.RC_BORDER_VERTEX))
        }
    }

    private fun calcAreaOfPolygon2D(vertices: IntArray, nvertices: Int): Int {
        var area = 0
        var i = 0
        var j = nvertices - 1
        while (i < nvertices) {
            val vi = i * 4
            val vj = j * 4
            area += vertices[vi] * vertices[vj + 2] - vertices[vj] * vertices[vi + 2]
            j = i++
        }
        return (area + 1) / 2
    }

    private fun intersectSegCountour(
        d0: Int, d1: Int, i: Int, n: Int, vertices: IntArray, d0vertices: IntArray,
        d1vertices: IntArray
    ): Boolean {
        // For each edge (k,k+1) of P
        var d0 = d0
        var d1 = d1
        val pvertices = IntArray(4 * 4)
        for (g in 0..3) {
            pvertices[g] = d0vertices[d0 + g]
            pvertices[4 + g] = d1vertices[d1 + g]
        }
        d0 = 0
        d1 = 4
        for (k in 0 until n) {
            val k1 = RecastMesh.next(k, n)
            // Skip edges incident to i.
            if (i == k || i == k1) continue
            var p0 = k * 4
            var p1 = k1 * 4
            for (g in 0..3) {
                pvertices[8 + g] = vertices[p0 + g]
                pvertices[12 + g] = vertices[p1 + g]
            }
            p0 = 8
            p1 = 12
            if (RecastMesh.vequal(pvertices, d0, p0) || RecastMesh.vequal(pvertices, d1, p0)
                || RecastMesh.vequal(pvertices, d0, p1) || RecastMesh.vequal(pvertices, d1, p1)
            ) continue
            if (RecastMesh.intersect(pvertices, d0, d1, p0, p1)) return true
        }
        return false
    }

    private fun inCone(i: Int, n: Int, vertices: IntArray, pj: Int, vertpj: IntArray): Boolean {
        var pj = pj
        var pi = i * 4
        var pi1 = RecastMesh.next(i, n) * 4
        var pin1 = RecastMesh.prev(i, n) * 4
        val pvertices = IntArray(4 * 4)
        for (g in 0..3) {
            pvertices[g] = vertices[pi + g]
            pvertices[4 + g] = vertices[pi1 + g]
            pvertices[8 + g] = vertices[pin1 + g]
            pvertices[12 + g] = vertpj[pj + g]
        }
        pi = 0
        pi1 = 4
        pin1 = 8
        pj = 12
        // If P[i] is a convex vertex [ i+1 left or on (i-1,i) ].
        return if (RecastMesh.leftOn(pvertices, pin1, pi, pi1)) RecastMesh.left(
            pvertices,
            pi,
            pj,
            pin1
        ) && RecastMesh.left(
            pvertices,
            pj,
            pi,
            pi1
        ) else !(RecastMesh.leftOn(
            pvertices,
            pi,
            pj,
            pi1
        ) && RecastMesh.leftOn(pvertices, pj, pi, pin1))
        // Assume (i-1,i,i+1) not collinear.
        // else P[i] is reflex.
    }

    private fun removeDegenerateSegments(simplified: IntArrayList) {
        // Remove adjacent vertices which are equal on xz-plane,
        // or else the triangulator will get confused.
        var npts = simplified.size / 4
        for (i in 0 until npts) {
            val ni = RecastMesh.next(i, npts)

            // if (vequal(&simplified[i*4], &simplified[ni*4]))
            if (simplified[i * 4] == simplified[ni * 4] && simplified[i * 4 + 2] == simplified[ni * 4 + 2]) {
                // Degenerate segment, remove.
                simplified.removeAt(i * 4)
                simplified.removeAt(i * 4)
                simplified.removeAt(i * 4)
                simplified.removeAt(i * 4)
                npts--
            }
        }
    }

    private fun mergeContours(ca: Contour?, cb: Contour?, ia: Int, ib: Int) {
        val maxVertices = ca!!.numVertices + cb!!.numVertices + 2
        val vertices = IntArray(maxVertices * 4)
        var nv = 0

        // Copy contour A.
        for (i in 0..ca.numVertices) {
            val dst = nv * 4
            val src = (ia + i) % ca.numVertices * 4
            vertices[dst] = ca.vertices!![src]
            vertices[dst + 1] = ca.vertices!![src + 1]
            vertices[dst + 2] = ca.vertices!![src + 2]
            vertices[dst + 3] = ca.vertices!![src + 3]
            nv++
        }

        // Copy contour B
        for (i in 0..cb.numVertices) {
            val dst = nv * 4
            val src = (ib + i) % cb.numVertices * 4
            vertices[dst] = cb.vertices!![src]
            vertices[dst + 1] = cb.vertices!![src + 1]
            vertices[dst + 2] = cb.vertices!![src + 2]
            vertices[dst + 3] = cb.vertices!![src + 3]
            nv++
        }
        ca.vertices = vertices
        ca.numVertices = nv
        cb.vertices = null
        cb.numVertices = 0
    }

    // Finds the lowest leftmost vertex of a contour.
    private fun findLeftMostVertex(contour: Contour): IntArray {
        var minx = contour.vertices!![0]
        var minz = contour.vertices!![2]
        var leftmost = 0
        for (i in 1 until contour.numVertices) {
            val x = contour.vertices!![i * 4]
            val z = contour.vertices!![i * 4 + 2]
            if (x < minx || x == minx && z < minz) {
                minx = x
                minz = z
                leftmost = i
            }
        }
        return intArrayOf(minx, minz, leftmost)
    }

    private fun mergeRegionHoles(ctx: Telemetry?, region: ContourRegion?) {
        // Sort holes from left to right.
        for (i in 0 until region!!.nholes) {
            val minleft = findLeftMostVertex(region.holes[i].contour!!)
            region.holes[i].minx = minleft[0]
            region.holes[i].minz = minleft[1]
            region.holes[i].leftmost = minleft[2]
        }
        region.holes.sortWith { a, b ->
            a!!.minx.compareTo(b!!.minx) * 2 +
                    a.minz.compareTo(b.minz)
        }
        var maxVertices = region.outline!!.numVertices
        for (i in 0 until region.nholes) maxVertices += region.holes[i].contour!!.numVertices
        val diags = Array(maxVertices) { PotentialDiagonal() }
        val outline = region.outline

        // Merge holes into the outline one by one.
        for (i in 0 until region.nholes) {
            val hole = region.holes[i].contour
            var index = -1
            var bestVertex = region.holes[i].leftmost
            for (iter in 0 until hole!!.numVertices) {
                // Find potential diagonals.
                // The 'best' vertex must be in the cone described by 3 cosequtive vertices of the outline.
                // ..o j-1
                // |
                // | * best
                // |
                // j o-----o j+1
                // :
                var ndiags = 0
                val corner = bestVertex * 4
                for (j in 0 until outline!!.numVertices) {
                    val vs = outline.vertices!!
                    val hs = hole.vertices!!
                    if (inCone(j, outline.numVertices, vs, corner, hs)) {
                        val dx = vs[j * 4] - hs[corner]
                        val dz = vs[j * 4 + 2] - hs[corner + 2]
                        diags[ndiags].vert = j
                        diags[ndiags].dist = dx * dx + dz * dz
                        ndiags++
                    }
                }
                // Sort potential diagonals by distance, we want to make the connection as short as possible.
                Arrays.sort(diags, 0, ndiags, CompareDiagDist())

                // Find a diagonal that is not intersecting the outline not the remaining holes.
                for (j in 0 until ndiags) {
                    val pt = diags[j].vert * 4
                    var intersect = intersectSegCountour(
                        pt, corner, diags[j].vert, outline.numVertices, outline.vertices!!,
                        outline.vertices!!, hole.vertices!!
                    )
                    var k = i
                    while (k < region.nholes && !intersect) {
                        intersect = intersectSegCountour(
                            pt, corner, -1, region.holes[k].contour!!.numVertices,
                            region.holes[k].contour!!.vertices!!, outline.vertices!!, hole.vertices!!
                        )
                        k++
                    }
                    if (!intersect) {
                        index = diags[j].vert
                        break
                    }
                }
                // If found non-intersecting diagonal, stop looking.
                if (index != -1) break
                // All the potential diagonals for the current vertex were intersecting, try next vertex.
                bestVertex = (bestVertex + 1) % hole.numVertices
            }
            if (index == -1) {
                ctx!!.warn("mergeHoles: Failed to find merge points for")
                continue
            }
            mergeContours(region.outline, hole, index, bestVertex)
        }
    }

    /// @par
    ///
    /// The raw contours will match the region outlines exactly. The @p maxError and @p maxEdgeLen
    /// parameters control how closely the simplified contours will match the raw contours.
    ///
    /// Simplified contours are generated such that the vertices for portals between areas match up.
    /// (They are considered mandatory vertices.)
    ///
    /// Setting @p maxEdgeLength to zero will disabled the edge length feature.
    ///
    /// See the #rcConfig documentation for more information on the configuration parameters.
    ///
    /// @see rcAllocContourSet, rcCompactHeightfield, rcContourSet, rcConfig
    fun buildContours(
        ctx: Telemetry?,
        chf: CompactHeightfield,
        maxError: Float,
        maxEdgeLen: Int,
        buildFlags: Int
    ): ContourSet {
        val w = chf.width
        val h = chf.height
        val borderSize = chf.borderSize
        val cset = ContourSet()
        ctx?.startTimer("CONTOURS")
        cset.bmin.set(chf.bmin)
        cset.bmax.set(chf.bmax)
        if (borderSize > 0) {
            // If the heightfield was build with bordersize, remove the offset.
            val pad = borderSize * chf.cellSize
            cset.bmin.add(pad, 0f, pad, cset.bmin)
            cset.bmax.sub(pad, 0f, pad, cset.bmin)
        }
        cset.cellSize = chf.cellSize
        cset.cellHeight = chf.cellHeight
        cset.width = chf.width - chf.borderSize * 2
        cset.height = chf.height - chf.borderSize * 2
        cset.borderSize = chf.borderSize
        cset.maxError = maxError
        val flags = IntArray(chf.spanCount)
        ctx?.startTimer("CONTOURS_TRACE")

        // Mark boundaries.
        for (y in 0 until h) {
            for (x in 0 until w) {
                val c = chf.cells[x + y * w]
                var i = c.index
                val ni = c.index + c.count
                while (i < ni) {
                    var res = 0
                    val s = chf.spans[i]
                    if (chf.spans[i].reg == 0 || chf.spans[i].reg and RecastConstants.RC_BORDER_REG != 0) {
                        flags[i] = 0
                        ++i
                        continue
                    }
                    for (dir in 0..3) {
                        var r = 0
                        if (RecastCommon.getCon(s, dir) != RecastConstants.RC_NOT_CONNECTED) {
                            val ax = x + RecastCommon.getDirOffsetX(dir)
                            val ay = y + RecastCommon.getDirOffsetY(dir)
                            val ai = chf.cells[ax + ay * w].index + RecastCommon.getCon(s, dir)
                            r = chf.spans[ai].reg
                        }
                        if (r == chf.spans[i].reg) res = res or (1 shl dir)
                    }
                    flags[i] = res xor 0xf // Inverse, mark non connected edges.
                    ++i
                }
            }
        }
        ctx?.stopTimer("CONTOURS_TRACE")
        val vertices= IntArrayList(256)
        val simplified= IntArrayList(64)
        for (y in 0 until h) {
            for (x in 0 until w) {
                val c = chf.cells[x + y * w]
                var i = c.index
                val ni = c.index + c.count
                while (i < ni) {
                    if (flags[i] == 0 || flags[i] == 0xf) {
                        flags[i] = 0
                        ++i
                        continue
                    }
                    val reg = chf.spans[i].reg
                    if (reg == 0 || reg and RecastConstants.RC_BORDER_REG != 0) {
                        ++i
                        continue
                    }
                    val area = chf.areas[i]
                    vertices.clear()
                    simplified.clear()
                    ctx?.startTimer("CONTOURS_WALK")
                    walkContour(x, y, i, chf, flags, vertices)
                    ctx?.stopTimer("CONTOURS_WALK")
                    ctx?.startTimer("CONTOURS_SIMPLIFY")
                    simplifyContour(vertices, simplified, maxError, maxEdgeLen, buildFlags)
                    removeDegenerateSegments(simplified)
                    ctx?.stopTimer("CONTOURS_SIMPLIFY")

                    // Store region->contour remap info.
                    // Create contour.
                    if (simplified.size / 4 >= 3) {
                        val cont = Contour()
                        cset.contours.add(cont)
                        cont.numVertices = simplified.size / 4
                        cont.vertices = IntArray(simplified.size)
                        val vs = cont.vertices!!
                        for (l in vs.indices) {
                            vs[l] = simplified[l]
                        }
                        if (borderSize > 0) {
                            // If the heightfield was build with bordersize, remove the offset.
                            for (j in 0 until cont.numVertices) {
                                vs[j * 4] -= borderSize
                                vs[j * 4 + 2] -= borderSize
                            }
                        }
                        cont.numRawVertices = vertices.size / 4
                        cont.rawVertices = IntArray(vertices.size)
                        for (l in cont.rawVertices.indices) {
                            cont.rawVertices[l] = vertices[l]
                        }
                        if (borderSize > 0) {
                            // If the heightfield was build with bordersize, remove the offset.
                            for (j in 0 until cont.numRawVertices) {
                                cont.rawVertices[j * 4] -= borderSize
                                cont.rawVertices[j * 4 + 2] -= borderSize
                            }
                        }
                        cont.reg = reg
                        cont.area = area
                    }
                    ++i
                }
            }
        }

        // Merge holes if needed.
        if (cset.contours.size > 0) {
            // Calculate winding of all polygons.
            val winding = IntArray(cset.contours.size)
            var nholes = 0
            for (i in cset.contours.indices) {
                val cont = cset.contours[i]
                // If the contour is wound backwards, it is a hole.
                winding[i] = if (calcAreaOfPolygon2D(cont.vertices!!, cont.numVertices) < 0) -1 else 1
                if (winding[i] < 0) nholes++
            }
            if (nholes > 0) {
                // Collect outline contour and holes contours per region.
                // We assume that there is one outline and multiple holes.
                val nregions = chf.maxRegions + 1
                val regions = Array(nregions) { ContourRegion() }
                for (i in cset.contours.indices) {
                    val cont = cset.contours[i]
                    // Positively would contours are outlines, negative holes.
                    if (winding[i] > 0) {
                        if (regions[cont.reg].outline != null) {
                            throw RuntimeException("rcBuildContours: Multiple outlines for region ${cont.reg}.")
                        }
                        regions[cont.reg].outline = cont
                    } else {
                        regions[cont.reg].nholes++
                    }
                }
                for (i in 0 until nregions) {
                    if (regions[i].nholes > 0) {
                        regions[i].holes = Array(regions[i].nholes) { ContourHole() }
                        regions[i].nholes = 0
                    }
                }
                for (i in cset.contours.indices) {
                    val cont = cset.contours[i]
                    val reg = regions[cont.reg]
                    if (winding[i] < 0) reg.holes[reg.nholes++].contour = cont
                }

                // Finally merge each regions holes into the outline.
                for (i in 0 until nregions) {
                    val reg = regions[i]
                    if (reg.nholes == 0) continue
                    if (reg.outline != null) {
                        mergeRegionHoles(ctx, reg)
                    } else {
                        // The region does not have an outline.
                        // This can happen if the contour becomes self-overlapping because of
                        // too aggressive simplification settings.
                        throw RuntimeException(
                            "rcBuildContours: Bad outline for region " + i
                                    + ", contour simplification is likely too aggressive."
                        )
                    }
                }
            }
        }
        ctx?.stopTimer("CONTOURS")
        return cset
    }

    private class ContourRegion {
        var outline: Contour? = null
        lateinit var holes: Array<ContourHole>
        var nholes = 0
    }

    private class ContourHole {
        var leftmost = 0
        var minx = 0
        var minz = 0
        var contour: Contour? = null
    }

    private class PotentialDiagonal {
        var dist = 0
        var vert = 0
    }

    private class CompareDiagDist : Comparator<PotentialDiagonal> {
        override fun compare(va: PotentialDiagonal, vb: PotentialDiagonal): Int {
            return va.dist.toFloat().compareTo(vb.dist.toFloat())
        }
    }
}