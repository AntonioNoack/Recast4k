package org.recast4j.recast

import org.joml.Vector3f
import org.recast4j.IntArrayList
import org.recast4j.recast.RecastMeshDetail.EV_UNDEF
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

object RecastMeshDetail0 {

    class HeightPatch(
        @JvmField
        val data: IntArray
    ) {
        @JvmField
        var xmin = 0

        @JvmField
        var ymin = 0

        @JvmField
        var width = 0

        @JvmField
        var height = 0
    }

    @JvmStatic
    fun vdot2(a: Vector3f, b: Vector3f): Float {
        return a.x * b.x + a.z * b.z
    }

    @JvmStatic
    fun vdistSq2(vertices: FloatArray, p: Int, q: Int): Float {
        val dx = vertices[q] - vertices[p]
        val dy = vertices[q + 2] - vertices[p + 2]
        return dx * dx + dy * dy
    }

    @JvmStatic
    fun vdist2(vertices: FloatArray, p: Int, q: Int): Float {
        return sqrt(vdistSq2(vertices, p, q))
    }

    @JvmStatic
    fun vdistSq2(p: Vector3f, q: Vector3f): Float {
        val dx = q.x - p.x
        val dy = q.z - p.z
        return dx * dx + dy * dy
    }

    @JvmStatic
    fun vdist2(p: Vector3f, q: Vector3f): Float {
        return sqrt(vdistSq2(p, q))
    }

    @JvmStatic
    fun vdistSq2(p: Vector3f, vertices: FloatArray, q: Int): Float {
        val dx = vertices[q] - p.x
        val dy = vertices[q + 2] - p.z
        return dx * dx + dy * dy
    }

    @JvmStatic
    fun vdist2(p: Vector3f, vertices: FloatArray, q: Int): Float {
        return sqrt(vdistSq2(p, vertices, q))
    }

    @JvmStatic
    fun vcross2(vertices: FloatArray, p1: Int, p2: Int, p3: Int): Float {
        val u1 = vertices[p2] - vertices[p1]
        val v1 = vertices[p2 + 2] - vertices[p1 + 2]
        val u2 = vertices[p3] - vertices[p1]
        val v2 = vertices[p3 + 2] - vertices[p1 + 2]
        return u1 * v2 - v1 * u2
    }

    @JvmStatic
    fun vcross2(p1: Vector3f, p2: Vector3f, p3: Vector3f): Float {
        val u1 = p2.x - p1.x
        val v1 = p2.z - p1.z
        val u2 = p3.x - p1.x
        val v2 = p3.z - p1.z
        return u1 * v2 - v1 * u2
    }

    @JvmStatic
    fun sub(dst: Vector3f, data: FloatArray, p1: Int, p2: Int) {
        dst.set(data[p1] - data[p2], data[p1 + 1] - data[p2 + 1], data[p1 + 2] - data[p2 + 2])
    }

    @JvmStatic
    fun sub(dst: Vector3f, a: Vector3f, b: FloatArray, bi: Int) {
        dst.set(a.x - b[bi], a.y - b[bi + 1], a.z - b[bi + 2])
    }

    @JvmStatic
    fun add(dst: Vector3f, a: Vector3f, b: FloatArray, bi: Int) {
        dst.set(a.x - b[bi], a.y - b[bi + 1], a.z - b[bi + 2])
    }

    @JvmStatic
    fun copy(dst: Vector3f, a: FloatArray, ai: Int) {
        dst.set(a[ai], a[ai + 1], a[ai + 2])
    }

    @JvmStatic
    fun copy(dst: FloatArray, di: Int, a: FloatArray, ai: Int) {
        dst[di] = a[ai]
        dst[di + 1] = a[ai + 1]
        dst[di + 2] = a[ai + 2]
    }

    @JvmStatic
    fun copy(dst: FloatArray, di: Int, a: Vector3f) {
        dst[di] = a.x
        dst[di + 1] = a.y
        dst[di + 2] = a.z
    }

    @JvmStatic
    fun findEdge(edges: IntArrayList, s: Int, t: Int): Int {
        var e = 0
        while (e < edges.size) {
            if ((edges[e] == s && edges[e + 1] == t) || (edges[e] == t && edges[e + 1] == s)) {
                return e shr 2
            }
            e += 4
        }
        return RecastMeshDetail.EV_UNDEF
    }

    @JvmStatic
    fun addEdge(edges: IntArrayList, maxEdges: Int, s: Int, t: Int, l: Int, r: Int) {
        if (edges.size shr 2 >= maxEdges) {
            throw RuntimeException("addEdge: Too many edges (" + (edges.size / 4) + "/" + maxEdges + ").")
        }

        // Add edge if not already in the triangulation.
        val e = findEdge(edges, s, t)
        if (e == RecastMeshDetail.EV_UNDEF) {
            edges.add(s)
            edges.add(t)
            edges.add(l)
            edges.add(r)
        }
    }

    @JvmStatic
    fun getEdgeFlags(vertices: FloatArray, va: Int, vb: Int, vpoly: FloatArray, npoly: Int): Int {
        // The flag returned by this function matches getDetailTriEdgeFlags in Detour.
        // Figure out if edge (va,vb) is part of the polygon boundary.
        val thrSqr = 0.001f * 0.001f
        var i = 0
        var j = npoly - 1
        while (i < npoly) {
            if (distancePtSeg2d(vertices, va, vpoly, j * 3, i * 3) < thrSqr
                && distancePtSeg2d(vertices, vb, vpoly, j * 3, i * 3) < thrSqr
            ) return 1
            j = i++
        }
        return 0
    }

    @JvmStatic
    fun getTriFlags(vertices: FloatArray, va: Int, vb: Int, vc: Int, vpoly: FloatArray, npoly: Int): Int {
        var flags = getEdgeFlags(vertices, va, vb, vpoly, npoly)
        flags = flags or (getEdgeFlags(vertices, vb, vc, vpoly, npoly) shl 2)
        flags = flags or (getEdgeFlags(vertices, vc, va, vpoly, npoly) shl 4)
        return flags
    }

    @JvmStatic
    fun distToTriMesh(p: Vector3f, vertices: FloatArray, tris: IntArrayList, ntris: Int): Float {
        var dmin = Float.MAX_VALUE
        for (i in 0 until ntris) {
            val va = tris[i * 4] * 3
            val vb = tris[i * 4 + 1] * 3
            val vc = tris[i * 4 + 2] * 3
            val d = distPtTri(p, vertices, va, vb, vc)
            if (d < dmin) {
                dmin = d
            }
        }
        return if (dmin == Float.MAX_VALUE) -1f else dmin
    }

    @JvmStatic
    fun distToPoly(nvert: Int, vertices: FloatArray, p: Vector3f): Float {
        var dmin = Float.MAX_VALUE
        var c = false
        val px = p.x
        val pz = p.z
        var j = nvert - 1
        for (i in 0 until nvert) {
            val vi = i * 3
            val vj = j * 3
            if (((vertices[vi + 2] > pz) != (vertices[vj + 2] > pz)) &&
                (px < (vertices[vj] - vertices[vi]) * (pz - vertices[vi + 2]) / (vertices[vj + 2] - vertices[vi + 2]) + vertices[vi])
            ) c = !c
            dmin = min(dmin, distancePtSeg2d(p, vertices, vj, vi))
            j = i
        }
        return if (c) -dmin else dmin
    }

    @JvmStatic
    fun getJitterX(i: Int): Float {
        return (i * -0x72594cbd and 0xffff) / 65535.0f * 2.0f - 1.0f
    }

    @JvmStatic
    fun getJitterY(i: Int): Float {
        return (i * -0x27e9c7bf and 0xffff) / 65535.0f * 2.0f - 1.0f
    }

    @JvmStatic
    fun min(dst: Vector3f, a: FloatArray, ai: Int) {
        dst.set(min(dst.x, a[ai]), min(dst.y, a[ai + 1]), min(dst.z, a[ai + 2]))
    }

    @JvmStatic
    fun max(dst: Vector3f, a: FloatArray, ai: Int) {
        dst.set(
            max(dst.x, a[ai]),
            max(dst.y, a[ai + 1]),
            max(dst.z, a[ai + 2])
        )
    }

    @JvmStatic
    fun distPtTri(p: Vector3f, vertices: FloatArray, a: Int, b: Int, c: Int): Float {
        val v0 = Vector3f()
        val v1 = Vector3f()
        val v2 = Vector3f()
        sub(v0, vertices, c, a)
        sub(v1, vertices, b, a)
        sub(v2, p, vertices, a)
        val dot00 = vdot2(v0, v0)
        val dot01 = vdot2(v0, v1)
        val dot02 = vdot2(v0, v2)
        val dot11 = vdot2(v1, v1)
        val dot12 = vdot2(v1, v2)

        // Compute barycentric coordinates
        val invDenom = 1.0f / (dot00 * dot11 - dot01 * dot01)
        val u = (dot11 * dot02 - dot01 * dot12) * invDenom
        val v = (dot00 * dot12 - dot01 * dot02) * invDenom

        // If point lies inside the triangle, return interpolated y-coord.
        val EPS = 1e-4f
        if (u >= -EPS && v >= -EPS && (u + v) <= 1 + EPS) {
            val y = vertices[a + 1] + v0.y * u + v1.y * v
            return abs(y - p.y)
        }
        return Float.MAX_VALUE
    }

    /** Calculate minimum extend of the polygon. */
    @JvmStatic
    fun polyMinExtent(vertices: FloatArray, numVertices: Int): Float {
        var minDist = Float.MAX_VALUE
        for (i in 0 until numVertices) {
            val ni = (i + 1) % numVertices
            val p1 = i * 3
            val p2 = ni * 3
            var maxEdgeDist = 0f
            for (j in 0 until numVertices) {
                if (j == i || j == ni) {
                    continue
                }
                val d = distancePtSeg2d(vertices, j * 3, vertices, p1, p2)
                maxEdgeDist = max(maxEdgeDist, d)
            }
            minDist = min(minDist, maxEdgeDist)
        }
        return sqrt(minDist)
    }

    private fun td(t0: Float, d: Float): Float {
        var t = t0
        if (d > 0) {
            t /= d
        }
        if (t < 0) {
            t = 0f
        } else if (t > 1) {
            t = 1f
        }
        return t
    }

    @JvmStatic
    fun distancePtSeg2d(vx: Float, vz: Float, poly: FloatArray, p: Int, q: Int): Float {
        val pqx = poly[q] - poly[p]
        val pqz = poly[q + 2] - poly[p + 2]
        var dx = vx - poly[p]
        var dz = vz - poly[p + 2]
        val d = pqx * pqx + pqz * pqz
        val t = td(pqx * dx + pqz * dz, d)
        dx = poly[p] + t * pqx - vx
        dz = poly[p + 2] + t * pqz - vz
        return dx * dx + dz * dz
    }

    @JvmStatic
    fun distancePtSeg2d(vertices: FloatArray, pt: Int, poly: FloatArray, p: Int, q: Int): Float {
        return distancePtSeg2d(vertices[pt], vertices[pt + 2], poly, p, q)
    }

    @JvmStatic
    fun distancePtSeg2d(v: Vector3f, poly: FloatArray, p: Int, q: Int): Float {
        return distancePtSeg2d(v.x, v.z, poly, p, q)
    }

    @JvmStatic
    fun distancePtSeg(vertices: FloatArray, pt: Int, p: Int, q: Int): Float {
        val pqx = vertices[q] - vertices[p]
        val pqy = vertices[q + 1] - vertices[p + 1]
        val pqz = vertices[q + 2] - vertices[p + 2]
        var dx = vertices[pt] - vertices[p]
        var dy = vertices[pt + 1] - vertices[p + 1]
        var dz = vertices[pt + 2] - vertices[p + 2]
        val d = pqx * pqx + pqy * pqy + pqz * pqz
        val t = td(pqx * dx + pqy * dy + pqz * dz, d)
        dx = vertices[p] + t * pqx - vertices[pt]
        dy = vertices[p + 1] + t * pqy - vertices[pt + 1]
        dz = vertices[p + 2] + t * pqz - vertices[pt + 2]
        return dx * dx + dy * dy + dz * dz
    }

    @JvmStatic
    fun circumcircle(vertices: FloatArray?, p1: Int, p2: Int, p3: Int, c: Vector3f): Float {
        val EPS = 1e-6f
        // Calculate the circle relative to p1, to avoid some precision issues.
        val v1 = Vector3f()
        val v2 = Vector3f()
        val v3 = Vector3f()
        sub(v2, vertices!!, p2, p1)
        sub(v3, vertices, p3, p1)
        val cp = vcross2(v1, v2, v3)
        return if (abs(cp) > EPS) {
            val v1Sq = vdot2(v1, v1)
            val v2Sq = vdot2(v2, v2)
            val v3Sq = vdot2(v3, v3)
            val n = 0.5f / cp
            c.set(
                (v1Sq * (v2.z - v3.z) + v2Sq * (v3.z - v1.z) + v3Sq * (v1.z - v2.z)) * n,
                0f,
                (v1Sq * (v3.x - v2.x) + v2Sq * (v1.x - v3.x) + v3Sq * (v2.x - v1.x)) * n
            )
            val r = vdist2(c, v1)
            add(c, c, vertices, p1)
            r
        } else {
            copy(c, vertices, p1)
            0f
        }
    }

    @JvmStatic
    fun updateLeftFace(edges0: IntArrayList, e: Int, s: Int, t: Int, f: Int) {
        val edges = edges0.values
        if (edges[e] == s && edges[e + 1] == t && edges[e + 2] == EV_UNDEF) {
            edges[e + 2] = f
        } else if (edges[e + 1] == s && edges[e] == t && edges[e + 3] == EV_UNDEF) {
            edges[e + 3] = f
        }
    }

    @JvmStatic
    fun overlapSegSeg2d(vertices: FloatArray, a: Int, b: Int, c: Int, d: Int): Boolean {
        val a1 = vcross2(vertices, a, b, d)
        val a2 = vcross2(vertices, a, b, c)
        if (a1 * a2 < 0.0f) {
            val a3 = vcross2(vertices, c, d, a)
            val a4 = a3 + a2 - a1
            return a3 * a4 < 0.0f
        }
        return false
    }

    @JvmStatic
    fun doesNotOverlapEdges(points: FloatArray, edges: IntArrayList, s1: Int, t1: Int): Boolean {
        val edgeData = edges.values
        var e = 0
        val l = edges.size
        while (e < l) {
            val s0 = edgeData[e]
            val t0 = edgeData[e + 1]
            // Same or connected edges do not overlap.
            if ((s0 == s1) || (s0 == t1) || (t0 == s1) || (t0 == t1)) {
                e += 4
                continue
            }
            if (overlapSegSeg2d(points, s0 * 3, t0 * 3, s1 * 3, t1 * 3)) {
                return false
            }
            e += 4
        }
        return true
    }

}