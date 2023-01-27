package org.recast4j.recast

import org.joml.Vector3f
import org.recast4j.IntArrayList
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
    fun vdistSq2(verts: FloatArray, p: Int, q: Int): Float {
        val dx = verts[q] - verts[p]
        val dy = verts[q + 2] - verts[p + 2]
        return dx * dx + dy * dy
    }

    @JvmStatic
    fun vdist2(verts: FloatArray, p: Int, q: Int): Float {
        return sqrt(vdistSq2(verts, p, q))
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
    fun vdistSq2(p: Vector3f, verts: FloatArray, q: Int): Float {
        val dx = verts[q] - p.x
        val dy = verts[q + 2] - p.z
        return dx * dx + dy * dy
    }

    @JvmStatic
    fun vdist2(p: Vector3f, verts: FloatArray, q: Int): Float {
        return sqrt(vdistSq2(p, verts, q))
    }

    @JvmStatic
    fun vcross2(verts: FloatArray, p1: Int, p2: Int, p3: Int): Float {
        val u1 = verts[p2] - verts[p1]
        val v1 = verts[p2 + 2] - verts[p1 + 2]
        val u2 = verts[p3] - verts[p1]
        val v2 = verts[p3 + 2] - verts[p1 + 2]
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
    fun getEdgeFlags(verts: FloatArray?, va: Int, vb: Int, vpoly: FloatArray?, npoly: Int): Int {
        // The flag returned by this function matches getDetailTriEdgeFlags in Detour.
        // Figure out if edge (va,vb) is part of the polygon boundary.
        val thrSqr = 0.001f * 0.001f
        var i = 0
        var j = npoly - 1
        while (i < npoly) {
            if (RecastMeshDetail.distancePtSeg2d(verts, va, vpoly, j * 3, i * 3) < thrSqr
                && RecastMeshDetail.distancePtSeg2d(verts, vb, vpoly, j * 3, i * 3) < thrSqr
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
    fun distToTriMesh(p: Vector3f, verts: FloatArray, tris: IntArrayList, ntris: Int): Float {
        var dmin = Float.MAX_VALUE
        for (i in 0 until ntris) {
            val va = tris[i * 4] * 3
            val vb = tris[i * 4 + 1] * 3
            val vc = tris[i * 4 + 2] * 3
            val d = distPtTri(p, verts, va, vb, vc)
            if (d < dmin) {
                dmin = d
            }
        }
        return if (dmin == Float.MAX_VALUE) -1f else dmin
    }

    @JvmStatic
    fun distToPoly(nvert: Int, verts: FloatArray, p: Vector3f): Float {
        var dmin = Float.MAX_VALUE
        var c = false
        val px = p.x
        val pz = p.z
        var j = nvert - 1
        for (i in 0 until nvert) {
            val vi = i * 3
            val vj = j * 3
            if (((verts[vi + 2] > pz) != (verts[vj + 2] > pz)) &&
                (px < (verts[vj] - verts[vi]) * (pz - verts[vi + 2]) / (verts[vj + 2] - verts[vi + 2]) + verts[vi])
            ) c = !c
            dmin = min(dmin, RecastMeshDetail.distancePtSeg2d(p, verts, vj, vi))
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

}