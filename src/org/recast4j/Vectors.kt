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
package org.recast4j

import org.joml.Vector3f
import org.joml.Vector3i
import org.recast4j.detour.BVNode
import org.recast4j.detour.VectorPtr
import kotlin.math.abs
import kotlin.math.sqrt

object Vectors {

    fun clamp(x: Float, min: Float, max: Float): Float {
        return if (x < min) min else if (x < max) x else max
    }

    fun clamp(x: Double, min: Double, max: Double): Double {
        return if (x < min) min else if (x < max) x else max
    }

    fun clamp(x: Int, min: Int, max: Int): Int {
        return if (x < min) min else if (x < max) x else max
    }

    fun min(a: Vector3f, b: FloatArray, i: Int) {
        a.x = kotlin.math.min(a.x, b[i])
        a.y = kotlin.math.min(a.y, b[i + 1])
        a.z = kotlin.math.min(a.z, b[i + 2])
    }

    fun max(a: Vector3f, b: FloatArray, i: Int) {
        a.x = kotlin.math.max(a.x, b[i])
        a.y = kotlin.math.max(a.y, b[i + 1])
        a.z = kotlin.math.max(a.z, b[i + 2])
    }

    fun copy(dst: FloatArray, input: FloatArray, i: Int) {
        copy(dst, 0, input, i)
    }

    fun copy(dst: Vector3f, src: FloatArray, srcI: Int) {
        dst.set(src[srcI], src[srcI + 1], src[srcI + 2])
    }

    fun copy(dst: FloatArray, src: FloatArray) {
        copy(dst, 0, src, 0)
    }

    fun copy(dst: FloatArray, dstI: Int, src: FloatArray, srcI: Int) {
        dst[dstI] = src[srcI]
        dst[dstI + 1] = src[srcI + 1]
        dst[dstI + 2] = src[srcI + 2]
    }

    fun add(dst: FloatArray, a: FloatArray, vertices: FloatArray, i: Int) {
        dst[0] = a[0] + vertices[i]
        dst[1] = a[1] + vertices[i + 1]
        dst[2] = a[2] + vertices[i + 2]
    }

    fun sub(dst: Vector3f, vertices: FloatArray, i: Int, j: Int) {
        dst.x = vertices[i] - vertices[j]
        dst.y = vertices[i + 1] - vertices[j + 1]
        dst.z = vertices[i + 2] - vertices[j + 2]
    }

    fun normalize(v: FloatArray) {
        val d = 1f / sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
        v[0] *= d
        v[1] *= d
        v[2] *= d
    }

    fun dot(v1: FloatArray, v2: FloatArray): Float {
        return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]
    }

    var EPS = 1e-4f

    /**
     * a + b * s
     */
    fun mad(a: Vector3f, b: Vector3f, f: Float): Vector3f {
        return Vector3f(b).mul(f).add(a)
    }

    /**
     * a += b * s
     */
    fun mad2(a: Vector3f, b: Vector3f, f: Float) {
        a.add(b.x * f, b.y * f, b.z * f)
    }

    fun lerp(vertices: FloatArray, v1: Int, v2: Int, t: Float): Vector3f {
        val dst = Vector3f()
        dst.x = vertices[v1] + (vertices[v2] - vertices[v1]) * t
        dst.y = vertices[v1 + 1] + (vertices[v2 + 1] - vertices[v1 + 1]) * t
        dst.z = vertices[v1 + 2] + (vertices[v2 + 2] - vertices[v1 + 2]) * t
        return dst
    }

    fun lerp(v1: Vector3f, v2: Vector3f, t: Float): Vector3f {
        return Vector3f(v1).lerp(v2, t)
    }

    fun sub(v1: VectorPtr, v2: VectorPtr): Vector3f {
        return Vector3f(v1[0] - v2[0], v1[1] - v2[1], v1[2] - v2[2])
    }

    fun sub(v1: Vector3f, v2: VectorPtr): Vector3f {
        return Vector3f(v1[0] - v2[0], v1[1] - v2[1], v1[2] - v2[2])
    }

    fun sub(v1: Vector3f, v2: Vector3f): Vector3f {
        return Vector3f(v1).sub(v2)
    }

    fun add(v1: Vector3f, v2: Vector3f): Vector3f {
        return Vector3f(v1).add(v2)
    }

    fun copy(v: Vector3f): Vector3f {
        return Vector3f(v)
    }

    fun set(out: Vector3f, a: Float, b: Float, c: Float) {
        out.set(a, b, c)
    }

    fun copy(out: Vector3f, input: Vector3f) {
        out.set(input)
    }

    fun copy(out: FloatArray, o: Int, v: Vector3f) {
        out[o] = v.x
        out[o + 1] = v.y
        out[o + 2] = v.z
    }

    fun copy(out: Vector3f, input: IntArray, i: Int) {
        out.set(input[i].toFloat(), input[i + 1].toFloat(), input[i + 2].toFloat())
    }

    fun sqr(a: Float): Float {
        return a * a
    }

    /**
     * Derives the distance between the specified points on the xz-plane.
     */
    fun dist2D(v1: Vector3f, v2: Vector3f): Float {
        val dx = v2.x - v1.x
        val dz = v2.z - v1.z
        return sqrt((dx * dx + dz * dz))
    }

    fun dist2DSqr(v1: Vector3f, v2: Vector3f): Float {
        val dx = v2.x - v1.x
        val dz = v2.z - v1.z
        return dx * dx + dz * dz
    }

    fun dist2DSqr(p: Vector3f, vertices: FloatArray, i: Int): Float {
        val dx = vertices[i] - p.x
        val dz = vertices[i + 2] - p.z
        return dx * dx + dz * dz
    }

    private val EQUAL_THRESHOLD = sqr(1f / 16384f)

    /**
     * Performs a 'sloppy' co-location check of the specified points.
     *
     * @return True if the points are considered to be at the same location.
     */
    fun vEqual(p0: Vector3f, p1: Vector3f): Boolean {
        return p0.distanceSquared(p1) < EQUAL_THRESHOLD
    }

    /// Derives the dot product of two vectors on the xz-plane. (@p u . @p v)
    /// @param[in] u A vector [(x, y, z)]
    /// @param[in] v A vector [(x, y, z)]
    /// @return The dot product on the xz-plane.
    ///
    /// The vectors are projected onto the xz-plane, so the y-values are
    /// ignored.
    fun dot2D(u: Vector3f, v: Vector3f): Float {
        return u.x * v.x + u.z * v.z
    }

    fun dot2D(u: Vector3f, v: FloatArray, vi: Int): Float {
        return u.x * v[vi] + u.z * v[vi + 2]
    }

    /// @}
    /// @name Computational geometry helper functions.
    /// @{
    /// Derives the signed xz-plane area of the triangle ABC, or the
    /// relationship of line AB to point C.
    /// @param[in] a Vertex A. [(x, y, z)]
    /// @param[in] b Vertex B. [(x, y, z)]
    /// @param[in] c Vertex C. [(x, y, z)]
    /// @return The signed xz-plane area of the triangle.
    fun triArea2D(vertices: FloatArray, a: Int, b: Int, c: Int): Float {
        val abx = vertices[b] - vertices[a]
        val abz = vertices[b + 2] - vertices[a + 2]
        val acx = vertices[c] - vertices[a]
        val acz = vertices[c + 2] - vertices[a + 2]
        return acx * abz - abx * acz
    }

    fun triArea2D(a: Vector3f, b: Vector3f, c: Vector3f): Float {
        val abx = b.x - a.x
        val abz = b.z - a.z
        val acx = c.x - a.x
        val acz = c.z - a.z
        return acx * abz - abx * acz
    }

    /**
     * Determines if two axis-aligned bounding boxes overlap.
     */
    fun overlapQuantBounds(amin: Vector3i, amax: Vector3i, n: BVNode): Boolean {
        return amin.x <= n.maxX && amax.x >= n.minX && amin.y <= n.maxY && amax.y >= n.minY && amin.z <= n.maxZ && amax.z >= n.minZ
    }

    /**
     * Determines if two axis-aligned bounding boxes overlap.
     */
    fun overlapBounds(amin: Vector3f, amax: Vector3f, bmin: Vector3f, bmax: Vector3f): Boolean {
        return amin.x <= bmax.x && amax.x >= bmin.x &&
                amin.y <= bmax.y && amax.y >= bmin.y &&
                amin.z <= bmax.z && amax.z >= bmin.z
    }

    fun distancePtSegSqr2D(pt: Vector3f, p: Vector3f, q: Vector3f): Pair<Float, Float> {
        val pqx = q.x - p.x
        val pqz = q.z - p.z
        var dx = pt.x - p.x
        var dz = pt.z - p.z
        val d = pqx * pqx + pqz * pqz
        var t = pqx * dx + pqz * dz
        if (d > 0) {
            t /= d
        }
        if (t < 0) {
            t = 0f
        } else if (t > 1) {
            t = 1f
        }
        dx = p.x + t * pqx - pt.x
        dz = p.z + t * pqz - pt.z
        return Pair(dx * dx + dz * dz, t)
    }

    fun closestHeightPointTriangle(p: Vector3f, a: Vector3f, b: Vector3f, c: Vector3f): Float {
        val v0 = sub(c, a)
        val v1 = sub(b, a)
        val v2 = sub(p, a)

        // Compute scaled barycentric coordinates
        var denom = v0.x * v1.z - v0.z * v1.x
        if (abs(denom) < EPS) {
            return Float.NaN
        }
        var u = v1.z * v2.x - v1.x * v2.z
        var v = v0.x * v2.z - v0.z * v2.x
        if (denom < 0) {
            denom = -denom
            u = -u
            v = -v
        }

        // If point lies inside the triangle, return interpolated y-coord.
        return if (u >= 0f && v >= 0f && u + v <= denom) {
            a.y + (v0.y * u + v1.y * v) / denom
        } else Float.NaN
    }

    /// @par
    ///
    /// All points are projected onto the xz-plane, so the y-values are ignored.
    fun pointInPolygon(pt: Vector3f, vertices: FloatArray, numVertices: Int): Boolean {
        var i: Int
        var j: Int
        var c = false
        i = 0
        j = numVertices - 1
        while (i < numVertices) {
            val vi = i * 3
            val vj = j * 3
            if (vertices[vi + 2] > pt.z != vertices[vj + 2] > pt.z && (pt.x < (vertices[vj] - vertices[vi])
                        * (pt.z - vertices[vi + 2]) / (vertices[vj + 2] - vertices[vi + 2]) + vertices[vi])
            ) {
                c = !c
            }
            j = i++
        }
        return c
    }

    fun distancePtPolyEdgesSqr(
        pt: Vector3f,
        vertices: FloatArray,
        numVertices: Int,
        ed: FloatArray,
        et: FloatArray
    ): Boolean {
        var i: Int
        var j: Int
        var c = false
        i = 0
        j = numVertices - 1
        while (i < numVertices) {
            val vi = i * 3
            val vj = j * 3
            if (vertices[vi + 2] > pt.z != vertices[vj + 2] > pt.z && (pt.x < (vertices[vj] - vertices[vi])
                        * (pt.z - vertices[vi + 2]) / (vertices[vj + 2] - vertices[vi + 2]) + vertices[vi])
            ) {
                c = !c
            }
            val (first, second) = distancePtSegSqr2D(pt, vertices, vj, vi)
            ed[j] = first
            et[j] = second
            j = i++
        }
        return c
    }

    fun projectPoly(axis: Vector3f, polygons: FloatArray, numPolygons: Int): FloatArray {
        var rmin: Float
        var rmax: Float
        rmax = dot2D(axis, polygons, 0)
        rmin = rmax
        for (i in 1 until numPolygons) {
            val d = dot2D(axis, polygons, i * 3)
            rmin = kotlin.math.min(rmin, d)
            rmax = kotlin.math.max(rmax, d)
        }
        return floatArrayOf(rmin, rmax)
    }

    fun overlapRange(amin: Float, amax: Float, bmin: Float, bmax: Float, eps: Float): Boolean {
        return amin + eps <= bmax && amax - eps >= bmin
    }

    fun overlapRange(amin: Float, amax: Float, bmin: Float, bmax: Float): Boolean {
        return amin <= bmax && amax >= bmin
    }

    fun overlapRange(amin: Int, amax: Int, bmin: Int, bmax: Int): Boolean {
        return amin <= bmax && amax >= bmin
    }

    var eps = 1e-4f

    /**
     * All vertices are projected onto the xz-plane, so the y-values are ignored.
     */
    fun overlapPolyPoly2D(polya: FloatArray, npolya: Int, polyb: FloatArray, npolyb: Int): Boolean {
        run {
            var i = 0
            var j = npolya - 1
            while (i < npolya) {
                val va = j * 3
                val vb = i * 3
                val n = Vector3f(polya[vb + 2] - polya[va + 2], 0f, -(polya[vb] - polya[va]))
                val aminmax = projectPoly(n, polya, npolya)
                val bminmax = projectPoly(n, polyb, npolyb)
                if (!overlapRange(aminmax[0], aminmax[1], bminmax[0], bminmax[1], eps)) {
                    // Found separating axis
                    return false
                }
                j = i++
            }
        }
        var i = 0
        var j = npolyb - 1
        while (i < npolyb) {
            val va = j * 3
            val vb = i * 3
            val n = Vector3f(polyb[vb + 2] - polyb[va + 2], 0f, -(polyb[vb] - polyb[va]))
            val aminmax = projectPoly(n, polya, npolya)
            val bminmax = projectPoly(n, polyb, npolyb)
            if (!overlapRange(aminmax[0], aminmax[1], bminmax[0], bminmax[1], eps)) {
                // Found separating axis
                return false
            }
            j = i++
        }
        return true
    }

    // Returns a random point in a convex polygon.
    // Adapted from Graphics Gems article.
    fun randomPointInConvexPoly(pts: FloatArray, npts: Int, areas: FloatArray, s: Float, t: Float): Vector3f {
        // Calc triangle areas
        var areasum = 0f
        for (i in 2 until npts) {
            areas[i] = triArea2D(pts, 0, (i - 1) * 3, i * 3)
            areasum += kotlin.math.max(0.001f, areas[i])
        }
        // Find sub triangle weighted by area.
        val thr = s * areasum
        var acc = 0f
        var u = 1f
        var tri = npts - 1
        for (i in 2 until npts) {
            val dacc = areas[i]
            if (thr >= acc && thr < acc + dacc) {
                u = (thr - acc) / dacc
                tri = i
                break
            }
            acc += dacc
        }
        val v = sqrt(t)
        val a = 1 - v
        val b = (1 - u) * v
        val c = u * v
        val pa = 0
        val pb = (tri - 1) * 3
        val pc = tri * 3
        return Vector3f(
            a * pts[pa] + b * pts[pb] + c * pts[pc],
            a * pts[pa + 1] + b * pts[pb + 1] + c * pts[pc + 1],
            a * pts[pa + 2] + b * pts[pb + 2] + c * pts[pc + 2]
        )
    }

    fun nextPow2(n: Int): Int {
        var v = n
        v--
        v = v or (v shr 1)
        v = v or (v shr 2)
        v = v or (v shr 4)
        v = v or (v shr 8)
        v = v or (v shr 16)
        v++
        return v
    }

    fun ilog2(n: Int): Int {
        var v = n
        var r: Int = (if (v > 0xffff) 1 else 0) shl 4
        v = v shr r
        var shift: Int = (if (v > 0xff) 1 else 0) shl 3
        v = v shr shift
        r = r or shift
        shift = (if (v > 0xf) 1 else 0) shl 2
        v = v shr shift
        r = r or shift
        shift = (if (v > 0x3) 1 else 0) shl 1
        v = v shr shift
        r = r or shift
        r = r or (v shr 1)
        return r
    }

    fun intersectSegmentPoly2D(p0: Vector3f, p1: Vector3f, vertices: FloatArray, nvertices: Int): IntersectResult {
        val result = IntersectResult()
        val EPS = 0.00000001f
        val dir = sub(p1, p0)
        var i = 0
        var j = nvertices - 1
        while (i < nvertices) {
            val vpj = VectorPtr(vertices, j * 3)
            val edge = sub(VectorPtr(vertices, i * 3), vpj)
            val diff = sub(p0, vpj)
            val n = -crossXZ(edge, diff)
            val d = -crossXZ(dir, edge)
            if (abs(d) < EPS) {
                // S is nearly parallel to this edge
                return if (n < 0) {
                    result
                } else {
                    j = i++
                    continue
                }
            }
            val t = n / d
            if (d < 0) {
                // segment S is entering across this edge
                if (t > result.tmin) {
                    result.tmin = t
                    result.segMin = j
                    // S enters after leaving polygon
                    if (result.tmin > result.tmax) {
                        return result
                    }
                }
            } else {
                // segment S is leaving across this edge
                if (t < result.tmax) {
                    result.tmax = t
                    result.segMax = j
                    // S leaves before entering polygon
                    if (result.tmax < result.tmin) {
                        return result
                    }
                }
            }
            j = i++
        }
        result.intersects = true
        return result
    }

    fun distancePtSegSqr2D(pt: Vector3f, vertices: FloatArray, p: Int, q: Int): Pair<Float, Float> {
        val pqx = vertices[q] - vertices[p]
        val pqz = vertices[q + 2] - vertices[p + 2]
        var dx = pt.x - vertices[p]
        var dz = pt.z - vertices[p + 2]
        val d = pqx * pqx + pqz * pqz
        var t = pqx * dx + pqz * dz
        if (d > 0) {
            t /= d
        }
        if (t < 0) {
            t = 0f
        } else if (t > 1) {
            t = 1f
        }
        dx = vertices[p] + t * pqx - pt.x
        dz = vertices[p + 2] + t * pqz - pt.z
        return Pair(dx * dx + dz * dz, t)
    }

    fun oppositeTile(side: Int): Int {
        return side + 4 and 0x7
    }

    fun crossXZ(a: Vector3f, b: Vector3f): Float {
        return a.x * b.z - a.z * b.x
    }

    fun intersectSegSeg2D(ap: Vector3f, aq: Vector3f, bp: Vector3f, bq: Vector3f): Pair<Float, Float>? {
        val u = sub(aq, ap)
        val v = sub(bq, bp)
        val w = sub(ap, bp)
        val d = crossXZ(u, v)
        if (abs(d) < 1e-6f) {
            return null
        }
        val s = crossXZ(v, w) / d
        val t = crossXZ(u, w) / d
        return Pair(s, t)
    }

    /**
     * Checks that the specified vector's components are all finite.
     */
    fun isFinite(v: Vector3f): Boolean {
        return v.isFinite
    }

    /**
     * Checks that the specified vector's xz components are finite.
     */
    fun isFinite2D(v: Vector3f): Boolean {
        return java.lang.Float.isFinite(v.x) && java.lang.Float.isFinite(v.z)
    }

    fun add(dst: Vector3f, v1: Vector3f, v2: Vector3f) {
        v1.add(v2, dst)
    }

    class IntersectResult {
        var intersects = false
        var tmin = 0f
        var tmax = 1f
        var segMin = -1
        var segMax = -1
    }
}