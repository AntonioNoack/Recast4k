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
package org.recast4j.detour;

import org.joml.Vector3f;
import org.joml.Vector3i;
import org.recast4j.Pair;

import java.util.Optional;

public class DetourCommon {

    static float EPS = 1e-4f;

    /// Performs a scaled vector addition. (@p v1 + (@p v2 * @p s))
    /// @param[out] dest The result vector. [(x, y, z)]
    /// @param[in] v1 The base vector. [(x, y, z)]
    /// @param[in] v2 The vector to scale and add to @p v1. [(x, y, z)]
    /// @param[in] s The amount to scale @p v2 by before adding to @p v1.
    public static Vector3f vMad(Vector3f v1, Vector3f v2, float s) {
        return new Vector3f(v2).mul(s).add(v1);
    }

    /// Performs a linear interpolation between two vectors. (@p v1 toward @p
    /// v2)
    /// @param[out] dest The result vector. [(x, y, x)]
    /// @param[in] v1 The starting vector.
    /// @param[in] v2 The destination vector.
    /// @param[in] t The interpolation factor. [Limits: 0 <= value <= 1.0]
    public static Vector3f vLerp(float[] verts, int v1, int v2, float t) {
        Vector3f dest = new Vector3f();
        dest.x = verts[v1] + (verts[v2] - verts[v1]) * t;
        dest.y = verts[v1 + 1] + (verts[v2 + 1] - verts[v1 + 1]) * t;
        dest.z = verts[v1 + 2] + (verts[v2 + 2] - verts[v1 + 2]) * t;
        return dest;
    }

    public static Vector3f vLerp(Vector3f v1, Vector3f v2, float t) {
        return new Vector3f(v1).lerp(v2, t);
    }

    public static Vector3f vSub(VectorPtr v1, VectorPtr v2) {
        return new Vector3f(v1.get(0) - v2.get(0), v1.get(1) - v2.get(1), v1.get(2) - v2.get(2));
    }

    public static Vector3f vSub(Vector3f v1, VectorPtr v2) {
        return new Vector3f(v1.get(0) - v2.get(0), v1.get(1) - v2.get(1), v1.get(2) - v2.get(2));
    }

    public static Vector3f vSub(Vector3f v1, Vector3f v2) {
        return new Vector3f(v1).sub(v2);
    }

    public static Vector3f vAdd(Vector3f v1, Vector3f v2) {
        return new Vector3f(v1).add(v2);
    }

    public static Vector3f copy(Vector3f in) {
        return new Vector3f(in);
    }

    public static void vSet(Vector3f out, float a, float b, float c) {
        out.set(a, b, c);
    }

    public static void copy(Vector3f out, Vector3f in) {
        out.set(in);
    }

    public static void copy(Vector3f out, float[] in, int i) {
        out.set(in[i], in[i + 1], in[i + 2]);
    }

    public static void copy(float[] out, int o, Vector3f in) {
        out[o] = in.x;
        out[o + 1] = in.y;
        out[o + 2] = in.z;
    }

    public static void copy(Vector3f out, int[] in, int i) {
        out.set(in[i], in[i + 1], in[i + 2]);
    }

    public static void min(Vector3f out, float[] in, int i) {
        out.x = Math.min(out.x, in[i]);
        out.y = Math.min(out.y, in[i + 1]);
        out.z = Math.min(out.z, in[i + 2]);
    }

    public static void max(Vector3f out, float[] in, int i) {
        out.x = Math.max(out.x, in[i]);
        out.y = Math.max(out.y, in[i + 1]);
        out.z = Math.max(out.z, in[i + 2]);
    }

    /// Returns the distance between two points.
    /// @param[in] v1 A point. [(x, y, z)]
    /// @param[in] v2 A point. [(x, y, z)]
    /// @return The distance between the two points.
    public static float vDist(float[] v1, float[] v2) {
        float dx = v2[0] - v1[0];
        float dy = v2[1] - v1[1];
        float dz = v2[2] - v1[2];
        return (float) Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    /// Returns the distance between two points.
    /// @param[in] v1 A point. [(x, y, z)]
    /// @param[in] v2 A point. [(x, y, z)]
    /// @return The distance between the two points.
    static float vDistSqr(float[] v1, float[] v2) {
        float dx = v2[0] - v1[0];
        float dy = v2[1] - v1[1];
        float dz = v2[2] - v1[2];
        return dx * dx + dy * dy + dz * dz;
    }

    public static float sqr(float a) {
        return a * a;
    }

    /// Derives the square of the scalar length of the vector. (len * len)
    /// @param[in] v The vector. [(x, y, z)]
    /// @return The square of the scalar length of the vector.
    public static float vLenSqr(float[] v) {
        return v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    }

    public static float vLen(float[] v) {
        return (float) Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    }

    static float vDist(float[] v1, float[] verts, int i) {
        float dx = verts[i] - v1[0];
        float dy = verts[i + 1] - v1[1];
        float dz = verts[i + 2] - v1[2];
        return (float) Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    public static float clamp(float v, float min, float max) {
        return Math.max(Math.min(v, max), min);
    }

    public static int clamp(int v, int min, int max) {
        return Math.max(Math.min(v, max), min);
    }

    /// Derives the distance between the specified points on the xz-plane.
    /// @param[in] v1 A point. [(x, y, z)]
    /// @param[in] v2 A point. [(x, y, z)]
    /// @return The distance between the point on the xz-plane.
    ///
    /// The vectors are projected onto the xz-plane, so the y-values are
    /// ignored.
    public static float vDist2D(Vector3f v1, Vector3f v2) {
        float dx = v2.x - v1.x;
        float dz = v2.z - v1.z;
        return (float) Math.sqrt(dx * dx + dz * dz);
    }

    public static float vDist2DSqr(Vector3f v1, Vector3f v2) {
        float dx = v2.x - v1.x;
        float dz = v2.z - v1.z;
        return dx * dx + dz * dz;
    }

    public static float vDist2DSqr(Vector3f p, float[] verts, int i) {
        float dx = verts[i] - p.x;
        float dz = verts[i + 2] - p.z;
        return dx * dx + dz * dz;
    }

    /// Normalizes the vector.
    /// @param[in,out] v The vector to normalize. [(x, y, z)]
    public static void normalize(float[] v) {
        float d = (float) (1.0f / Math.sqrt(sqr(v[0]) + sqr(v[1]) + sqr(v[2])));
        if (d != 0) {
            v[0] *= d;
            v[1] *= d;
            v[2] *= d;
        }
    }

    private static final float EQUAL_THRESHOLD = sqr(1.0f / 16384.0f);

    /// Performs a 'sloppy' colocation check of the specified points.
    /// @param[in] p0 A point. [(x, y, z)]
    /// @param[in] p1 A point. [(x, y, z)]
    /// @return True if the points are considered to be at the same location.
    ///
    /// Basically, this function will return true if the specified points are
    /// close enough to eachother to be considered colocated.
    public static boolean vEqual(Vector3f p0, Vector3f p1) {
        return p0.distanceSquared(p1) < EQUAL_THRESHOLD;
    }

    /// Derives the dot product of two vectors on the xz-plane. (@p u . @p v)
    /// @param[in] u A vector [(x, y, z)]
    /// @param[in] v A vector [(x, y, z)]
    /// @return The dot product on the xz-plane.
    ///
    /// The vectors are projected onto the xz-plane, so the y-values are
    /// ignored.
    public static float vDot2D(Vector3f u, Vector3f v) {
        return u.x * v.x + u.z * v.z;
    }

    static float vDot2D(Vector3f u, float[] v, int vi) {
        return u.x * v[vi] + u.z * v[vi + 2];
    }

    /// Derives the xz-plane 2D perp product of the two vectors. (uz*vx - ux*vz)
    /// @param[in] u The LHV vector [(x, y, z)]
    /// @param[in] v The RHV vector [(x, y, z)]
    /// @return The dot product on the xz-plane.
    ///
    /// The vectors are projected onto the xz-plane, so the y-values are
    /// ignored.
    public static float vPerp2D(Vector3f u, Vector3f v) {
        return u.z * v.x - u.x * v.z;
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
    public static float triArea2D(float[] verts, int a, int b, int c) {
        float abx = verts[b] - verts[a];
        float abz = verts[b + 2] - verts[a + 2];
        float acx = verts[c] - verts[a];
        float acz = verts[c + 2] - verts[a + 2];
        return acx * abz - abx * acz;
    }

    public static float triArea2D(Vector3f a, Vector3f b, Vector3f c) {
        float abx = b.x - a.x;
        float abz = b.z - a.z;
        float acx = c.x - a.x;
        float acz = c.z - a.z;
        return acx * abz - abx * acz;
    }

    /// Determines if two axis-aligned bounding boxes overlap.
    /// @param[in] amin Minimum bounds of box A. [(x, y, z)]
    /// @param[in] amax Maximum bounds of box A. [(x, y, z)]
    /// @param[in] bmin Minimum bounds of box B. [(x, y, z)]
    /// @param[in] bmax Maximum bounds of box B. [(x, y, z)]
    /// @return True if the two AABB's overlap.
    /// @see dtOverlapBounds
    static boolean overlapQuantBounds(Vector3i amin, Vector3i amax, Vector3i bmin, Vector3i bmax) {
        return amin.x <= bmax.x && amax.x >= bmin.x &&
                amin.y <= bmax.y && amax.y >= bmin.y &&
                amin.z <= bmax.z && amax.z >= bmin.z;
    }

    /// Determines if two axis-aligned bounding boxes overlap.
    /// @param[in] amin Minimum bounds of box A. [(x, y, z)]
    /// @param[in] amax Maximum bounds of box A. [(x, y, z)]
    /// @param[in] bmin Minimum bounds of box B. [(x, y, z)]
    /// @param[in] bmax Maximum bounds of box B. [(x, y, z)]
    /// @return True if the two AABB's overlap.
    /// @see dtOverlapQuantBounds
    public static boolean overlapBounds(Vector3f amin, Vector3f amax, Vector3f bmin, Vector3f bmax) {
        return !(amin.x > bmax.x) && !(amax.x < bmin.x) &&
                !(amin.y > bmax.y) && !(amax.y < bmin.y) &&
                !(amin.z > bmax.z) && !(amax.z < bmin.z);
    }

    public static Pair<Float, Float> distancePtSegSqr2D(Vector3f pt, Vector3f p, Vector3f q) {
        float pqx = q.x - p.x;
        float pqz = q.z - p.z;
        float dx = pt.x - p.x;
        float dz = pt.z - p.z;
        float d = pqx * pqx + pqz * pqz;
        float t = pqx * dx + pqz * dz;
        if (d > 0) {
            t /= d;
        }
        if (t < 0) {
            t = 0;
        } else if (t > 1) {
            t = 1;
        }
        dx = p.x + t * pqx - pt.x;
        dz = p.z + t * pqz - pt.z;
        return new Pair<>(dx * dx + dz * dz, t);
    }

    static Optional<Float> closestHeightPointTriangle(Vector3f p, Vector3f a, Vector3f b, Vector3f c) {
        Vector3f v0 = vSub(c, a);
        Vector3f v1 = vSub(b, a);
        Vector3f v2 = vSub(p, a);

        // Compute scaled barycentric coordinates
        float denom = v0.x * v1.z - v0.z * v1.x;
        if (Math.abs(denom) < EPS) {
            return Optional.empty();
        }

        float u = v1.z * v2.x - v1.x * v2.z;
        float v = v0.x * v2.z - v0.z * v2.x;

        if (denom < 0) {
            denom = -denom;
            u = -u;
            v = -v;
        }

        // If point lies inside the triangle, return interpolated ycoord.
        if (u >= 0.0f && v >= 0.0f && (u + v) <= denom) {
            float h = a.y + (v0.y * u + v1.y * v) / denom;
            return Optional.of(h);
        }

        return Optional.empty();
    }

    /// @par
    ///
    /// All points are projected onto the xz-plane, so the y-values are ignored.
    static boolean pointInPolygon(Vector3f pt, float[] verts, int nverts) {
        int i, j;
        boolean c = false;
        for (i = 0, j = nverts - 1; i < nverts; j = i++) {
            int vi = i * 3;
            int vj = j * 3;
            if (((verts[vi + 2] > pt.z) != (verts[vj + 2] > pt.z)) && (pt.x < (verts[vj] - verts[vi])
                    * (pt.z - verts[vi + 2]) / (verts[vj + 2] - verts[vi + 2]) + verts[vi])) {
                c = !c;
            }
        }
        return c;
    }

    static boolean distancePtPolyEdgesSqr(Vector3f pt, float[] verts, int nverts, float[] ed, float[] et) {
        int i, j;
        boolean c = false;
        for (i = 0, j = nverts - 1; i < nverts; j = i++) {
            int vi = i * 3;
            int vj = j * 3;
            if (((verts[vi + 2] > pt.z) != (verts[vj + 2] > pt.z)) && (pt.x < (verts[vj] - verts[vi])
                    * (pt.z - verts[vi + 2]) / (verts[vj + 2] - verts[vi + 2]) + verts[vi])) {
                c = !c;
            }
            Pair<Float, Float> edet = distancePtSegSqr2D(pt, verts, vj, vi);
            ed[j] = edet.first;
            et[j] = edet.second;
        }
        return c;
    }

    static float[] projectPoly(Vector3f axis, float[] poly, int npoly) {
        float rmin, rmax;
        rmin = rmax = vDot2D(axis, poly, 0);
        for (int i = 1; i < npoly; ++i) {
            float d = vDot2D(axis, poly, i * 3);
            rmin = Math.min(rmin, d);
            rmax = Math.max(rmax, d);
        }
        return new float[]{rmin, rmax};
    }

    public static boolean overlapRange(float amin, float amax, float bmin, float bmax, float eps) {
        return !((amin + eps) > bmax) && !((amax - eps) < bmin);
    }

    public static boolean overlapRange(float amin, float amax, float bmin, float bmax) {
        return !(amin > bmax) && !(amax < bmin);
    }

    public static boolean overlapRange(int amin, int amax, int bmin, int bmax) {
        return amin <= bmax && amax >= bmin;
    }

    static float eps = 1e-4f;

    /// @par
    ///
    /// All vertices are projected onto the xz-plane, so the y-values are ignored.
    static boolean overlapPolyPoly2D(float[] polya, int npolya, float[] polyb, int npolyb) {

        for (int i = 0, j = npolya - 1; i < npolya; j = i++) {
            int va = j * 3;
            int vb = i * 3;

            Vector3f n = new Vector3f(polya[vb + 2] - polya[va + 2], 0, -(polya[vb] - polya[va]));

            float[] aminmax = projectPoly(n, polya, npolya);
            float[] bminmax = projectPoly(n, polyb, npolyb);
            if (!overlapRange(aminmax[0], aminmax[1], bminmax[0], bminmax[1], eps)) {
                // Found separating axis
                return false;
            }
        }
        for (int i = 0, j = npolyb - 1; i < npolyb; j = i++) {
            int va = j * 3;
            int vb = i * 3;

            Vector3f n = new Vector3f(polyb[vb + 2] - polyb[va + 2], 0, -(polyb[vb] - polyb[va]));

            float[] aminmax = projectPoly(n, polya, npolya);
            float[] bminmax = projectPoly(n, polyb, npolyb);
            if (!overlapRange(aminmax[0], aminmax[1], bminmax[0], bminmax[1], eps)) {
                // Found separating axis
                return false;
            }
        }
        return true;
    }

    // Returns a random point in a convex polygon.
    // Adapted from Graphics Gems article.
    static Vector3f randomPointInConvexPoly(float[] pts, int npts, float[] areas, float s, float t) {
        // Calc triangle araes
        float areasum = 0.0f;
        for (int i = 2; i < npts; i++) {
            areas[i] = triArea2D(pts, 0, (i - 1) * 3, i * 3);
            areasum += Math.max(0.001f, areas[i]);
        }
        // Find sub triangle weighted by area.
        float thr = s * areasum;
        float acc = 0.0f;
        float u = 1.0f;
        int tri = npts - 1;
        for (int i = 2; i < npts; i++) {
            float dacc = areas[i];
            if (thr >= acc && thr < (acc + dacc)) {
                u = (thr - acc) / dacc;
                tri = i;
                break;
            }
            acc += dacc;
        }

        float v = (float) Math.sqrt(t);

        float a = 1 - v;
        float b = (1 - u) * v;
        float c = u * v;
        int pa = 0;
        int pb = (tri - 1) * 3;
        int pc = tri * 3;

        return new Vector3f(a * pts[pa] + b * pts[pb] + c * pts[pc],
                a * pts[pa + 1] + b * pts[pb + 1] + c * pts[pc + 1],
                a * pts[pa + 2] + b * pts[pb + 2] + c * pts[pc + 2]);
    }

    public static int nextPow2(int v) {
        v--;
        v |= v >> 1;
        v |= v >> 2;
        v |= v >> 4;
        v |= v >> 8;
        v |= v >> 16;
        v++;
        return v;
    }

    public static int ilog2(int v) {
        int r;
        int shift;
        r = (v > 0xffff ? 1 : 0) << 4;
        v >>= r;
        shift = (v > 0xff ? 1 : 0) << 3;
        v >>= shift;
        r |= shift;
        shift = (v > 0xf ? 1 : 0) << 2;
        v >>= shift;
        r |= shift;
        shift = (v > 0x3 ? 1 : 0) << 1;
        v >>= shift;
        r |= shift;
        r |= (v >> 1);
        return r;
    }

    public static class IntersectResult {
        boolean intersects;
        float tmin;
        float tmax = 1f;
        int segMin = -1;
        int segMax = -1;
    }

    static IntersectResult intersectSegmentPoly2D(Vector3f p0, Vector3f p1, float[] verts, int nverts) {

        IntersectResult result = new IntersectResult();
        float EPS = 0.00000001f;
        Vector3f dir = vSub(p1, p0);

        for (int i = 0, j = nverts - 1; i < nverts; j = i++) {
            VectorPtr vpj = new VectorPtr(verts, j * 3);
            Vector3f edge = vSub(new VectorPtr(verts, i * 3), vpj);
            Vector3f diff = vSub(p0, vpj);
            float n = vPerp2D(edge, diff);
            float d = vPerp2D(dir, edge);
            if (Math.abs(d) < EPS) {
                // S is nearly parallel to this edge
                if (n < 0) {
                    return result;
                } else {
                    continue;
                }
            }
            float t = n / d;
            if (d < 0) {
                // segment S is entering across this edge
                if (t > result.tmin) {
                    result.tmin = t;
                    result.segMin = j;
                    // S enters after leaving polygon
                    if (result.tmin > result.tmax) {
                        return result;
                    }
                }
            } else {
                // segment S is leaving across this edge
                if (t < result.tmax) {
                    result.tmax = t;
                    result.segMax = j;
                    // S leaves before entering polygon
                    if (result.tmax < result.tmin) {
                        return result;
                    }
                }
            }
        }
        result.intersects = true;
        return result;
    }

    public static Pair<Float, Float> distancePtSegSqr2D(Vector3f pt, float[] verts, int p, int q) {
        float pqx = verts[q] - verts[p];
        float pqz = verts[q + 2] - verts[p + 2];
        float dx = pt.x - verts[p];
        float dz = pt.z - verts[p + 2];
        float d = pqx * pqx + pqz * pqz;
        float t = pqx * dx + pqz * dz;
        if (d > 0) {
            t /= d;
        }
        if (t < 0) {
            t = 0;
        } else if (t > 1) {
            t = 1;
        }
        dx = verts[p] + t * pqx - pt.x;
        dz = verts[p + 2] + t * pqz - pt.z;
        return new Pair<>(dx * dx + dz * dz, t);
    }

    static int oppositeTile(int side) {
        return (side + 4) & 0x7;
    }

    static float vperpXZ(Vector3f a, Vector3f b) {
        return a.x * b.z - a.z * b.x;
    }

    static Optional<Pair<Float, Float>> intersectSegSeg2D(Vector3f ap, Vector3f aq, Vector3f bp, Vector3f bq) {
        Vector3f u = vSub(aq, ap);
        Vector3f v = vSub(bq, bp);
        Vector3f w = vSub(ap, bp);
        float d = vperpXZ(u, v);
        if (Math.abs(d) < 1e-6f) {
            return Optional.empty();
        }
        float s = vperpXZ(v, w) / d;
        float t = vperpXZ(u, w) / d;
        return Optional.of(new Pair<>(s, t));
    }

    /// Checks that the specified vector's components are all finite.
    /// @param[in] v A point. [(x, y, z)]
    /// @return True if all of the point's components are finite, i.e. not NaN
    /// or any of the infinities.
    public static boolean isFinite(float[] v) {
        return Float.isFinite(v[0]) && Float.isFinite(v[1]) && Float.isFinite(v[2]);
    }

    public static boolean isFinite(Vector3f v) {
        return v.isFinite();
    }

    /// Checks that the specified vector's 2D components are finite.
    /// @param[in] v A point. [(x, y, z)]
    public static boolean vIsFinite2D(Vector3f v) {
        return Float.isFinite(v.x) && Float.isFinite(v.z);
    }

}
