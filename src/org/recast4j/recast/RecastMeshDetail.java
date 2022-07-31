/*
Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
recast4J copyright (c) 2015-2019 Piotr Piastucki piotr@jtilia.org

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
package org.recast4j.recast;

import org.joml.Vector3f;
import org.recast4j.Vectors;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import static org.joml.Math.clamp;
import static org.recast4j.Vectors.copy;
import static org.recast4j.recast.RecastCommon.*;
import static org.recast4j.recast.RecastConstants.*;

public class RecastMeshDetail {

    static int MAX_VERTICES = 127;
    static int MAX_TRIS = 255; // Max tris for delaunay is 2n-2-k (n=num vertices, k=num hull vertices).
    static int MAX_VERTICES_PER_EDGE = 32;

    static int RC_UNSET_HEIGHT = RecastConstants.SPAN_MAX_HEIGHT;
    static int EV_UNDEF = -1;
    static int EV_HULL = -2;

    private static class HeightPatch {
        int xmin;
        int ymin;
        int width;
        int height;
        int[] data;
    }

    private static float vdot2(Vector3f a, Vector3f b) {
        return a.x * b.x + a.z * b.z;
    }

    private static float vdistSq2(float[] vertices, int p, int q) {
        float dx = vertices[q] - vertices[p];
        float dy = vertices[q + 2] - vertices[p + 2];
        return dx * dx + dy * dy;
    }

    private static float vdist2(float[] vertices, int p, int q) {
        return (float) Math.sqrt(vdistSq2(vertices, p, q));
    }

    private static float vdistSq2(Vector3f p, Vector3f q) {
        float dx = q.x - p.x;
        float dy = q.z - p.z;
        return dx * dx + dy * dy;
    }

    private static float vdist2(Vector3f p, Vector3f q) {
        return (float) Math.sqrt(vdistSq2(p, q));
    }

    private static float vdistSq2(Vector3f p, float[] vertices, int q) {
        float dx = vertices[q] - p.x;
        float dy = vertices[q + 2] - p.z;
        return dx * dx + dy * dy;
    }

    private static float vdist2(Vector3f p, float[] vertices, int q) {
        return (float) Math.sqrt(vdistSq2(p, vertices, q));
    }

    private static float vcross2(float[] vertices, int p1, int p2, int p3) {
        float u1 = vertices[p2] - vertices[p1];
        float v1 = vertices[p2 + 2] - vertices[p1 + 2];
        float u2 = vertices[p3] - vertices[p1];
        float v2 = vertices[p3 + 2] - vertices[p1 + 2];
        return u1 * v2 - v1 * u2;
    }

    private static float vcross2(Vector3f p1, Vector3f p2, Vector3f p3) {
        float u1 = p2.x - p1.x;
        float v1 = p2.z - p1.z;
        float u2 = p3.x - p1.x;
        float v2 = p3.z - p1.z;
        return u1 * v2 - v1 * u2;
    }

    private static void circumCircle(float[] vertices, int p1, int p2, int p3, Vector3f c, AtomicReference<Float> r) {
        float EPS = 1e-6f;
        // Calculate the circle relative to p1, to avoid some precision issues.
        Vector3f v1 = new Vector3f();
        Vector3f v2 = new Vector3f();
        Vector3f v3 = new Vector3f();
        Vectors.sub(v2, vertices, p2, p1);
        Vectors.sub(v3, vertices, p3, p1);

        float cp = vcross2(v1, v2, v3);
        if (Math.abs(cp) > EPS) {
            float v1Sq = vdot2(v1, v1);
            float v2Sq = vdot2(v2, v2);
            float v3Sq = vdot2(v3, v3);
            c.x = (v1Sq * (v2.z - v3.z) + v2Sq * (v3.z - v1.z) + v3Sq * (v1.z - v2.z)) / (2 * cp);
            c.y = 0;
            c.z = (v1Sq * (v3.x - v2.x) + v2Sq * (v1.x - v3.x) + v3Sq * (v2.x - v1.x)) / (2 * cp);
            r.set(vdist2(c, v1));
            c.add(vertices[p1], vertices[p1 + 1], vertices[p1 + 2]);
            return;
        }
        Vectors.copy(c, vertices, p1);
        r.set(0f);
    }

    private static float distPtTri(Vector3f p, float[] vertices, int a, int b, int c) {
        Vector3f v0 = new Vector3f();
        Vector3f v1 = new Vector3f();
        Vector3f v2 = new Vector3f();
        Vectors.sub(v0, vertices, c, a);
        Vectors.sub(v1, vertices, b, a);
        v2.set(p).sub(vertices[a], vertices[a + 1], vertices[a + 2]);

        float dot00 = vdot2(v0, v0);
        float dot01 = vdot2(v0, v1);
        float dot02 = vdot2(v0, v2);
        float dot11 = vdot2(v1, v1);
        float dot12 = vdot2(v1, v2);

        // Compute barycentric coordinates
        float invDenom = 1.0f / (dot00 * dot11 - dot01 * dot01);
        float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
        float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

        // If point lies inside the triangle, return interpolated y-coord.
        float EPS = 1e-4f;
        if (u >= -EPS && v >= -EPS && (u + v) <= 1 + EPS) {
            float y = vertices[a + 1] + v0.y * u + v1.y * v;
            return Math.abs(y - p.y);
        }
        return Float.MAX_VALUE;
    }

    private static float distancePtSeg(float[] vertices, int pt, int p, int q) {
        float pqx = vertices[q] - vertices[p];
        float pqy = vertices[q + 1] - vertices[p + 1];
        float pqz = vertices[q + 2] - vertices[p + 2];
        float dx = vertices[pt] - vertices[p];
        float dy = vertices[pt + 1] - vertices[p + 1];
        float dz = vertices[pt + 2] - vertices[p + 2];
        float d = pqx * pqx + pqy * pqy + pqz * pqz;
        float t = pqx * dx + pqy * dy + pqz * dz;
        if (d > 0) {
            t /= d;
        }
        if (t < 0) {
            t = 0;
        } else if (t > 1) {
            t = 1;
        }

        dx = vertices[p] + t * pqx - vertices[pt];
        dy = vertices[p + 1] + t * pqy - vertices[pt + 1];
        dz = vertices[p + 2] + t * pqz - vertices[pt + 2];

        return dx * dx + dy * dy + dz * dz;
    }

    private static float distancePtSeg2d(Vector3f px, float[] poly, int p, int q) {
        float pqx = poly[q] - poly[p];
        float pqz = poly[q + 2] - poly[p + 2];
        float dx = px.x - poly[p];
        float dz = px.z - poly[p + 2];
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

        dx = poly[p] + t * pqx - px.x;
        dz = poly[p + 2] + t * pqz - px.z;

        return dx * dx + dz * dz;
    }

    private static float distancePtSeg2d(float[] vertices, int pt, float[] poly, int p, int q) {
        float pqx = poly[q] - poly[p];
        float pqz = poly[q + 2] - poly[p + 2];
        float dx = vertices[pt] - poly[p];
        float dz = vertices[pt + 2] - poly[p + 2];
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

        dx = poly[p] + t * pqx - vertices[pt];
        dz = poly[p + 2] + t * pqz - vertices[pt + 2];

        return dx * dx + dz * dz;
    }

    private static float distToTriMesh(Vector3f p, float[] vertices, List<Integer> tris, int ntris) {
        float dmin = Float.MAX_VALUE;
        for (int i = 0; i < ntris; ++i) {
            int va = tris.get(i * 4) * 3;
            int vb = tris.get(i * 4 + 1) * 3;
            int vc = tris.get(i * 4 + 2) * 3;
            float d = distPtTri(p, vertices, va, vb, vc);
            if (d < dmin) {
                dmin = d;
            }
        }
        if (dmin == Float.MAX_VALUE) {
            return -1;
        }
        return dmin;
    }

    private static float distToPoly(int nvert, float[] vertices, Vector3f p) {

        float dmin = Float.MAX_VALUE;
        int i, j;
        boolean c = false;
        for (i = 0, j = nvert - 1; i < nvert; j = i++) {
            int vi = i * 3;
            int vj = j * 3;
            if (((vertices[vi + 2] > p.z) != (vertices[vj + 2] > p.z)) && (p.x < (vertices[vj] - vertices[vi]) * (p.z - vertices[vi + 2]) / (vertices[vj + 2] - vertices[vi + 2]) + vertices[vi])) {
                c = !c;
            }
            dmin = Math.min(dmin, distancePtSeg2d(p, vertices, vj, vi));
        }
        return c ? -dmin : dmin;
    }

    private static int getHeight(float fx, float fy, float fz, float ics, float ch, int radius, HeightPatch hp) {
        int ix = (int) Math.floor(fx * ics + 0.01f);
        int iz = (int) Math.floor(fz * ics + 0.01f);
        ix = clamp(ix - hp.xmin, 0, hp.width - 1);
        iz = clamp(iz - hp.ymin, 0, hp.height - 1);
        int h = hp.data[ix + iz * hp.width];
        if (h == RC_UNSET_HEIGHT) {
            // Special case when data might be bad.
            // Walk adjacent cells in a spiral up to 'radius', and look
            // for a pixel which has a valid height.
            int x = 1, z = 0, dx = 1, dz = 0;
            int maxSize = radius * 2 + 1;
            int maxIter = maxSize * maxSize - 1;

            int nextRingIterStart = 8;
            int nextRingIters = 16;

            float dmin = Float.MAX_VALUE;
            for (int i = 0; i < maxIter; ++i) {
                int nx = ix + x;
                int nz = iz + z;

                if (nx >= 0 && nz >= 0 && nx < hp.width && nz < hp.height) {
                    int nh = hp.data[nx + nz * hp.width];
                    if (nh != RC_UNSET_HEIGHT) {
                        float d = Math.abs(nh * ch - fy);
                        if (d < dmin) {
                            h = nh;
                            dmin = d;
                        }
                    }
                }

                // We are searching in a grid which looks approximately like this:
                // __________
                // |2 ______ 2|
                // | |1 __ 1| |
                // | | |__| | |
                // | |______| |
                // |__________|
                // We want to find the best height as close to the center cell as possible. This means that
                // if we find a height in one of the neighbor cells to the center, we don't want to
                // expand further out than the 8 neighbors - we want to limit our search to the closest
                // of these "rings", but the best height in the ring.
                // For example, the center is just 1 cell. We checked that at the entrance to the function.
                // The next "ring" contains 8 cells (marked 1 above). Those are all the neighbors to the center cell.
                // The next one again contains 16 cells (marked 2). In general each ring has 8 additional cells, which
                // can be thought of as adding 2 cells around the "center" of each side when we expand the ring.
                // Here we detect if we are about to enter the next ring, and if we are and we have found
                // a height, we abort the search.
                if (i + 1 == nextRingIterStart) {
                    if (h != RC_UNSET_HEIGHT) {
                        break;
                    }

                    nextRingIterStart += nextRingIters;
                    nextRingIters += 8;
                }

                if ((x == z) || ((x < 0) && (x == -z)) || ((x > 0) && (x == 1 - z))) {
                    int tmp = dx;
                    dx = -dz;
                    dz = tmp;
                }
                x += dx;
                z += dz;
            }
        }
        return h;
    }

    private static int findEdge(List<Integer> edges, int s, int t) {
        for (int i = 0; i < edges.size() / 4; i++) {
            int e = i * 4;
            if ((edges.get(e) == s && edges.get(e + 1) == t) || (edges.get(e) == t && edges.get(e + 1) == s)) {
                return i;
            }
        }
        return EV_UNDEF;
    }

    private static void addEdge(List<Integer> edges, int maxEdges, int s, int t, int l, int r) {
        if (edges.size() / 4 >= maxEdges) {
            throw new RuntimeException("addEdge: Too many edges (" + edges.size() / 4 + "/" + maxEdges + ").");
        }

        // Add edge if not already in the triangulation.
        int e = findEdge(edges, s, t);
        if (e == EV_UNDEF) {
            edges.add(s);
            edges.add(t);
            edges.add(l);
            edges.add(r);
        }
    }

    private static void updateLeftFace(List<Integer> edges, int e, int s, int t, int f) {
        if (edges.get(e) == s && edges.get(e + 1) == t && edges.get(e + 2) == EV_UNDEF) {
            edges.set(e + 2, f);
        } else if (edges.get(e + 1) == s && edges.get(e) == t && edges.get(e + 3) == EV_UNDEF) {
            edges.set(e + 3, f);
        }
    }

    private static boolean overlapSegSeg2d(float[] vertices, int a, int b, int c, int d) {
        float a1 = vcross2(vertices, a, b, d);
        float a2 = vcross2(vertices, a, b, c);
        if (a1 * a2 < 0.0f) {
            float a3 = vcross2(vertices, c, d, a);
            float a4 = a3 + a2 - a1;
            return a3 * a4 < 0.0f;
        }
        return false;
    }

    private static boolean overlapEdges(float[] pts, List<Integer> edges, int s1, int t1) {
        for (int i = 0; i < edges.size() / 4; ++i) {
            int s0 = edges.get(i * 4);
            int t0 = edges.get(i * 4 + 1);
            // Same or connected edges do not overlap.
            if (s0 == s1 || s0 == t1 || t0 == s1 || t0 == t1) {
                continue;
            }
            if (overlapSegSeg2d(pts, s0 * 3, t0 * 3, s1 * 3, t1 * 3)) {
                return true;
            }
        }
        return false;
    }

    static int completeFacet(float[] pts, int npts, List<Integer> edges, int maxEdges, int nfaces, int e) {
        float EPS = 1e-5f;

        int edge = e * 4;

        // Cache s and t.
        int s, t;
        if (edges.get(edge + 2) == EV_UNDEF) {
            s = edges.get(edge);
            t = edges.get(edge + 1);
        } else if (edges.get(edge + 3) == EV_UNDEF) {
            s = edges.get(edge + 1);
            t = edges.get(edge);
        } else {
            // Edge already completed.
            return nfaces;
        }

        // Find best point on left of edge.
        int pt = npts;
        Vector3f c = new Vector3f();
        AtomicReference<Float> r = new AtomicReference<>(-1f);
        for (int u = 0; u < npts; ++u) {
            if (u == s || u == t) {
                continue;
            }
            if (vcross2(pts, s * 3, t * 3, u * 3) > EPS) {
                if (r.get() < 0) {
                    // The circle is not updated yet, do it now.
                    pt = u;
                    circumCircle(pts, s * 3, t * 3, u * 3, c, r);
                    continue;
                }
                float d = vdist2(c, pts, u * 3);
                float tol = 0.001f;
                if (d > r.get() * (1 + tol)) {
                    // Outside current circumcircle, skip.
                } else if (d < r.get() * (1 - tol)) {
                    // Inside safe circumcircle, update circle.
                    pt = u;
                    circumCircle(pts, s * 3, t * 3, u * 3, c, r);
                } else {
                    // Inside epsilon circum circle, do extra tests to make sure the edge is valid.
                    // s-u and t-u cannot overlap with s-pt nor t-pt if they exists.
                    if (overlapEdges(pts, edges, s, u) || overlapEdges(pts, edges, t, u)) {
                        continue;
                    }
                    // Edge is valid.
                    pt = u;
                    circumCircle(pts, s * 3, t * 3, u * 3, c, r);
                }
            }
        }

        // Add new triangle or update edge info if s-t is on hull.
        if (pt < npts) {
            // Update face information of edge being completed.
            updateLeftFace(edges, e * 4, s, t, nfaces);

            // Add new edge or update face info of old edge.
            e = findEdge(edges, pt, s);
            if (e == EV_UNDEF) {
                addEdge(edges, maxEdges, pt, s, nfaces, EV_UNDEF);
            } else {
                updateLeftFace(edges, e * 4, pt, s, nfaces);
            }

            // Add new edge or update face info of old edge.
            e = findEdge(edges, t, pt);
            if (e == EV_UNDEF) {
                addEdge(edges, maxEdges, t, pt, nfaces, EV_UNDEF);
            } else {
                updateLeftFace(edges, e * 4, t, pt, nfaces);
            }

            nfaces++;
        } else {
            updateLeftFace(edges, e * 4, s, t, EV_HULL);
        }
        return nfaces;
    }

    private static void delaunayHull(int npts, float[] pts, int nhull, int[] hull, List<Integer> tris) {
        int nfaces = 0;
        int maxEdges = npts * 10;
        List<Integer> edges = new ArrayList<>(64);
        for (int i = 0, j = nhull - 1; i < nhull; j = i++) {
            addEdge(edges, maxEdges, hull[j], hull[i], EV_HULL, EV_UNDEF);
        }
        int currentEdge = 0;
        while (currentEdge < edges.size() / 4) {
            if (edges.get(currentEdge * 4 + 2) == EV_UNDEF) {
                nfaces = completeFacet(pts, npts, edges, maxEdges, nfaces, currentEdge);
            }
            if (edges.get(currentEdge * 4 + 3) == EV_UNDEF) {
                nfaces = completeFacet(pts, npts, edges, maxEdges, nfaces, currentEdge);
            }
            currentEdge++;
        }
        // Create tris
        tris.clear();
        for (int i = 0; i < nfaces * 4; ++i) {
            tris.add(-1);
        }

        for (int i = 0; i < edges.size() / 4; ++i) {
            int e = i * 4;
            if (edges.get(e + 3) >= 0) {
                // Left face
                int t = edges.get(e + 3) * 4;
                if (tris.get(t) == -1) {
                    tris.set(t, edges.get(e));
                    tris.set(t + 1, edges.get(e + 1));
                } else if (tris.get(t) == edges.get(e + 1)) {
                    tris.set(t + 2, edges.get(e));
                } else if (tris.get(t + 1) == edges.get(e)) {
                    tris.set(t + 2, edges.get(e + 1));
                }
            }
            if (edges.get(e + 2) >= 0) {
                // Right
                int t = edges.get(e + 2) * 4;
                if (tris.get(t) == -1) {
                    tris.set(t, edges.get(e + 1));
                    tris.set(t + 1, edges.get(e));
                } else if (tris.get(t) == edges.get(e)) {
                    tris.set(t + 2, edges.get(e + 1));
                } else if (tris.get(t + 1) == edges.get(e + 1)) {
                    tris.set(t + 2, edges.get(e));
                }
            }
        }

        for (int i = 0; i < tris.size() / 4; ++i) {
            int t = i * 4;
            if (tris.get(t) == -1 || tris.get(t + 1) == -1 || tris.get(t + 2) == -1) {
                System.err.println("Dangling! " + tris.get(t) + " " + tris.get(t + 1) + "  " + tris.get(t + 2));
                // ctx.log(RC_LOG_WARNING, "delaunayHull: Removing dangling face %d [%d,%d,%d].", i, t[0],t[1],t[2]);
                tris.set(t, tris.get(tris.size() - 4));
                tris.set(t + 1, tris.get(tris.size() - 3));
                tris.set(t + 2, tris.get(tris.size() - 2));
                tris.set(t + 3, tris.get(tris.size() - 1));
                tris.remove(tris.size() - 1);
                tris.remove(tris.size() - 1);
                tris.remove(tris.size() - 1);
                tris.remove(tris.size() - 1);
                --i;
            }
        }
    }

    // Calculate minimum extend of the polygon.
    private static float polyMinExtent(float[] vertices, int nvertices) {
        float minDist = Float.MAX_VALUE;
        for (int i = 0; i < nvertices; i++) {
            int ni = (i + 1) % nvertices;
            int p1 = i * 3;
            int p2 = ni * 3;
            float maxEdgeDist = 0;
            for (int j = 0; j < nvertices; j++) {
                if (j == i || j == ni) {
                    continue;
                }
                float d = distancePtSeg2d(vertices, j * 3, vertices, p1, p2);
                maxEdgeDist = Math.max(maxEdgeDist, d);
            }
            minDist = Math.min(minDist, maxEdgeDist);
        }
        return (float) Math.sqrt(minDist);
    }

    private static void triangulateHull(float[] vertices, int nhull, int[] hull, int nin, List<Integer> tris) {
        int start = 0, left = 1, right = nhull - 1;

        // Start from an ear with shortest perimeter.
        // This tends to favor well formed triangles as starting point.
        float dmin = Float.MAX_VALUE;
        for (int i = 0; i < nhull; i++) {
            if (hull[i] >= nin) {
                continue; // Ears are triangles with original vertices as middle vertex while others are actually line
            }
            // segments on edges
            int pi = RecastMesh.prev(i, nhull);
            int ni = RecastMesh.next(i, nhull);
            int pv = hull[pi] * 3;
            int cv = hull[i] * 3;
            int nv = hull[ni] * 3;
            float d = vdist2(vertices, pv, cv) + vdist2(vertices, cv, nv) + vdist2(vertices, nv, pv);
            if (d < dmin) {
                start = i;
                left = ni;
                right = pi;
                dmin = d;
            }
        }

        // Add first triangle
        tris.add(hull[start]);
        tris.add(hull[left]);
        tris.add(hull[right]);
        tris.add(0);

        // Triangulate the polygon by moving left or right,
        // depending on which triangle has shorter perimeter.
        // This heuristic was chose emprically, since it seems
        // handle tesselated straight edges well.
        while (RecastMesh.next(left, nhull) != right) {
            // Check to see if se should advance left or right.
            int nleft = RecastMesh.next(left, nhull);
            int nright = RecastMesh.prev(right, nhull);

            int cvleft = hull[left] * 3;
            int nvleft = hull[nleft] * 3;
            int cvright = hull[right] * 3;
            int nvright = hull[nright] * 3;
            float dleft = vdist2(vertices, cvleft, nvleft) + vdist2(vertices, nvleft, cvright);
            float dright = vdist2(vertices, cvright, nvright) + vdist2(vertices, cvleft, nvright);

            tris.add(hull[left]);
            if (dleft < dright) {
                tris.add(hull[nleft]);
                tris.add(hull[right]);
                tris.add(0);
                left = nleft;
            } else {
                tris.add(hull[nright]);
                tris.add(hull[right]);
                tris.add(0);
                right = nright;
            }
        }
    }

    private static float getJitterX(int i) {
        return (((i * 0x8da6b343) & 0xffff) / 65535.0f * 2.0f) - 1.0f;
    }

    private static float getJitterY(int i) {
        return (((i * 0xd8163841) & 0xffff) / 65535.0f * 2.0f) - 1.0f;
    }

    static int buildPolyDetail(float[] in, int nin, float sampleDist, float sampleMaxError,
                               int heightSearchRadius, CompactHeightfield chf, HeightPatch hp, float[] vertices, List<Integer> tris) {

        List<Integer> samples = new ArrayList<>(512);

        float[] edge = new float[(MAX_VERTICES_PER_EDGE + 1) * 3];
        int[] hull = new int[MAX_VERTICES];
        int nhull = 0;
        int nvertices = nin;

        for (int i = 0; i < nin; ++i) {
            Vectors.copy(vertices, i * 3, in, i * 3);
        }
        tris.clear();

        float cs = chf.cellSize;
        float ics = 1.0f / cs;

        // Calculate minimum extents of the polygon based on input data.
        float minExtent = polyMinExtent(vertices, nvertices);

        // Tessellate outlines.
        // This is done in separate pass in order to ensure
        // seamless height values across the ply boundaries.
        if (sampleDist > 0) {
            for (int i = 0, j = nin - 1; i < nin; j = i++) {
                int vj = j * 3;
                int vi = i * 3;
                boolean swapped = false;
                // Make sure the segments are always handled in same order
                // using lexological sort or else there will be seams.
                if (Math.abs(in[vj] - in[vi]) < 1e-6f) {
                    if (in[vj + 2] > in[vi + 2]) {
                        int temp = vi;
                        vi = vj;
                        vj = temp;
                        swapped = true;
                    }
                } else {
                    if (in[vj] > in[vi]) {
                        int temp = vi;
                        vi = vj;
                        vj = temp;
                        swapped = true;
                    }
                }
                // Create samples along the edge.
                float dx = in[vi] - in[vj];
                float dy = in[vi + 1] - in[vj + 1];
                float dz = in[vi + 2] - in[vj + 2];
                float d = (float) Math.sqrt(dx * dx + dz * dz);
                int nn = 1 + (int) Math.floor(d / sampleDist);
                if (nn >= MAX_VERTICES_PER_EDGE) {
                    nn = MAX_VERTICES_PER_EDGE - 1;
                }
                if (nvertices + nn >= MAX_VERTICES) {
                    nn = MAX_VERTICES - 1 - nvertices;
                }

                for (int k = 0; k <= nn; ++k) {
                    float u = (float) k / (float) nn;
                    int pos = k * 3;
                    edge[pos] = in[vj] + dx * u;
                    edge[pos + 1] = in[vj + 1] + dy * u;
                    edge[pos + 2] = in[vj + 2] + dz * u;
                    edge[pos + 1] = getHeight(edge[pos], edge[pos + 1], edge[pos + 2], ics, chf.cellHeight,
                            heightSearchRadius, hp) * chf.cellHeight;
                }
                // Simplify samples.
                int[] idx = new int[MAX_VERTICES_PER_EDGE];
                idx[0] = 0;
                idx[1] = nn;
                int nidx = 2;
                for (int k = 0; k < nidx - 1; ) {
                    int a = idx[k];
                    int b = idx[k + 1];
                    int va = a * 3;
                    int vb = b * 3;
                    // Find maximum deviation along the segment.
                    float maxd = 0;
                    int maxi = -1;
                    for (int m = a + 1; m < b; ++m) {
                        float dev = distancePtSeg(edge, m * 3, va, vb);
                        if (dev > maxd) {
                            maxd = dev;
                            maxi = m;
                        }
                    }
                    // If the max deviation is larger than accepted error,
                    // add new point, else continue to next segment.
                    if (maxi != -1 && maxd > sampleMaxError * sampleMaxError) {
                        if (nidx - k >= 0) System.arraycopy(idx, k, idx, k + 1, nidx - k);
                        idx[k + 1] = maxi;
                        nidx++;
                    } else {
                        ++k;
                    }
                }

                hull[nhull++] = j;
                // Add new vertices.
                if (swapped) {
                    for (int k = nidx - 2; k > 0; --k) {
                        Vectors.copy(vertices, nvertices * 3, edge, idx[k] * 3);
                        hull[nhull++] = nvertices;
                        nvertices++;
                    }
                } else {
                    for (int k = 1; k < nidx - 1; ++k) {
                        Vectors.copy(vertices, nvertices * 3, edge, idx[k] * 3);
                        hull[nhull++] = nvertices;
                        nvertices++;
                    }
                }
            }
        }

        // If the polygon minimum extent is small (sliver or small triangle), do not try to add internal points.
        if (minExtent < sampleDist * 2) {
            triangulateHull(vertices, nhull, hull, nin, tris);
            return nvertices;
        }

        // Tessellate the base mesh.
        // We're using the triangulateHull instead of delaunayHull as it tends to
        // create a bit better triangulation for long thin triangles when there
        // are no internal points.
        triangulateHull(vertices, nhull, hull, nin, tris);

        if (tris.size() == 0) {
            // Could not triangulate the poly, make sure there is some valid data there.
            throw new RuntimeException("buildPolyDetail: Could not triangulate polygon (" + nvertices + ") vertices).");
        }

        if (sampleDist > 0) {
            // Create sample locations in a grid.
            Vector3f bmin = new Vector3f();
            Vector3f bmax = new Vector3f();
            Vectors.copy(bmin, in, 0);
            Vectors.copy(bmax, in, 0);
            for (int i = 1; i < nin; ++i) {
                Vectors.min(bmin, in, i * 3);
                Vectors.max(bmax, in, i * 3);
            }
            int x0 = (int) Math.floor(bmin.x / sampleDist);
            int x1 = (int) Math.ceil(bmax.x / sampleDist);
            int z0 = (int) Math.floor(bmin.z / sampleDist);
            int z1 = (int) Math.ceil(bmax.z / sampleDist);
            for (int z = z0; z < z1; ++z) {
                for (int x = x0; x < x1; ++x) {
                    Vector3f pt = new Vector3f(x * sampleDist, (bmax.y + bmin.y) * 0.5f, z * sampleDist);
                    // Make sure the samples are not too close to the edges.
                    if (distToPoly(nin, in, pt) > -sampleDist / 2) {
                        continue;
                    }
                    samples.add(x);
                    samples.add(getHeight(pt.x, pt.y, pt.z, ics, chf.cellHeight, heightSearchRadius, hp));
                    samples.add(z);
                    samples.add(0); // Not added
                }
            }

            // Add the samples starting from the one that has the most
            // error. The procedure stops when all samples are added
            // or when the max error is within treshold.
            int nsamples = samples.size() / 4;
            for (int iter = 0; iter < nsamples; ++iter) {
                if (nvertices >= MAX_VERTICES) {
                    break;
                }

                // Find sample with most error.
                Vector3f bestpt = new Vector3f();
                float bestd = 0;
                int besti = -1;
                for (int i = 0; i < nsamples; ++i) {
                    int s = i * 4;
                    if (samples.get(s + 3) != 0) {
                        continue; // skip added.
                    }
                    Vector3f pt = new Vector3f();
                    // The sample location is jittered to get rid of some bad triangulations
                    // which are cause by symmetrical data from the grid structure.
                    pt.x = samples.get(s) * sampleDist + getJitterX(i) * cs * 0.1f;
                    pt.y = samples.get(s + 1) * chf.cellHeight;
                    pt.z = samples.get(s + 2) * sampleDist + getJitterY(i) * cs * 0.1f;
                    float d = distToTriMesh(pt, vertices, tris, tris.size() / 4);
                    if (d < 0) {
                        continue; // did not hit the mesh.
                    }
                    if (d > bestd) {
                        bestd = d;
                        besti = i;
                        bestpt = pt;
                    }
                }
                // If the max error is within accepted threshold, stop tesselating.
                if (bestd <= sampleMaxError || besti == -1) {
                    break;
                }
                // Mark sample as added.
                samples.set(besti * 4 + 3, 1);
                // Add the new sample point.
                copy(vertices, nvertices * 3, bestpt);
                nvertices++;

                // Create new triangulation.
                // TODO: Incremental add instead of full rebuild.
                delaunayHull(nvertices, vertices, nhull, hull, tris);
            }
        }

        int ntris = tris.size() / 4;
        if (ntris > MAX_TRIS) {
            List<Integer> subList = tris.subList(0, MAX_TRIS * 4);
            tris.clear();
            tris.addAll(subList);
            throw new RuntimeException(
                    "rcBuildPolyMeshDetail: Shrinking triangle count from " + ntris + " to max " + MAX_TRIS);
        }
        return nvertices;
    }

    static void seedArrayWithPolyCenter(Telemetry ctx, CompactHeightfield chf, int[] meshpoly, int poly, int npoly,
                                        int[] vertices, int bs, HeightPatch hp, List<Integer> array) {
        // Note: Reads to the compact heightfield are offset by border size (bs)
        // since border size offset is already removed from the polymesh vertices.

        int[] offset = {0, 0, -1, -1, 0, -1, 1, -1, 1, 0, 1, 1, 0, 1, -1, 1, -1, 0,};

        // Find cell closest to a poly vertex
        int startCellX = 0, startCellY = 0, startSpanIndex = -1;
        int dmin = RC_UNSET_HEIGHT;
        for (int j = 0; j < npoly && dmin > 0; ++j) {
            for (int k = 0; k < 9 && dmin > 0; ++k) {
                int ax = vertices[meshpoly[poly + j] * 3] + offset[k * 2];
                int ay = vertices[meshpoly[poly + j] * 3 + 1];
                int az = vertices[meshpoly[poly + j] * 3 + 2] + offset[k * 2 + 1];
                if (ax < hp.xmin || ax >= hp.xmin + hp.width || az < hp.ymin || az >= hp.ymin + hp.height) {
                    continue;
                }

                CompactCell c = chf.cells[(ax + bs) + (az + bs) * chf.width];
                for (int i = c.index, ni = c.index + c.count; i < ni && dmin > 0; ++i) {
                    CompactSpan s = chf.spans[i];
                    int d = Math.abs(ay - s.y);
                    if (d < dmin) {
                        startCellX = ax;
                        startCellY = az;
                        startSpanIndex = i;
                        dmin = d;
                    }
                }
            }
        }

        // Find center of the polygon
        int pcx = 0, pcy = 0;
        for (int j = 0; j < npoly; ++j) {
            pcx += vertices[meshpoly[poly + j] * 3];
            pcy += vertices[meshpoly[poly + j] * 3 + 2];
        }
        pcx /= npoly;
        pcy /= npoly;

        array.clear();
        array.add(startCellX);
        array.add(startCellY);
        array.add(startSpanIndex);
        int[] dirs = {0, 1, 2, 3};
        Arrays.fill(hp.data, 0, hp.width * hp.height, 0);
        // DFS to move to the center. Note that we need a DFS here and can not just move
        // directly towards the center without recording intermediate nodes, even though the polygons
        // are convex. In very rare we can get stuck due to contour simplification if we do not
        // record nodes.
        int cx = -1, cy = -1, ci = -1;
        while (true) {
            if (array.size() < 3) {
                ctx.warn("Walk towards polygon center failed to reach center");
                break;
            }
            ci = array.remove(array.size() - 1);
            cy = array.remove(array.size() - 1);
            cx = array.remove(array.size() - 1);

            // Check if close to center of the polygon.
            if (cx == pcx && cy == pcy) {
                break;
            }
            // If we are already at the correct X-position, prefer direction
            // directly towards the center in the Y-axis; otherwise prefer
            // direction in the X-axis
            int directDir;
            if (cx == pcx) {
                directDir = getDirForOffset(0, pcy > cy ? 1 : -1);
            } else {
                directDir = getDirForOffset(pcx > cx ? 1 : -1, 0);
            }

            // Push the direct dir last so we start with this on next iteration
            int tmp = dirs[3];
            dirs[3] = dirs[directDir];
            dirs[directDir] = tmp;

            CompactSpan cs = chf.spans[ci];

            for (int i = 0; i < 4; ++i) {
                int dir = dirs[i];
                if (getCon(cs, dir) == RC_NOT_CONNECTED) {
                    continue;
                }

                int newX = cx + getDirOffsetX(dir);
                int newY = cy + getDirOffsetY(dir);

                int hpx = newX - hp.xmin;
                int hpy = newY - hp.ymin;
                if (hpx < 0 || hpx >= hp.width || hpy < 0 || hpy >= hp.height) {
                    continue;
                }
                if (hp.data[hpx + hpy * hp.width] != 0) {
                    continue;
                }

                hp.data[hpx + hpy * hp.width] = 1;

                array.add(newX);
                array.add(newY);
                array.add(chf.cells[(newX + bs) + (newY + bs) * chf.width].index + getCon(cs, dir));
            }

            tmp = dirs[3];
            dirs[3] = dirs[directDir];
            dirs[directDir] = tmp;

        }

        array.clear();
        // getHeightData seeds are given in coordinates with borders
        array.add(cx + bs);
        array.add(cy + bs);
        array.add(ci);
        Arrays.fill(hp.data, 0, hp.width * hp.height, RC_UNSET_HEIGHT);
        CompactSpan cs = chf.spans[ci];
        hp.data[cx - hp.xmin + (cy - hp.ymin) * hp.width] = cs.y;
    }

    static final int RETRACT_SIZE = 256;

    static void push3(List<Integer> queue, int v1, int v2, int v3) {
        queue.add(v1);
        queue.add(v2);
        queue.add(v3);
    }

    static void getHeightData(Telemetry ctx, CompactHeightfield chf, int[] meshpolys, int poly, int npoly, int[] vertices,
                              int bs, HeightPatch hp, int region) {
        // Note: Reads to the compact heightfield are offset by border size (bs)
        // since border size offset is already removed from the polymesh vertices.

        List<Integer> queue = new ArrayList<>(512);
        Arrays.fill(hp.data, 0, hp.width * hp.height, RC_UNSET_HEIGHT);

        boolean empty = true;

        // We cannot sample from this poly if it was created from polys
        // of different regions. If it was then it could potentially be overlapping
        // with polys of that region and the heights sampled here could be wrong.
        if (region != RC_MULTIPLE_REGS) {
            // Copy the height from the same region, and mark region borders
            // as seed points to fill the rest.
            for (int hy = 0; hy < hp.height; hy++) {
                int y = hp.ymin + hy + bs;
                for (int hx = 0; hx < hp.width; hx++) {
                    int x = hp.xmin + hx + bs;
                    CompactCell c = chf.cells[x + y * chf.width];
                    for (int i = c.index, ni = c.index + c.count; i < ni; ++i) {
                        CompactSpan s = chf.spans[i];
                        if (s.reg == region) {
                            // Store height
                            hp.data[hx + hy * hp.width] = s.y;
                            empty = false;
                            // If any of the neighbours is not in same region,
                            // add the current location as flood fill start
                            boolean border = false;
                            for (int dir = 0; dir < 4; ++dir) {
                                if (getCon(s, dir) != RC_NOT_CONNECTED) {
                                    int ax = x + getDirOffsetX(dir);
                                    int ay = y + getDirOffsetY(dir);
                                    int ai = chf.cells[ax + ay * chf.width].index + getCon(s, dir);
                                    CompactSpan as = chf.spans[ai];
                                    if (as.reg != region) {
                                        border = true;
                                        break;
                                    }
                                }
                            }
                            if (border) {
                                push3(queue, x, y, i);
                            }
                            break;
                        }
                    }
                }
            }
        }

        // if the polygon does not contain any points from the current region (rare, but happens)
        // or if it could potentially be overlapping polygons of the same region,
        // then use the center as the seed point.
        if (empty) {
            seedArrayWithPolyCenter(ctx, chf, meshpolys, poly, npoly, vertices, bs, hp, queue);
        }

        int head = 0;

        // We assume the seed is centered in the polygon, so a BFS to collect
        // height data will ensure we do not move onto overlapping polygons and
        // sample wrong heights.
        while (head * 3 < queue.size()) {
            int cx = queue.get(head * 3);
            int cy = queue.get(head * 3 + 1);
            int ci = queue.get(head * 3 + 2);
            head++;
            if (head >= RETRACT_SIZE) {
                head = 0;
                queue = queue.subList(RETRACT_SIZE * 3, queue.size());
            }

            CompactSpan cs = chf.spans[ci];
            for (int dir = 0; dir < 4; ++dir) {
                if (getCon(cs, dir) == RC_NOT_CONNECTED) {
                    continue;
                }

                int ax = cx + getDirOffsetX(dir);
                int ay = cy + getDirOffsetY(dir);
                int hx = ax - hp.xmin - bs;
                int hy = ay - hp.ymin - bs;

                if (hx < 0 || hx >= hp.width || hy < 0 || hy >= hp.height) {
                    continue;
                }

                if (hp.data[hx + hy * hp.width] != RC_UNSET_HEIGHT) {
                    continue;
                }

                int ai = chf.cells[ax + ay * chf.width].index + getCon(cs, dir);
                CompactSpan as = chf.spans[ai];

                hp.data[hx + hy * hp.width] = as.y;
                push3(queue, ax, ay, ai);
            }
        }
    }

    static int getEdgeFlags(float[] vertices, int va, int vb, float[] vpoly, int npoly) {
        // The flag returned by this function matches getDetailTriEdgeFlags in Detour.
        // Figure out if edge (va,vb) is part of the polygon boundary.
        float thrSqr = 0.001f * 0.001f;
        for (int i = 0, j = npoly - 1; i < npoly; j = i++) {
            if (distancePtSeg2d(vertices, va, vpoly, j * 3, i * 3) < thrSqr
                    && distancePtSeg2d(vertices, vb, vpoly, j * 3, i * 3) < thrSqr) {
                return 1;
            }
        }
        return 0;
    }

    static int getTriFlags(float[] vertices, int va, int vb, int vc, float[] vpoly, int npoly) {
        return (getEdgeFlags(vertices, va, vb, vpoly, npoly)) |
                (getEdgeFlags(vertices, vb, vc, vpoly, npoly) << 2) |
                (getEdgeFlags(vertices, vc, va, vpoly, npoly) << 4);
    }

    /// @par
    ///
    /// See the #rcConfig documentation for more information on the configuration parameters.
    ///
    /// @see rcAllocPolyMeshDetail, rcPolyMesh, rcCompactHeightfield, rcPolyMeshDetail, rcConfig
    public static PolyMeshDetail buildPolyMeshDetail(Telemetry ctx, PolyMesh mesh, CompactHeightfield chf,
                                                     float sampleDist, float sampleMaxError) {

        ctx.startTimer("POLYMESHDETAIL");
        if (mesh.numVertices == 0 || mesh.numPolygons == 0) {
            return null;
        }

        PolyMeshDetail dmesh = new PolyMeshDetail();
        int nvp = mesh.maxVerticesPerPolygon;
        float cs = mesh.cs;
        float ch = mesh.ch;
        Vector3f orig = mesh.bmin;
        int borderSize = mesh.borderSize;
        int heightSearchRadius = (int) Math.max(1, Math.ceil(mesh.maxEdgeError));

        List<Integer> tris = new ArrayList<>(512);
        float[] vertices = new float[256 * 3];
        HeightPatch hp = new HeightPatch();
        int nPolyVertices = 0;
        int maxhw = 0, maxhh = 0;

        int[] bounds = new int[mesh.numPolygons * 4];
        float[] poly = new float[nvp * 3];

        // Find max size for a polygon area.
        for (int i = 0; i < mesh.numPolygons; ++i) {
            int p = i * nvp * 2;
            bounds[i * 4] = chf.width;
            bounds[i * 4 + 1] = 0;
            bounds[i * 4 + 2] = chf.height;
            bounds[i * 4 + 3] = 0;
            for (int j = 0; j < nvp; ++j) {
                if (mesh.polys[p + j] == RC_MESH_NULL_IDX) {
                    break;
                }
                int v = mesh.polys[p + j] * 3;
                bounds[i * 4] = Math.min(bounds[i * 4], mesh.vertices[v]);
                bounds[i * 4 + 1] = Math.max(bounds[i * 4 + 1], mesh.vertices[v]);
                bounds[i * 4 + 2] = Math.min(bounds[i * 4 + 2], mesh.vertices[v + 2]);
                bounds[i * 4 + 3] = Math.max(bounds[i * 4 + 3], mesh.vertices[v + 2]);
                nPolyVertices++;
            }
            bounds[i * 4] = Math.max(0, bounds[i * 4] - 1);
            bounds[i * 4 + 1] = Math.min(chf.width, bounds[i * 4 + 1] + 1);
            bounds[i * 4 + 2] = Math.max(0, bounds[i * 4 + 2] - 1);
            bounds[i * 4 + 3] = Math.min(chf.height, bounds[i * 4 + 3] + 1);
            if (bounds[i * 4] >= bounds[i * 4 + 1] || bounds[i * 4 + 2] >= bounds[i * 4 + 3]) {
                continue;
            }
            maxhw = Math.max(maxhw, bounds[i * 4 + 1] - bounds[i * 4]);
            maxhh = Math.max(maxhh, bounds[i * 4 + 3] - bounds[i * 4 + 2]);
        }
        hp.data = new int[maxhw * maxhh];

        dmesh.numSubMeshes = mesh.numPolygons;
        dmesh.meshes = new int[dmesh.numSubMeshes * 4];

        int vcap = nPolyVertices + nPolyVertices / 2;
        int tcap = vcap * 2;

        dmesh.numVertices = 0;
        dmesh.vertices = new float[vcap * 3];
        dmesh.numTriangles = 0;
        dmesh.triangles = new int[tcap * 4];

        for (int i = 0; i < mesh.numPolygons; ++i) {
            int p = i * nvp * 2;

            // Store polygon vertices for processing.
            int npoly = 0;
            for (int j = 0; j < nvp; ++j) {
                if (mesh.polys[p + j] == RC_MESH_NULL_IDX) {
                    break;
                }
                int v = mesh.polys[p + j] * 3;
                poly[j * 3] = mesh.vertices[v] * cs;
                poly[j * 3 + 1] = mesh.vertices[v + 1] * ch;
                poly[j * 3 + 2] = mesh.vertices[v + 2] * cs;
                npoly++;
            }

            // Get the height data from the area of the polygon.
            hp.xmin = bounds[i * 4];
            hp.ymin = bounds[i * 4 + 2];
            hp.width = bounds[i * 4 + 1] - bounds[i * 4];
            hp.height = bounds[i * 4 + 3] - bounds[i * 4 + 2];
            getHeightData(ctx, chf, mesh.polys, p, npoly, mesh.vertices, borderSize, hp, mesh.regs[i]);

            // Build detail mesh.
            int nvertices = buildPolyDetail(poly, npoly, sampleDist, sampleMaxError, heightSearchRadius, chf, hp,
                    vertices, tris);

            // Move detail vertices to world space.
            for (int j = 0; j < nvertices; ++j) {
                vertices[j * 3] += orig.x;
                vertices[j * 3 + 1] += orig.y + chf.cellHeight;
                vertices[j * 3 + 2] += orig.z;
            }
            // Offset poly too, will be used to flag checking.
            for (int j = 0; j < npoly; ++j) {
                poly[j * 3] += orig.x;
                poly[j * 3 + 1] += orig.y;
                poly[j * 3 + 2] += orig.z;
            }

            // Store detail submesh.
            int ntris = tris.size() / 4;

            dmesh.meshes[i * 4] = dmesh.numVertices;
            dmesh.meshes[i * 4 + 1] = nvertices;
            dmesh.meshes[i * 4 + 2] = dmesh.numTriangles;
            dmesh.meshes[i * 4 + 3] = ntris;

            // Store vertices, allocate more memory if necessary.
            if (dmesh.numVertices + nvertices > vcap) {
                while (dmesh.numVertices + nvertices > vcap) {
                    vcap += 256;
                }

                float[] newv = new float[vcap * 3];
                if (dmesh.numVertices != 0) {
                    System.arraycopy(dmesh.vertices, 0, newv, 0, 3 * dmesh.numVertices);
                }
                dmesh.vertices = newv;
            }
            for (int j = 0; j < nvertices; ++j) {
                dmesh.vertices[dmesh.numVertices * 3] = vertices[j * 3];
                dmesh.vertices[dmesh.numVertices * 3 + 1] = vertices[j * 3 + 1];
                dmesh.vertices[dmesh.numVertices * 3 + 2] = vertices[j * 3 + 2];
                dmesh.numVertices++;
            }

            // Store triangles, allocate more memory if necessary.
            if (dmesh.numTriangles + ntris > tcap) {
                while (dmesh.numTriangles + ntris > tcap) {
                    tcap += 256;
                }
                int[] newt = new int[tcap * 4];
                if (dmesh.numTriangles != 0) {
                    System.arraycopy(dmesh.triangles, 0, newt, 0, 4 * dmesh.numTriangles);
                }
                dmesh.triangles = newt;
            }
            for (int j = 0; j < ntris; ++j) {
                int t = j * 4;
                dmesh.triangles[dmesh.numTriangles * 4] = tris.get(t);
                dmesh.triangles[dmesh.numTriangles * 4 + 1] = tris.get(t + 1);
                dmesh.triangles[dmesh.numTriangles * 4 + 2] = tris.get(t + 2);
                dmesh.triangles[dmesh.numTriangles * 4 + 3] = getTriFlags(vertices, tris.get(t) * 3, tris.get(t + 1) * 3,
                        tris.get(t + 2) * 3, poly, npoly);
                dmesh.numTriangles++;
            }
        }

        ctx.stopTimer("POLYMESHDETAIL");
        return dmesh;

    }

}
