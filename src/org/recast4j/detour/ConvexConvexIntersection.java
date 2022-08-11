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

package org.recast4j.detour;

import org.joml.Vector3f;
import org.recast4j.Pair;

import java.util.Arrays;
import java.util.Optional;

import static org.recast4j.Vectors.*;

/**
 * Convex-convex intersection based on "Computational Geometry in C" by Joseph O'Rourke
 */
public class ConvexConvexIntersection {

    private static final float EPSILON = 0.0001f;

    private enum InFlag {
        Pin, Qin, Unknown
    }

    private enum Intersection {
        None, Single, Overlap
    }

    public static float[] intersect(float[] p, float[] q) {
        int n = p.length / 3;
        int m = q.length / 3;
        float[] inters = new float[Math.max(m, n) * 3 * 3];
        int ii = 0;
        /* Initialize variables. */
        Vector3f a = new Vector3f();
        Vector3f b = new Vector3f();
        Vector3f a1 = new Vector3f();
        Vector3f b1 = new Vector3f();

        int aa = 0;
        int ba = 0;
        int ai = 0;
        int bi = 0;

        InFlag f = InFlag.Unknown;
        boolean FirstPoint = true;
        Vector3f ip = new Vector3f();
        Vector3f iq = new Vector3f();

        do {
            copy(a, p, 3 * (ai % n));
            copy(b, q, 3 * (bi % m));
            copy(a1, p, 3 * ((ai + n - 1) % n)); // prev a
            copy(b1, q, 3 * ((bi + m - 1) % m)); // prev b

            Vector3f A = sub(a, a1);
            Vector3f B = sub(b, b1);

            float cross = B.x * A.z - A.x * B.z;// triArea2D({0, 0}, A, B);
            float aHB = triArea2D(b1, b, a);
            float bHA = triArea2D(a1, a, b);
            if (Math.abs(cross) < EPSILON) {
                cross = 0f;
            }
            boolean parallel = cross == 0f;
            Intersection code = parallel ? parallelInt(a1, a, b1, b, ip, iq) : segSegInt(a1, a, b1, b, ip);

            if (code == Intersection.Single) {
                if (FirstPoint) {
                    FirstPoint = false;
                    aa = ba = 0;
                }
                ii = addVertex(inters, ii, ip);
                f = inOut(f, aHB, bHA);
            }

            /*-----Advance rules-----*/

            /* Special case: A & B overlap and oppositely oriented. */
            if (code == Intersection.Overlap && dot2D(A, B) < 0) {
                ii = addVertex(inters, ii, ip);
                ii = addVertex(inters, ii, iq);
                break;
            }

            /* Special case: A & B parallel and separated. */
            if (parallel && aHB < 0f && bHA < 0f) {
                return null;
            }

            /* Special case: A & B collinear. */
            else if (parallel && Math.abs(aHB) < EPSILON && Math.abs(bHA) < EPSILON) {
                /* Advance but do not output point. */
                if (f == InFlag.Pin) {
                    ba++;
                    bi++;
                } else {
                    aa++;
                    ai++;
                }
            }

            /* Generic cases. */
            else if (cross >= 0) {
                if (bHA > 0) {
                    if (f == InFlag.Pin) {
                        ii = addVertex(inters, ii, a);
                    }
                    aa++;
                    ai++;
                } else {
                    if (f == InFlag.Qin) {
                        ii = addVertex(inters, ii, b);
                    }
                    ba++;
                    bi++;
                }
            } else {
                if (aHB > 0) {
                    if (f == InFlag.Qin) {
                        ii = addVertex(inters, ii, b);
                    }
                    ba++;
                    bi++;
                } else {
                    if (f == InFlag.Pin) {
                        ii = addVertex(inters, ii, a);
                    }
                    aa++;
                    ai++;
                }
            }
            /* Quit when both adv. indices have cycled, or one has cycled twice. */
        } while ((aa < n || ba < m) && aa < 2 * n && ba < 2 * m);

        /* Deal with special cases: not implemented. */
        if (f == InFlag.Unknown) {
            return null;
        }
        return Arrays.copyOf(inters, ii);
    }

    private static int addVertex(float[] inters, int ii, Vector3f p) {
        if (ii > 0) {
            if (inters[ii - 3] == p.x && inters[ii - 2] == p.y && inters[ii - 1] == p.z)
                return ii;
            if (inters[0] == p.x && inters[1] == p.y && inters[2] == p.z)
                return ii;
        }
        inters[ii] = p.x;
        inters[ii + 1] = p.y;
        inters[ii + 2] = p.z;
        return ii + 3;
    }

    private static InFlag inOut(InFlag inflag, float aHB, float bHA) {
        if (aHB > 0) {
            return InFlag.Pin;
        } else if (bHA > 0) {
            return InFlag.Qin;
        }
        return inflag;
    }

    private static Intersection segSegInt(Vector3f a, Vector3f b, Vector3f c, Vector3f d, Vector3f p) {
        Optional<Pair<Float, Float>> i = intersectSegSeg2D(a, b, c, d);
        if (i.isPresent()) {
            float s = i.get().first;
            float t = i.get().second;
            if (s >= 0f && s <= 1f && t >= 0f && t <= 1f) {
                p.set(a).lerp(b, s);
                return Intersection.Single;
            }
        }
        return Intersection.None;
    }

    private static Intersection parallelInt(Vector3f a, Vector3f b, Vector3f c, Vector3f d, Vector3f p, Vector3f q) {
        if (between(a, b, c) && between(a, b, d)) {
            p.set(c);
            q.set(d);
            return Intersection.Overlap;
        }
        if (between(c, d, a) && between(c, d, b)) {
            p.set(a);
            q.set(b);
            return Intersection.Overlap;
        }
        if (between(a, b, c) && between(c, d, b)) {
            p.set(c);
            q.set(b);
            return Intersection.Overlap;
        }
        if (between(a, b, c) && between(c, d, a)) {
            p.set(c);
            q.set(a);
            return Intersection.Overlap;
        }
        if (between(a, b, d) && between(c, d, b)) {
            p.set(d);
            q.set(b);
            return Intersection.Overlap;
        }
        if (between(a, b, d) && between(c, d, a)) {
            p.set(d);
            q.set(a);
            return Intersection.Overlap;
        }
        return Intersection.None;
    }

    private static boolean between(Vector3f a, Vector3f b, Vector3f c) {
        if (Math.abs(a.x - b.x) > Math.abs(a.z - b.z)) {
            return ((a.x <= c.x) && (c.x <= b.x)) || ((a.x >= c.x) && (c.x >= b.x));
        } else {
            return ((a.z <= c.z) && (c.z <= b.z)) || ((a.z >= c.z) && (c.z >= b.z));
        }
    }
}
