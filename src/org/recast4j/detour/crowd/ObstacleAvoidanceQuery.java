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
package org.recast4j.detour.crowd;

import org.joml.Vector3f;
import org.recast4j.Pair;
import org.recast4j.detour.crowd.debug.ObstacleAvoidanceDebugData;

import static org.joml.Math.clamp;
import static org.recast4j.Vectors.*;

public class ObstacleAvoidanceQuery {

    /** Max numver of adaptive divs. */
    private static final int DT_MAX_PATTERN_DIVS = 32;
    /** Max number of adaptive rings. */
    private static final int DT_MAX_PATTERN_RINGS = 4;

    static class ObstacleCircle {
        /**
         * Position of the obstacle
         */
        final Vector3f p = new Vector3f();
        /**
         * Velocity of the obstacle
         */
        final Vector3f vel = new Vector3f();
        /**
         * Velocity of the obstacle
         */
        final Vector3f dvel = new Vector3f();
        /**
         * Radius of the obstacle
         */
        float rad;
        /**
         * Use for side selection during sampling.
         */
        final Vector3f dp = new Vector3f();
        /**
         * Use for side selection during sampling.
         */
        final Vector3f np = new Vector3f();
    }

    static class ObstacleSegment {
        /**
         * End points of the obstacle segment
         */
        final Vector3f p = new Vector3f();
        /**
         * End points of the obstacle segment
         */
        final Vector3f q = new Vector3f();
        boolean touch;
    }

    public static class ObstacleAvoidanceParams {
        public float velBias;
        public float weightDesVel;
        public float weightCurVel;
        public float weightSide;
        public float weightToi;
        public float horizTime;
        public int gridSize; /// < grid
        public int adaptiveDivs; /// < adaptive
        public int adaptiveRings; /// < adaptive
        public int adaptiveDepth; /// < adaptive

        public ObstacleAvoidanceParams() {
            velBias = 0.4f;
            weightDesVel = 2f;
            weightCurVel = 0.75f;
            weightSide = 0.75f;
            weightToi = 2.5f;
            horizTime = 2.5f;
            gridSize = 33;
            adaptiveDivs = 7;
            adaptiveRings = 2;
            adaptiveDepth = 5;
        }

        public ObstacleAvoidanceParams(ObstacleAvoidanceParams params) {
            velBias = params.velBias;
            weightDesVel = params.weightDesVel;
            weightCurVel = params.weightCurVel;
            weightSide = params.weightSide;
            weightToi = params.weightToi;
            horizTime = params.horizTime;
            gridSize = params.gridSize;
            adaptiveDivs = params.adaptiveDivs;
            adaptiveRings = params.adaptiveRings;
            adaptiveDepth = params.adaptiveDepth;
        }
    }

    private ObstacleAvoidanceParams m_params;
    private float m_invHorizTime;
    private float m_invVmax;

    private final int m_maxCircles;
    public final ObstacleCircle[] circles;
    public int circleCount;

    private final int maxNumSegments;
    public final ObstacleSegment[] segments;
    public int segmentCount;

    public ObstacleAvoidanceQuery(int maxCircles, int maxSegments) {
        m_maxCircles = maxCircles;
        circleCount = 0;
        circles = new ObstacleCircle[m_maxCircles];
        for (int i = 0; i < m_maxCircles; i++) {
            circles[i] = new ObstacleCircle();
        }
        maxNumSegments = maxSegments;
        segmentCount = 0;
        segments = new ObstacleSegment[maxNumSegments];
        for (int i = 0; i < maxNumSegments; i++) {
            segments[i] = new ObstacleSegment();
        }
    }

    public void reset() {
        circleCount = 0;
        segmentCount = 0;
    }

    public void addCircle(Vector3f pos, float rad, Vector3f vel, Vector3f dvel) {
        if (circleCount >= m_maxCircles)
            return;

        ObstacleCircle cir = circles[circleCount++];
        copy(cir.p, pos);
        cir.rad = rad;
        copy(cir.vel, vel);
        copy(cir.dvel, dvel);
    }

    public void addSegment(Vector3f p, Vector3f q) {
        if (segmentCount >= maxNumSegments) return;
        ObstacleSegment seg = segments[segmentCount++];
        copy(seg.p, p);
        copy(seg.q, q);
    }

    private void prepare(Vector3f pos, Vector3f dvel) {
        // Prepare obstacles
        for (int i = 0; i < circleCount; i++) {
            ObstacleCircle cir = circles[i];

            // Side
            Vector3f pb = cir.p;

            Vector3f orig = new Vector3f();
            Vector3f dv;
            copy(cir.dp, sub(pb, pos));
            cir.dp.normalize();
            dv = sub(cir.dvel, dvel);

            float a = triArea2D(orig, cir.dp, dv);
            if (a < 0.01f) {
                cir.np.x = -cir.dp.z;
                cir.np.z = cir.dp.x;
            } else {
                cir.np.x = cir.dp.z;
                cir.np.z = -cir.dp.x;
            }
        }

        for (int i = 0; i < segmentCount; ++i) {
            ObstacleSegment seg = segments[i];

            // Precalc if the agent is really close to the segment.
            float r = 0.01f;
            Pair<Float, Float> dt = distancePtSegSqr2D(pos, seg.p, seg.q);
            seg.touch = dt.first < sqr(r);
        }
    }

    SweepCircleCircleResult sweepCircleCircle(Vector3f c0, float r0, Vector3f v, Vector3f c1, float r1) {
        final float EPS = 0.0001f;
        Vector3f s = sub(c1, c0);
        float r = r0 + r1;
        float c = dot2D(s, s) - r * r;
        float a = dot2D(v, v);
        if (a < EPS) return null; // not moving
        // Overlap, calc time to exit.
        float b = dot2D(v, s);
        float d = b * b - a * c;
        if (d < 0f) return null; // no intersection.
        a = 1f / a;
        float rd = (float) Math.sqrt(d);
        return new SweepCircleCircleResult((b - rd) * a, (b + rd) * a);
    }

    float isectRaySeg(Vector3f ap, Vector3f u, Vector3f bp, Vector3f bq) {
        Vector3f v = sub(bq, bp);
        Vector3f w = sub(ap, bp);
        float d = -crossXZ(u, v);
        if (Math.abs(d) < 1e-6f)
            return -1f;
        d = 1f / d;
        float t = -crossXZ(v, w) * d;
        if (t < 0 || t > 1)
            return -1f;
        float s = -crossXZ(u, w) * d;
        if (s < 0 || s > 1)
            return -1f;
        return t;
    }

    /**
     * Calculate the collision penalty for a given velocity vector
     *
     * @param vcand      sampled velocity
     * @param dvel       desired velocity
     * @param minPenalty threshold penalty for early out
     */
    private float processSample(Vector3f vcand, float cs, Vector3f pos, float rad, Vector3f vel, Vector3f dvel,
                                float minPenalty, ObstacleAvoidanceDebugData debug) {
        // penalty for straying away from the desired and current velocities
        float vpen = m_params.weightDesVel * (dist2D(vcand, dvel) * m_invVmax);
        float vcpen = m_params.weightCurVel * (dist2D(vcand, vel) * m_invVmax);

        // find the threshold hit time to bail out based on the early out penalty
        // (see how the penalty is calculated below to understnad)
        float minPen = minPenalty - vpen - vcpen;
        float tThresold = (m_params.weightToi / minPen - 0.1f) * m_params.horizTime;
        if (tThresold - m_params.horizTime > -Float.MIN_VALUE)
            return minPenalty; // already too much

        // Find min time of impact and exit amongst all obstacles.
        float tmin = m_params.horizTime;
        float side = 0;
        int nside = 0;

        for (int i = 0; i < circleCount; ++i) {
            ObstacleCircle cir = circles[i];

            // RVO
            Vector3f vab = new Vector3f(vcand).mul(2f);
            vab = sub(vab, vel);
            vab = sub(vab, cir.vel);

            // Side
            side += clamp(Math.min(dot2D(cir.dp, vab) * 0.5f + 0.5f, dot2D(cir.np, vab) * 2), 0f, 1f);
            nside++;

            SweepCircleCircleResult sres = sweepCircleCircle(pos, rad, vab, cir.p, cir.rad);
            if (sres == null) continue;
            float htmin = sres.htmin, htmax = sres.htmax;

            // Handle overlapping obstacles.
            if (htmin < 0f && htmax > 0f) {
                // Avoid more when overlapped.
                htmin = -htmin * 0.5f;
            }

            if (htmin >= 0f) {
                // The closest obstacle is somewhere ahead of us, keep track of nearest obstacle.
                if (htmin < tmin) {
                    tmin = htmin;
                    if (tmin < tThresold)
                        return minPenalty;
                }
            }
        }

        for (int i = 0; i < segmentCount; ++i) {
            ObstacleSegment seg = segments[i];
            float htmin;

            if (seg.touch) {
                // Special case when the agent is very close to the segment.
                Vector3f sdir = sub(seg.q, seg.p);
                Vector3f snorm = new Vector3f();
                snorm.x = -sdir.z;
                snorm.z = sdir.x;
                // If the velocity is pointing towards the segment, no collision.
                if (dot2D(snorm, vcand) < 0f)
                    continue;
                // Else immediate collision.
                htmin = 0f;
            } else {
                float ires = isectRaySeg(pos, vcand, seg.p, seg.q);
                if (ires < 0f) continue;
                htmin = ires;
            }

            // Avoid less when facing walls.
            htmin *= 2f;

            // The closest obstacle is somewhere ahead of us, keep track of nearest obstacle.
            if (htmin < tmin) {
                tmin = htmin;
                if (tmin < tThresold)
                    return minPenalty;
            }
        }

        // Normalize side bias, to prevent it dominating too much.
        if (nside != 0)
            side /= nside;

        float spen = m_params.weightSide * side;
        float tpen = m_params.weightToi * (1f / (0.1f + tmin * m_invHorizTime));

        float penalty = vpen + vcpen + spen + tpen;
        // Store different penalties for debug viewing
        if (debug != null)
            debug.addSample(vcand, cs, penalty, vpen, vcpen, spen, tpen);

        return penalty;
    }

    @SuppressWarnings("unused")
    public Pair<Integer, Vector3f> sampleVelocityGrid(Vector3f pos, float rad, float vmax, Vector3f vel, Vector3f dvel,
                                                      ObstacleAvoidanceParams params, ObstacleAvoidanceDebugData debug) {
        prepare(pos, dvel);
        m_params = params;
        m_invHorizTime = 1f / m_params.horizTime;
        m_invVmax = vmax > 0 ? 1f / vmax : Float.MAX_VALUE;

        Vector3f nvel = new Vector3f();
        set(nvel, 0f, 0f, 0f);

        if (debug != null)
            debug.reset();

        float cvx = dvel.x * m_params.velBias;
        float cvz = dvel.z * m_params.velBias;
        float cs = vmax * 2 * (1 - m_params.velBias) / (m_params.gridSize - 1);
        float half = (m_params.gridSize - 1) * cs * 0.5f;

        float minPenalty = Float.MAX_VALUE;
        int ns = 0;

        for (int y = 0; y < m_params.gridSize; ++y) {
            for (int x = 0; x < m_params.gridSize; ++x) {
                Vector3f vcand = new Vector3f(cvx + x * cs - half, 0f, cvz + y * cs - half);
                if (sqr(vcand.x) + sqr(vcand.z) > sqr(vmax + cs / 2))
                    continue;

                float penalty = processSample(vcand, cs, pos, rad, vel, dvel, minPenalty, debug);
                ns++;
                if (penalty < minPenalty) {
                    minPenalty = penalty;
                    copy(nvel, vcand);
                }
            }
        }

        return new Pair<>(ns, nvel);
    }

    // vector normalization that ignores the y-component.
    void dtNormalize2D(float[] v) {
        float d = (float) Math.sqrt(v[0] * v[0] + v[2] * v[2]);
        if (d == 0)
            return;
        d = 1f / d;
        v[0] *= d;
        v[2] *= d;
    }

    static final float DT_PI = 3.14159265f;

    public Pair<Integer, Vector3f> sampleVelocityAdaptive(Vector3f pos, float rad, float vmax, Vector3f vel,
                                                          Vector3f dvel, ObstacleAvoidanceParams params, ObstacleAvoidanceDebugData debug) {
        prepare(pos, dvel);
        m_params = params;
        m_invHorizTime = 1f / m_params.horizTime;
        m_invVmax = vmax > 0 ? 1f / vmax : Float.MAX_VALUE;

        Vector3f nvel = new Vector3f();

        if (debug != null)
            debug.reset();

        // Build sampling pattern aligned to desired velocity.
        float[] pat = new float[(DT_MAX_PATTERN_DIVS * DT_MAX_PATTERN_RINGS + 1) * 2];

        int ndivs = m_params.adaptiveDivs;
        int nrings = m_params.adaptiveRings;
        int depth = m_params.adaptiveDepth;

        int nd = clamp(ndivs, 1, DT_MAX_PATTERN_DIVS);
        int nr = clamp(nrings, 1, DT_MAX_PATTERN_RINGS);
        float da = (1f / nd) * DT_PI * 2;
        float ca = (float) Math.cos(da);
        float sa = (float) Math.sin(da);

        // desired direction
        float[] ddir = {dvel.x, dvel.y, dvel.z, 0f, 0f, 0f};
        dtNormalize2D(ddir);
        Vector3f rotated = new Vector3f(ddir).rotateY(da * 0.5f); // rotated by da/2
        ddir[3] = rotated.x;
        ddir[4] = rotated.y;
        ddir[5] = rotated.z;

        int npat = 1;
        for (int j = 0; j < nr; ++j) {
            float r = (float) (nr - j) / (float) nr;
            pat[npat * 2] = ddir[(j % 2) * 3] * r;
            pat[npat * 2 + 1] = ddir[(j % 2) * 3 + 2] * r;
            int last1 = npat * 2;
            int last2 = last1;
            npat++;

            for (int i = 1; i < nd - 1; i += 2) {
                // get next point on the "right" (rotate CW)
                pat[npat * 2] = pat[last1] * ca + pat[last1 + 1] * sa;
                pat[npat * 2 + 1] = -pat[last1] * sa + pat[last1 + 1] * ca;
                // get next point on the "left" (rotate CCW)
                pat[npat * 2 + 2] = pat[last2] * ca - pat[last2 + 1] * sa;
                pat[npat * 2 + 3] = pat[last2] * sa + pat[last2 + 1] * ca;

                last1 = npat * 2;
                last2 = last1 + 2;
                npat += 2;
            }

            if ((nd & 1) == 0) {
                pat[npat * 2 + 2] = pat[last2] * ca - pat[last2 + 1] * sa;
                pat[npat * 2 + 3] = pat[last2] * sa + pat[last2 + 1] * ca;
                npat++;
            }
        }

        // Start sampling.
        float cr = vmax * (1f - m_params.velBias);
        Vector3f res = new Vector3f(dvel.x * m_params.velBias, 0, dvel.z * m_params.velBias);
        int ns = 0;
        for (int k = 0; k < depth; ++k) {
            float minPenalty = Float.MAX_VALUE;
            Vector3f bVel = new Vector3f();
            for (int i = 0; i < npat; ++i) {
                Vector3f vcand = new Vector3f(res.x + pat[i * 2] * cr, 0f, res.z + pat[i * 2 + 1] * cr);
                if (sqr(vcand.x) + sqr(vcand.z) > sqr(vmax + 0.001f))
                    continue;

                float penalty = processSample(vcand, cr / 10, pos, rad, vel, dvel, minPenalty, debug);
                ns++;
                if (penalty < minPenalty) {
                    minPenalty = penalty;
                    bVel.set(vcand);
                }
            }

            res.set(bVel);

            cr *= 0.5f;
        }
        nvel.set(res);

        return new Pair<>(ns, nvel);
    }
}
