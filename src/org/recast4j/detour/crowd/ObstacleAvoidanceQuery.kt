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
package org.recast4j.detour.crowd

import org.joml.Vector3f
import org.recast4j.Vectors
import org.recast4j.detour.crowd.debug.ObstacleAvoidanceDebugData
import kotlin.math.*

class ObstacleAvoidanceQuery(private val m_maxCircles: Int, maxSegments: Int) {
    class ObstacleCircle {
        /**
         * Position of the obstacle
         */
        val p: Vector3f = Vector3f()

        /**
         * Velocity of the obstacle
         */
        val vel: Vector3f = Vector3f()

        /**
         * Velocity of the obstacle
         */
        val dvel: Vector3f = Vector3f()

        /**
         * Radius of the obstacle
         */
        var rad = 0f

        /**
         * Use for side selection during sampling.
         */
        val dp: Vector3f = Vector3f()

        /**
         * Use for side selection during sampling.
         */
        val np: Vector3f = Vector3f()
    }

    class ObstacleSegment {
        /**
         * End points of the obstacle segment
         */
        val p: Vector3f = Vector3f()

        /**
         * End points of the obstacle segment
         */
        val q: Vector3f = Vector3f()
        var touch = false
    }

    class ObstacleAvoidanceParams {
        var velBias: Float
        var weightDesVel: Float
        var weightCurVel: Float
        var weightSide: Float
        var weightToi: Float
        var horizTime: Float
        var gridSize /// < grid
                : Int
        var adaptiveDivs /// < adaptive
                : Int
        var adaptiveRings /// < adaptive
                : Int
        var adaptiveDepth /// < adaptive
                : Int

        constructor() {
            velBias = 0.4f
            weightDesVel = 2f
            weightCurVel = 0.75f
            weightSide = 0.75f
            weightToi = 2.5f
            horizTime = 2.5f
            gridSize = 33
            adaptiveDivs = 7
            adaptiveRings = 2
            adaptiveDepth = 5
        }

        constructor(params: ObstacleAvoidanceParams) {
            velBias = params.velBias
            weightDesVel = params.weightDesVel
            weightCurVel = params.weightCurVel
            weightSide = params.weightSide
            weightToi = params.weightToi
            horizTime = params.horizTime
            gridSize = params.gridSize
            adaptiveDivs = params.adaptiveDivs
            adaptiveRings = params.adaptiveRings
            adaptiveDepth = params.adaptiveDepth
        }
    }

    private var m_params: ObstacleAvoidanceParams? = null
    private var m_invHorizTime = 0f
    private var m_invVmax = 0f
    val circles: Array<ObstacleCircle?> = arrayOfNulls(m_maxCircles)
    var circleCount = 0
    private val maxNumSegments: Int
    val segments: Array<ObstacleSegment?>
    var segmentCount: Int
    fun reset() {
        circleCount = 0
        segmentCount = 0
    }

    fun addCircle(pos: Vector3f, rad: Float, vel: Vector3f, dvel: Vector3f) {
        if (circleCount >= m_maxCircles) return
        val cir = circles[circleCount++]!!
        cir.p.set(pos)
        cir.rad = rad
        cir.vel.set(vel)
        cir.dvel.set(dvel)
    }

    fun addSegment(p: Vector3f, q: Vector3f) {
        if (segmentCount >= maxNumSegments) return
        val seg = segments[segmentCount++]!!
        seg.p.set(p)
        seg.q.set(q)
    }

    private fun prepare(pos: Vector3f, dvel: Vector3f) {
        // Prepare obstacles
        for (i in 0 until circleCount) {
            val cir = circles[i]

            // Side
            val pb: Vector3f = cir!!.p
            val orig = Vector3f()
            var dv: Vector3f
            Vectors.copy(cir.dp, Vectors.sub(pb, pos))
            cir.dp.normalize()
            dv = Vectors.sub(cir.dvel, dvel)
            val a: Float = Vectors.triArea2D(orig, cir.dp, dv)
            if (a < 0.01f) {
                cir.np.x = -cir.dp.z
                cir.np.z = cir.dp.x
            } else {
                cir.np.x = cir.dp.z
                cir.np.z = -cir.dp.x
            }
        }
        for (i in 0 until segmentCount) {
            val seg = segments[i]

            // Precalc if the agent is really close to the segment.
            val r = 0.01f
            val (first) = Vectors.distancePtSegSqr2D(pos, seg!!.p, seg.q)
            seg.touch = first < Vectors.sqr(r)
        }
    }

    fun sweepCircleCircle(c0: Vector3f, r0: Float, v: Vector3f, c1: Vector3f, r1: Float): SweepCircleCircleResult? {
        val EPS = 0.0001f
        val s: Vector3f = Vectors.sub(c1, c0)
        val r = r0 + r1
        val c: Float = Vectors.dot2D(s, s) - r * r
        var a: Float = Vectors.dot2D(v, v)
        if (a < EPS) return null // not moving
        // Overlap, calc time to exit.
        val b: Float = Vectors.dot2D(v, s)
        val d = b * b - a * c
        if (d < 0f) return null // no intersection.
        a = 1f / a
        val rd = sqrt(d).toFloat()
        return SweepCircleCircleResult((b - rd) * a, (b + rd) * a)
    }

    fun isectRaySeg(ap: Vector3f, u: Vector3f, bp: Vector3f, bq: Vector3f): Float {
        val v: Vector3f = Vectors.sub(bq, bp)
        val w: Vector3f = Vectors.sub(ap, bp)
        var d: Float = -Vectors.crossXZ(u, v)
        if (abs(d) < 1e-6f) return -1f
        d = 1f / d
        val t: Float = -Vectors.crossXZ(v, w) * d
        if (t < 0 || t > 1) return -1f
        val s: Float = -Vectors.crossXZ(u, w) * d
        return if (s < 0 || s > 1) -1f else t
    }

    /**
     * Calculate the collision penalty for a given velocity vector
     *
     * @param vcand      sampled velocity
     * @param dvel       desired velocity
     * @param minPenalty threshold penalty for early out
     */
    private fun processSample(
        vcand: Vector3f, cs: Float, pos: Vector3f, rad: Float, vel: Vector3f, dvel: Vector3f,
        minPenalty: Float, debug: ObstacleAvoidanceDebugData?
    ): Float {
        // penalty for straying away from the desired and current velocities
        val vpen: Float = m_params!!.weightDesVel * (Vectors.dist2D(vcand, dvel) * m_invVmax)
        val vcpen: Float = m_params!!.weightCurVel * (Vectors.dist2D(vcand, vel) * m_invVmax)

        // find the threshold hit time to bail out based on the early out penalty
        // (see how the penalty is calculated below to understnad)
        val minPen = minPenalty - vpen - vcpen
        val tThresold = (m_params!!.weightToi / minPen - 0.1f) * m_params!!.horizTime
        if (tThresold - m_params!!.horizTime > -Float.MIN_VALUE) return minPenalty // already too much

        // Find min time of impact and exit amongst all obstacles.
        var tmin = m_params!!.horizTime
        var side = 0f
        var nside = 0
        for (i in 0 until circleCount) {
            val cir = circles[i]

            // RVO
            var vab: Vector3f = Vector3f(vcand).mul(2f)
            vab = Vectors.sub(vab, vel)
            vab = Vectors.sub(vab, cir!!.vel)

            // Side
            side += Vectors.clamp(min(Vectors.dot2D(cir.dp, vab) * 0.5f + 0.5f, Vectors.dot2D(cir.np, vab) * 2), 0f, 1f)
            nside++
            val sres = sweepCircleCircle(pos, rad, vab, cir.p, cir.rad) ?: continue
            var htmin = sres.htmin
            val htmax = sres.htmax

            // Handle overlapping obstacles.
            if (htmin < 0f && htmax > 0f) {
                // Avoid more when overlapped.
                htmin = -htmin * 0.5f
            }
            if (htmin >= 0f) {
                // The closest obstacle is somewhere ahead of us, keep track of nearest obstacle.
                if (htmin < tmin) {
                    tmin = htmin
                    if (tmin < tThresold) return minPenalty
                }
            }
        }
        for (i in 0 until segmentCount) {
            val seg = segments[i]
            var htmin: Float
            if (seg!!.touch) {
                // Special case when the agent is very close to the segment.
                val sdir: Vector3f = Vectors.sub(seg.q, seg.p)
                val snorm = Vector3f()
                snorm.x = -sdir.z
                snorm.z = sdir.x
                // If the velocity is pointing towards the segment, no collision.
                if (Vectors.dot2D(snorm, vcand) < 0f) continue
                // Else immediate collision.
                htmin = 0f
            } else {
                val ires = isectRaySeg(pos, vcand, seg.p, seg.q)
                if (ires < 0f) continue
                htmin = ires
            }

            // Avoid less when facing walls.
            htmin *= 2f

            // The closest obstacle is somewhere ahead of us, keep track of nearest obstacle.
            if (htmin < tmin) {
                tmin = htmin
                if (tmin < tThresold) return minPenalty
            }
        }

        // Normalize side bias, to prevent it dominating too much.
        if (nside != 0) side /= nside.toFloat()
        val spen = m_params!!.weightSide * side
        val tpen = m_params!!.weightToi * (1f / (0.1f + tmin * m_invHorizTime))
        val penalty = vpen + vcpen + spen + tpen
        // Store different penalties for debug viewing
        if (debug != null) debug.addSample(vcand, cs, penalty, vpen, vcpen, spen, tpen)
        return penalty
    }

    fun sampleVelocityGrid(
        pos: Vector3f, rad: Float, vmax: Float, vel: Vector3f, dvel: Vector3f,
        params: ObstacleAvoidanceParams?, debug: ObstacleAvoidanceDebugData?
    ): Pair<Int, Vector3f> {
        prepare(pos, dvel)
        m_params = params
        m_invHorizTime = 1f / m_params!!.horizTime
        m_invVmax = if (vmax > 0) 1f / vmax else Float.MAX_VALUE
        val nvel = Vector3f()
        Vectors.set(nvel, 0f, 0f, 0f)
        if (debug != null) debug.reset()
        val cvx: Float = dvel.x * m_params!!.velBias
        val cvz: Float = dvel.z * m_params!!.velBias
        val cs = vmax * 2 * (1 - m_params!!.velBias) / (m_params!!.gridSize - 1)
        val half = (m_params!!.gridSize - 1) * cs * 0.5f
        var minPenalty = Float.MAX_VALUE
        var ns = 0
        for (y in 0 until m_params!!.gridSize) {
            for (x in 0 until m_params!!.gridSize) {
                val vcand = Vector3f(cvx + x * cs - half, 0f, cvz + y * cs - half)
                if (Vectors.sqr(vcand.x) + Vectors.sqr(vcand.z) > Vectors.sqr(vmax + cs / 2)) continue
                val penalty = processSample(vcand, cs, pos, rad, vel, dvel, minPenalty, debug)
                ns++
                if (penalty < minPenalty) {
                    minPenalty = penalty
                    Vectors.copy(nvel, vcand)
                }
            }
        }
        return Pair<Int, Vector3f>(ns, nvel)
    }

    // vector normalization that ignores the y-component.
    fun dtNormalize2D(v: FloatArray) {
        var d = sqrt((v[0] * v[0] + v[2] * v[2])).toFloat()
        if (d == 0f) return
        d = 1f / d
        v[0] *= d
        v[2] *= d
    }

    init {
        for (i in 0 until m_maxCircles) {
            circles[i] = ObstacleCircle()
        }
        maxNumSegments = maxSegments
        segmentCount = 0
        segments = arrayOfNulls(maxNumSegments)
        for (i in 0 until maxNumSegments) {
            segments[i] = ObstacleSegment()
        }
    }

    fun sampleVelocityAdaptive(
        pos: Vector3f, rad: Float, vmax: Float, vel: Vector3f,
        dvel: Vector3f, params: ObstacleAvoidanceParams?, debug: ObstacleAvoidanceDebugData?
    ): Pair<Int, Vector3f> {
        prepare(pos, dvel)
        m_params = params
        m_invHorizTime = 1f / m_params!!.horizTime
        m_invVmax = if (vmax > 0) 1f / vmax else Float.MAX_VALUE
        val nvel = Vector3f()
        debug?.reset()

        // Build sampling pattern aligned to desired velocity.
        val pat = FloatArray((DT_MAX_PATTERN_DIVS * DT_MAX_PATTERN_RINGS + 1) * 2)
        val ndivs = m_params!!.adaptiveDivs
        val nrings = m_params!!.adaptiveRings
        val depth = m_params!!.adaptiveDepth
        val nd: Int = Vectors.clamp(ndivs, 1, DT_MAX_PATTERN_DIVS)
        val nr: Int = Vectors.clamp(nrings, 1, DT_MAX_PATTERN_RINGS)
        val da = 1f / nd * DT_PI * 2
        val ca = cos(da)
        val sa = sin(da)

        // desired direction
        val ddir = floatArrayOf(dvel.x, dvel.y, dvel.z, 0f, 0f, 0f)
        dtNormalize2D(ddir)
        val rotated: Vector3f = Vector3f(ddir).rotateY(da * 0.5f) // rotated by da/2
        ddir[3] = rotated.x
        ddir[4] = rotated.y
        ddir[5] = rotated.z
        var npat = 1
        for (j in 0 until nr) {
            val r = (nr - j).toFloat() / nr.toFloat()
            pat[npat * 2] = ddir[j % 2 * 3] * r
            pat[npat * 2 + 1] = ddir[j % 2 * 3 + 2] * r
            var last1 = npat * 2
            var last2 = last1
            npat++
            var i = 1
            while (i < nd - 1) {

                // get next point on the "right" (rotate CW)
                pat[npat * 2] = pat[last1] * ca + pat[last1 + 1] * sa
                pat[npat * 2 + 1] = -pat[last1] * sa + pat[last1 + 1] * ca
                // get next point on the "left" (rotate CCW)
                pat[npat * 2 + 2] = pat[last2] * ca - pat[last2 + 1] * sa
                pat[npat * 2 + 3] = pat[last2] * sa + pat[last2 + 1] * ca
                last1 = npat * 2
                last2 = last1 + 2
                npat += 2
                i += 2
            }
            if (nd and 1 == 0) {
                pat[npat * 2 + 2] = pat[last2] * ca - pat[last2 + 1] * sa
                pat[npat * 2 + 3] = pat[last2] * sa + pat[last2 + 1] * ca
                npat++
            }
        }

        // Start sampling.
        var cr = vmax * (1f - m_params!!.velBias)
        val res = Vector3f(dvel.x * m_params!!.velBias, 0f, dvel.z * m_params!!.velBias)
        var ns = 0
        for (k in 0 until depth) {
            var minPenalty = Float.MAX_VALUE
            val bVel = Vector3f()
            for (i in 0 until npat) {
                val vcand = Vector3f(res.x + pat[i * 2] * cr, 0f, res.z + pat[i * 2 + 1] * cr)
                if (Vectors.sqr(vcand.x) + Vectors.sqr(vcand.z) > Vectors.sqr(vmax + 0.001f)) continue
                val penalty = processSample(vcand, cr / 10, pos, rad, vel, dvel, minPenalty, debug)
                ns++
                if (penalty < minPenalty) {
                    minPenalty = penalty
                    bVel.set(vcand)
                }
            }
            res.set(bVel)
            cr *= 0.5f
        }
        nvel.set(res)
        return Pair(ns, nvel)
    }

    companion object {
        /** Max numver of adaptive divs.  */
        private const val DT_MAX_PATTERN_DIVS = 32

        /** Max number of adaptive rings.  */
        private const val DT_MAX_PATTERN_RINGS = 4
        const val DT_PI = 3.1415927f
    }
}