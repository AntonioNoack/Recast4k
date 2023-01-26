package org.recast4j.detour.extras.jumplink

import org.joml.Vector3f
import org.recast4j.Vectors
import org.recast4j.recast.Heightfield
import org.recast4j.recast.Span
import kotlin.math.*

internal class TrajectorySampler {
    fun sample(jlc: JumpLinkBuilderConfig, heightfield: Heightfield, es: EdgeSampler) {
        val l = es.start.samples!!.size
        for (i in 0 until l) {
            val s0 = es.start.samples!![i]
            for (end in es.end) {
                val s1 = end.samples!![i]
                if (!s0.validHeight || !s1.validHeight) {
                    continue
                }
                if (!sampleTrajectory(jlc, heightfield, s0.p, s1.p, es.trajectory)) {
                    continue
                }
                s0.validTrajectory = true
                s1.validTrajectory = true
            }
        }
    }

    private fun sampleTrajectory(
        acfg: JumpLinkBuilderConfig,
        solid: Heightfield,
        pa: Vector3f,
        pb: Vector3f,
        tra: Trajectory
    ): Boolean {
        val cs = min(acfg.cellSize, acfg.cellHeight)
        val d = Vectors.dist2D(pa, pb) + abs(pa.y - pb.y)
        val l = max(2, ceil((d / cs)).toInt())
        for (i in 0 until l) {
            val u = i.toFloat() / (l - 1).toFloat()
            val p = tra.apply(pa, pb, u)
            if (checkHeightfieldCollision(solid, p.x, p.y + acfg.groundTolerance, p.y + acfg.agentHeight, p.z)) {
                return false
            }
        }
        return true
    }

    private fun checkHeightfieldCollision(solid: Heightfield, x: Float, ymin: Float, ymax: Float, z: Float): Boolean {
        val cellSize = solid.cellSize
        val origin = solid.bmin
        val ix = floor(((x - origin.x) / cellSize)).toInt()
        val iz = floor(((z - origin.z) / cellSize)).toInt()
        val w = solid.width
        val h = solid.height
        if (ix < 0 || iz < 0 || ix > w || iz > h) {
            return false
        }
        var s: Span? = solid.spans[ix + iz * w] ?: return false
        while (s != null) {
            val cellHeight = solid.cellHeight
            val syMin = origin.y + s.min * cellHeight
            val syMax = origin.y + s.max * cellHeight
            if (Vectors.overlapRange(ymin, ymax, syMin, syMax)) {
                return true
            }
            s = s.next
        }
        return false
    }
}