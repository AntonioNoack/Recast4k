package org.recast4j.detour.extras.jumplink

import org.joml.Vector3f
import org.recast4j.Vectors
import kotlin.math.ceil
import kotlin.math.max

internal abstract class AbstractGroundSampler : GroundSampler {

    protected fun sampleGround(
        cfg: JumpLinkBuilderConfig, es: EdgeSampler,
        heightFunc: (Vector3f, Float) -> Pair<Boolean, Float>
    ) {
        val cellSize = cfg.cellSize
        val dist = es.start.p.distance(es.start.q)
        val numSamples = max(2, ceil((dist / cellSize)).toInt())
        sampleGroundSegment(heightFunc, es.start, numSamples)
        for (end in es.end) {
            sampleGroundSegment(heightFunc, end, numSamples)
        }
    }

    private fun sampleGroundSegment(
        heightFunc: (Vector3f, Float) -> Pair<Boolean, Float>,
        seg: GroundSegment,
        numSamples: Int
    ) {
        seg.samples = Array(numSamples) { GroundSample() }
        for (i in 0 until numSamples) {
            val u = i / (numSamples - 1).toFloat()
            val s = seg.samples!![i]
            val pt = Vectors.lerp(seg.p, seg.q, u)
            val (first, second) = heightFunc(pt, seg.height)
            s.p.set(pt)
            s.p.y = second
            if (!first) continue
            s.validHeight = true
        }
    }
}