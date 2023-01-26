package org.recast4j.detour.extras.jumplink

import org.recast4j.Vectors
import org.recast4j.recast.RecastBuilder.RecastBuilderResult
import java.util.*
import kotlin.math.min

class JumpLinkBuilder(private val results: List<RecastBuilderResult>) {

    private val edgeExtractor = EdgeExtractor()
    private val edgeSamplerFactory = EdgeSamplerFactory()
    private val groundSampler: GroundSampler = NavMeshGroundSampler()
    private val trajectorySampler = TrajectorySampler()
    private val jumpSegmentBuilder = JumpSegmentBuilder()

    val edges = results.map { r: RecastBuilderResult -> edgeExtractor.extractEdges(r.mesh) }

    fun build(cfg: JumpLinkBuilderConfig, type: JumpLinkType): List<JumpLink> {
        val links: MutableList<JumpLink> = ArrayList()
        for (tile in results.indices) {
            val edges = edges[tile]
            for (edge in edges) {
                links.addAll(processEdge(cfg, results[tile], type, edge))
            }
        }
        return links
    }

    private fun processEdge(
        cfg: JumpLinkBuilderConfig,
        result: RecastBuilderResult,
        type: JumpLinkType,
        edge: Edge
    ): List<JumpLink> {
        val es = edgeSamplerFactory[cfg, type, edge]
        groundSampler.sample(cfg, result, es)
        trajectorySampler.sample(cfg, result.solidHeightField, es)
        val jumpSegments = jumpSegmentBuilder.build(cfg, es)
        return buildJumpLinks(cfg, es, jumpSegments)
    }

    private fun buildJumpLinks(
        acfg: JumpLinkBuilderConfig,
        es: EdgeSampler,
        jumpSegments: Array<JumpSegment>
    ): List<JumpLink> {
        val links: MutableList<JumpLink> = ArrayList()
        for (js in jumpSegments) {
            val sp = es.start.samples!![js.startSample].p
            val sq = es.start.samples!![js.startSample + js.samples - 1].p
            val end = es.end[js.groundSegment]
            val ep = end.samples!![js.startSample].p
            val eq = end.samples!![js.startSample + js.samples - 1].p
            val d = min(Vectors.dist2DSqr(sp, sq), Vectors.dist2DSqr(ep, eq))
            if (d >= 4 * acfg.agentRadius * acfg.agentRadius) {
                val link = JumpLink()
                links.add(link)
                link.startSamples = (es.start.samples!!).copyOfRange(js.startSample, js.startSample + js.samples)
                link.endSamples = (end.samples!!).copyOfRange(js.startSample, js.startSample + js.samples)
                link.start = es.start
                link.end = end
                link.trajectory = es.trajectory
                for (j in 0 until link.numSpines) {
                    val u = j.toFloat() / (link.numSpines - 1)
                    Vectors.copy(link.spine0, j * 3, es.trajectory.apply(sp, ep, u))
                    Vectors.copy(link.spine1, j * 3, es.trajectory.apply(sq, eq, u))
                }
            }
        }
        return links
    }
}