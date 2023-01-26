package org.recast4j.detour.extras.jumplink

import java.util.*
import kotlin.math.abs

internal class JumpSegmentBuilder {
    fun build(cfg: JumpLinkBuilderConfig, es: EdgeSampler): Array<JumpSegment> {
        val n = es.end[0].samples!!.size
        val sampleGrid = Array(n) { IntArray(es.end.size) }
        for (j in es.end.indices) {
            for (i in 0 until n) {
                sampleGrid[i][j] = -1
            }
        }
        // Fill connected regions
        var region = 0
        for (j in es.end.indices) {
            for (i in 0 until n) {
                if (sampleGrid[i][j] == -1) {
                    val p = es.end[j].samples!![i]
                    if (!p.validTrajectory) {
                        sampleGrid[i][j] = -2
                    } else {
                        val queue: Deque<IntArray> = LinkedList()
                        queue.add(intArrayOf(i, j))
                        fill(es, sampleGrid, queue, cfg.agentClimb, region)
                        region++
                    }
                }
            }
        }
        val jumpSegments = Array(region) { JumpSegment() }
        // Find longest segments per region
        for (j in es.end.indices) {
            var l = 0
            var r = -2
            for (i in 0 until n + 1) {
                if (i == n || sampleGrid[i][j] != r) {
                    if (r >= 0) {
                        if (jumpSegments[r].samples < l) {
                            jumpSegments[r].samples = l
                            jumpSegments[r].startSample = i - l
                            jumpSegments[r].groundSegment = j
                        }
                    }
                    if (i < n) {
                        r = sampleGrid[i][j]
                    }
                    l = 1
                } else {
                    l++
                }
            }
        }
        return jumpSegments
    }

    private fun fill(
        es: EdgeSampler,
        sampleGrid: Array<IntArray>,
        queue: Deque<IntArray>,
        agentClimb: Float,
        region: Int
    ) {
        while (!queue.isEmpty()) {
            val ij = queue.poll()
            val i = ij[0]
            val j = ij[1]
            if (sampleGrid[i][j] == -1) {
                val p = es.end[j].samples!![i]
                sampleGrid[i][j] = region
                val h = p.p.y
                if (i < sampleGrid.size - 1) {
                    addNeighbour(es, queue, agentClimb, h, i + 1, j)
                }
                if (i > 0) {
                    addNeighbour(es, queue, agentClimb, h, i - 1, j)
                }
                if (j < sampleGrid[0].size - 1) {
                    addNeighbour(es, queue, agentClimb, h, i, j + 1)
                }
                if (j > 0) {
                    addNeighbour(es, queue, agentClimb, h, i, j - 1)
                }
            }
        }
    }

    private fun addNeighbour(es: EdgeSampler, queue: Deque<IntArray>, agentClimb: Float, h: Float, i: Int, j: Int) {
        val q = es.end[j].samples!![i]
        if (q.validTrajectory && abs(q.p.y - h) < agentClimb) {
            queue.add(intArrayOf(i, j))
        }
    }
}