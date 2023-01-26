package org.recast4j.detour.extras.jumplink

import org.joml.Vector3f

class EdgeSampler(edge: Edge, val trajectory: Trajectory) {
    val start = GroundSegment()
    val end = ArrayList<GroundSegment>()
    val ax = Vector3f()
    val ay = Vector3f()
    val az = Vector3f()

    init {
        ax.set(edge.b).sub(edge.a).normalize()
        az.set(ax.z, 0f, -ax.x).normalize()
        ay[0f, 1f] = 0f
    }
}