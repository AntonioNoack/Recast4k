package org.recast4j.detour.extras.jumplink

import org.joml.Vector3f

interface Trajectory {
    companion object {
        fun lerp(a: Float, b: Float, t: Float) = a + (b - a) * t
    }

    fun apply(start: Vector3f, end: Vector3f, u: Float): Vector3f
}