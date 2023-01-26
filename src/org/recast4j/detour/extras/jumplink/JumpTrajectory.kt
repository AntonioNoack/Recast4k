package org.recast4j.detour.extras.jumplink

import org.joml.Vector3f
import kotlin.math.sqrt

class JumpTrajectory(private val jumpHeight: Float) : Trajectory {
    override fun apply(start: Vector3f, end: Vector3f, u: Float): Vector3f {
        return Vector3f(
            lerp(start.x, end.x, u),
            interpolateHeight(start.y, end.y, u),
            lerp(start.z, end.z, u)
        )
    }

    private fun interpolateHeight(ys: Float, ye: Float, u: Float): Float {
        if (u == 0f) {
            return ys
        } else if (u == 1f) {
            return ye
        }
        val h1: Float
        val h2: Float
        if (ys >= ye) { // jump down
            h1 = jumpHeight
            h2 = jumpHeight + ys - ye
        } else { // jump up
            h1 = jumpHeight + ys - ye
            h2 = jumpHeight
        }
        val t = (sqrt(h1) / (sqrt(h2) + sqrt(h1)))
        if (u <= t) {
            val v = 1f - u / t
            return ys + h1 - h1 * v * v
        }
        val v = (u - t) / (1f - t)
        return ys + h1 - h2 * v * v
    }

    override fun lerp(f: Float, g: Float, u: Float): Float {
        return u * g + (1f - u) * f
    }
}