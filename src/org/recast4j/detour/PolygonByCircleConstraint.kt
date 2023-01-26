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
package org.recast4j.detour

import org.joml.Vector3f
import org.recast4j.Vectors
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

interface PolygonByCircleConstraint {
    fun apply(polyVertices: FloatArray, circleCenter: Vector3f, radius: Float): FloatArray?
    class NoOpPolygonByCircleConstraint : PolygonByCircleConstraint {
        override fun apply(polyVertices: FloatArray, circleCenter: Vector3f, radius: Float) = polyVertices
    }

    /**
     * Calculate the intersection between a polygon and a circle. A dodecagon is used as an approximation of the circle.
     */
    class StrictPolygonByCircleConstraint : PolygonByCircleConstraint {
        override fun apply(polyVertices: FloatArray, circleCenter: Vector3f, radius: Float): FloatArray? {
            val radiusSqr = radius * radius
            var outsideVertex = -1
            var pv = 0
            while (pv < polyVertices.size) {
                if (Vectors.dist2DSqr(circleCenter, polyVertices, pv) > radiusSqr) {
                    outsideVertex = pv
                    break
                }
                pv += 3
            }
            if (outsideVertex == -1) {
                // polygon inside circle
                return polyVertices
            }
            val circle = circle(circleCenter, radius)
            val intersection = ConvexConvexIntersection.intersect(polyVertices, circle)
            return if (intersection == null && Vectors.pointInPolygon(
                    circleCenter,
                    polyVertices,
                    polyVertices.size / 3
                )
            ) {
                // circle inside polygon
                circle
            } else intersection
        }

        private fun circle(center: Vector3f, radius: Float): FloatArray {
            if (unitCircle == null) {
                unitCircle = FloatArray(CIRCLE_SEGMENTS * 3)
                for (i in 0 until CIRCLE_SEGMENTS) {
                    val a = i * PI * 2 / CIRCLE_SEGMENTS
                    unitCircle!![3 * i] = cos(a).toFloat()
                    unitCircle!![3 * i + 1] = 0f
                    unitCircle!![3 * i + 2] = -sin(a).toFloat()
                }
            }
            val circle = FloatArray(12 * 3)
            var i = 0
            while (i < CIRCLE_SEGMENTS * 3) {
                circle[i] = unitCircle!![i] * radius + center.x
                circle[i + 1] = center.y
                circle[i + 2] = unitCircle!![i + 2] * radius + center.z
                i += 3
            }
            return circle
        }

        companion object {
            private const val CIRCLE_SEGMENTS = 12
            private var unitCircle: FloatArray? = null
        }
    }

    companion object {
        fun noop() = NoOpPolygonByCircleConstraint()
        fun strict() = StrictPolygonByCircleConstraint()
    }
}