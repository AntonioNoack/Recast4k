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
package org.recast4j.dynamic.collider

import org.recast4j.recast.Heightfield
import org.recast4j.recast.RecastRasterization.rasterizeTriangle
import org.recast4j.recast.Telemetry
import kotlin.math.floor
import kotlin.math.max
import kotlin.math.min

class TrimeshCollider : AbstractCollider {
    private val vertices: FloatArray
    private val triangles: IntArray

    constructor(vertices: FloatArray, triangles: IntArray, area: Int, flagMergeThreshold: Float) : super(
        area,
        flagMergeThreshold,
        computeBounds(vertices)
    ) {
        this.vertices = vertices
        this.triangles = triangles
    }

    constructor(
        vertices: FloatArray,
        triangles: IntArray,
        bounds: FloatArray?,
        area: Int,
        flagMergeThreshold: Float
    ) : super(area, flagMergeThreshold, bounds!!) {
        this.vertices = vertices
        this.triangles = triangles
    }

    override fun rasterize(hf: Heightfield, telemetry: Telemetry?) {
        var i = 0
        while (i < triangles.size) {
            rasterizeTriangle(
                hf,
                vertices,
                triangles[i],
                triangles[i + 1],
                triangles[i + 2],
                area,
                floor((flagMergeThreshold / hf.cellHeight)).toInt(),
                telemetry
            )
            i += 3
        }
    }

    companion object {
        fun computeBounds(vertices: FloatArray): FloatArray {
            val bounds = floatArrayOf(vertices[0], vertices[1], vertices[2], vertices[0], vertices[1], vertices[2])
            var i = 3
            while (i < vertices.size) {
                bounds[0] = min(bounds[0], vertices[i])
                bounds[1] = min(bounds[1], vertices[i + 1])
                bounds[2] = min(bounds[2], vertices[i + 2])
                bounds[3] = max(bounds[3], vertices[i])
                bounds[4] = max(bounds[4], vertices[i + 1])
                bounds[5] = max(bounds[5], vertices[i + 2])
                i += 3
            }
            return bounds
        }
    }
}