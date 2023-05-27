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
package org.recast4j.detour.crowd.debug

import org.joml.Vector3f
import org.recast4j.Vectors.clamp
import kotlin.math.max
import kotlin.math.min

class ObstacleAvoidanceDebugData(var maxSamples: Int) {

    var sampleCount = 0

    var vel: FloatArray = FloatArray(3 * maxSamples)
    var ssize: FloatArray = FloatArray(maxSamples)
    var pen: FloatArray = FloatArray(maxSamples)
    var vpen: FloatArray = FloatArray(maxSamples)
    var vcpen: FloatArray = FloatArray(maxSamples)
    var spen: FloatArray = FloatArray(maxSamples)
    var tpen: FloatArray = FloatArray(maxSamples)

    fun reset() {
        sampleCount = 0
    }

    fun normalizeArray(arr: FloatArray, n: Int) {
        // Normalize penalty range.
        var minPen = Float.MAX_VALUE
        var maxPen = -Float.MAX_VALUE
        for (i in 0 until n) {
            minPen = min(minPen, arr[i])
            maxPen = max(maxPen, arr[i])
        }
        val penRange = maxPen - minPen
        val s = if (penRange > 0.001f) 1f / penRange else 1f
        for (i in 0 until n) arr[i] = clamp((arr[i] - minPen) * s, 0f, 1f)
    }

    fun normalizeSamples() {
        normalizeArray(pen, sampleCount)
        normalizeArray(vpen, sampleCount)
        normalizeArray(vcpen, sampleCount)
        normalizeArray(spen, sampleCount)
        normalizeArray(tpen, sampleCount)
    }

    fun addSample(vel: Vector3f, ssize: Float, pen: Float, vpen: Float, vcpen: Float, spen: Float, tpen: Float) {
        if (sampleCount >= maxSamples) return
        vel.get(this.vel, sampleCount * 3)
        this.ssize[sampleCount] = ssize
        this.pen[sampleCount] = pen
        this.vpen[sampleCount] = vpen
        this.vcpen[sampleCount] = vcpen
        this.spen[sampleCount] = spen
        this.tpen[sampleCount] = tpen
        sampleCount++
    }

    fun getSampleVelocity(i: Int): Vector3f {
        val vel = Vector3f()
        vel.set(this.vel, i * 3)
        return vel
    }

    fun getSampleSize(i: Int): Float {
        return ssize[i]
    }

    fun getSamplePenalty(i: Int): Float {
        return pen[i]
    }

    fun getSampleDesiredVelocityPenalty(i: Int): Float {
        return vpen[i]
    }

    fun getSampleCurrentVelocityPenalty(i: Int): Float {
        return vcpen[i]
    }

    fun getSamplePreferredSidePenalty(i: Int): Float {
        return spen[i]
    }

    fun getSampleCollisionTimePenalty(i: Int): Float {
        return tpen[i]
    }
}