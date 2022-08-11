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
package org.recast4j.detour.crowd.debug;

import org.joml.Vector3f;

import static org.joml.Math.clamp;
import static org.recast4j.Vectors.copy;

public class ObstacleAvoidanceDebugData {
    int numSamples, maxSamples;
    float[] vel, ssize;
    float[] pen, vpen, vcpen, spen, tpen;

    public ObstacleAvoidanceDebugData(int maxNumSamples) {
        maxSamples = maxNumSamples;
        vel = new float[3 * maxSamples];
        pen = new float[maxSamples];
        ssize = new float[maxSamples];
        vpen = new float[maxSamples];
        vcpen = new float[maxSamples];
        spen = new float[maxSamples];
        tpen = new float[maxSamples];
    }

    public void reset() {
        numSamples = 0;
    }

    void normalizeArray(float[] arr, int n) {
        // Normalize penalty range.
        float minPen = Float.MAX_VALUE;
        float maxPen = -Float.MAX_VALUE;
        for (int i = 0; i < n; ++i) {
            minPen = Math.min(minPen, arr[i]);
            maxPen = Math.max(maxPen, arr[i]);
        }
        float penRange = maxPen - minPen;
        float s = penRange > 0.001f ? (1f / penRange) : 1;
        for (int i = 0; i < n; ++i)
            arr[i] = clamp((arr[i] - minPen) * s, 0f, 1f);
    }

    public void normalizeSamples() {
        normalizeArray(pen, numSamples);
        normalizeArray(vpen, numSamples);
        normalizeArray(vcpen, numSamples);
        normalizeArray(spen, numSamples);
        normalizeArray(tpen, numSamples);
    }

    public void addSample(Vector3f vel, float ssize, float pen, float vpen, float vcpen, float spen, float tpen) {
        if (numSamples >= maxSamples) return;
        copy(this.vel, numSamples * 3, vel);
        this.ssize[numSamples] = ssize;
        this.pen[numSamples] = pen;
        this.vpen[numSamples] = vpen;
        this.vcpen[numSamples] = vcpen;
        this.spen[numSamples] = spen;
        this.tpen[numSamples] = tpen;
        numSamples++;
    }

    public int getSampleCount() {
        return numSamples;
    }

    public Vector3f getSampleVelocity(int i) {
        Vector3f vel = new Vector3f();
        copy(vel, this.vel, i * 3);
        return vel;
    }

    public float getSampleSize(int i) {
        return ssize[i];
    }

    public float getSamplePenalty(int i) {
        return pen[i];
    }

    public float getSampleDesiredVelocityPenalty(int i) {
        return vpen[i];
    }

    public float getSampleCurrentVelocityPenalty(int i) {
        return vcpen[i];
    }

    public float getSamplePreferredSidePenalty(int i) {
        return spen[i];
    }

    public float getSampleCollisionTimePenalty(int i) {
        return tpen[i];
    }
}