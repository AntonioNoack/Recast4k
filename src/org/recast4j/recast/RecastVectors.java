/*
Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
recast4j Copyright (c) 2015-2019 Piotr Piastucki piotr@jtilia.org

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
package org.recast4j.recast;

import org.joml.Vector3f;

public class RecastVectors {

    public static void min(Vector3f a, float[] b, int i) {
        a.x = Math.min(a.x, b[i]);
        a.y = Math.min(a.y, b[i + 1]);
        a.z = Math.min(a.z, b[i + 2]);
    }

    public static void max(Vector3f a, float[] b, int i) {
        a.x = Math.max(a.x, b[i]);
        a.y = Math.max(a.y, b[i + 1]);
        a.z = Math.max(a.z, b[i + 2]);
    }

    public static void copy(float[] out, float[] in, int i) {
        copy(out, 0, in, i);
    }

    public static void copy(Vector3f out, float[] in, int i) {
        out.set(in[i], in[i + 1], in[i + 2]);
    }

    public static void copy(float[] out, float[] in) {
        copy(out, 0, in, 0);
    }

    public static void copy(float[] out, int n, float[] in, int m) {
        out[n] = in[m];
        out[n + 1] = in[m + 1];
        out[n + 2] = in[m + 2];
    }

    public static void add(float[] dst, float[] a, float[] verts, int i) {
        dst[0] = a[0] + verts[i];
        dst[1] = a[1] + verts[i + 1];
        dst[2] = a[2] + verts[i + 2];
    }

    public static void sub(Vector3f dst, float[] verts, int i, int j) {
        dst.x = verts[i] - verts[j];
        dst.y = verts[i + 1] - verts[j + 1];
        dst.z = verts[i + 2] - verts[j + 2];
    }

    public static void sub(float[] dst, float[] i, float[] verts, int j) {
        dst[0] = i[0] - verts[j];
        dst[1] = i[1] - verts[j + 1];
        dst[2] = i[2] - verts[j + 2];
    }

    public static void cross(Vector3f dest, Vector3f v1, Vector3f v2) {
        v1.cross(v2, dest);
    }

    public static void normalize(float[] v) {
        float d = (float) (1.0f / Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]));
        v[0] *= d;
        v[1] *= d;
        v[2] *= d;
    }

    public static float dot(float[] v1, float[] v2) {
        return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
    }

}