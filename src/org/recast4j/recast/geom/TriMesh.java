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
package org.recast4j.recast.geom;

import java.util.List;

import org.recast4j.Callback;
import org.recast4j.recast.geom.ChunkyTriMesh.Node1;

public class TriMesh {

    public final float[] vertices;
    public final int[] triangles;
    private final ChunkyTriMesh chunkyTriMesh;

    public TriMesh(float[] vertices, int[] triangles) {
        this.vertices = vertices;
        this.triangles = triangles;
        chunkyTriMesh = new ChunkyTriMesh(vertices, triangles, triangles.length / 3, 32);
    }

    @SuppressWarnings("unused")
    public List<Node1> getChunksOverlappingRect(float[] bmin, float[] bmax) {
        return chunkyTriMesh.getChunksOverlappingRect(bmin, bmax);
    }

    public void foreachChunkOverlappingRect(float[] bmin, float[] bmax, Callback<Node1> callback) {
        chunkyTriMesh.foreachChunkOverlappingRect(bmin, bmax, callback);
    }

}
