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
package org.recast4j.recast;

import org.joml.Vector3f;
import org.recast4j.Vectors;

import static org.recast4j.recast.RecastConstants.RC_NULL_AREA;

public class Recast {

    public static int calcGridSizeX(Vector3f bmin, Vector3f bmax, float cellSize) {
        return (int) ((bmax.x - bmin.x) / cellSize + 0.5f);
    }

    public static int calcGridSizeY(Vector3f bmin, Vector3f bmax, float cellSize) {
        return (int) ((bmax.z - bmin.z) / cellSize + 0.5f);
    }

    public static int calcTileCountX(Vector3f bmin, Vector3f bmax, float cellSize, int tileSizeX) {
        int gwd = Recast.calcGridSizeX(bmin, bmax, cellSize);
        return (gwd + tileSizeX - 1) / tileSizeX;
    }

    public static int calcTileCountY(Vector3f bmin, Vector3f bmax, float cellSize, int tileSizeZ) {
        int gwd = Recast.calcGridSizeY(bmin, bmax, cellSize);
        return (gwd + tileSizeZ - 1) / tileSizeZ;
    }

    /**
     * Modifies the area id of all triangles with a slope below the specified value.
     * See the rcConfig documentation for more information on the configuration parameters.
     */
    public static int[] markWalkableTriangles(float walkableSlopeAngle, float[] vertices, int[] tris, int nt,
                                              AreaModification areaMod) {
        int[] areas = new int[nt];
        float walkableThr = (float) Math.cos(walkableSlopeAngle / 180.0f * Math.PI);
        Vector3f norm = new Vector3f();
        for (int i = 0; i < nt; ++i) {
            int tri = i * 3;
            calcTriNormal(vertices, tris[tri], tris[tri + 1], tris[tri + 2], norm);
            // Check if the face is walkable.
            if (norm.y > walkableThr)
                areas[i] = areaMod.apply(areas[i]);
        }
        return areas;
    }

    static void calcTriNormal(float[] vertices, int v0, int v1, int v2, Vector3f norm) {
        Vector3f e0 = new Vector3f(), e1 = new Vector3f();
        Vectors.sub(e0, vertices, v1 * 3, v0 * 3);
        Vectors.sub(e1, vertices, v2 * 3, v0 * 3);
        e0.cross(e1, norm).normalize();
    }

    /**
     * Only sets the area id's for the unwalkable triangles. Does not alter the area id's for walkable triangles.
     * See the rcConfig documentation for more information on the configuration parameters.
     */
    @SuppressWarnings("unused")
    public static void clearUnwalkableTriangles(float walkableSlopeAngle, float[] vertices,
                                                int[] tris, int nt, int[] areas) {
        float walkableThr = (float) Math.cos(walkableSlopeAngle / 180.0f * Math.PI);
        Vector3f norm = new Vector3f();
        for (int i = 0; i < nt; ++i) {
            int tri = i * 3;
            calcTriNormal(vertices, tris[tri], tris[tri + 1], tris[tri + 2], norm);
            // Check if the face is walkable.
            if (norm.y <= walkableThr)
                areas[i] = RC_NULL_AREA;
        }
    }

}
