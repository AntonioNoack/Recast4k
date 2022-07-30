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

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

public class ChunkyTriMesh {

    private static class Node0 {
        protected float minX, minY, maxX, maxY;
        protected int i;
    }

    public static class Node1 extends Node0 {
        public int[] tris;
    }

    private static class CompareItemX implements Comparator<Node0> {
        @Override
        public int compare(Node0 a, Node0 b) {
            return Float.compare(a.minX, b.minX);
        }
    }

    private static class CompareItemY implements Comparator<Node0> {
        @Override
        public int compare(Node0 a, Node0 b) {
            return Float.compare(a.minY, b.minY);
        }
    }

    List<Node1> nodes;
    int ntris;
    int maxTrisPerChunk;

    private void calcExtends(Node0[] items, int imin, int imax, Node1 dst) {

        dst.minX = items[imin].minX;
        dst.minY = items[imin].minY;
        dst.maxX = items[imin].maxX;
        dst.maxY = items[imin].maxY;

        for (int i = imin + 1; i < imax; ++i) {
            Node0 it = items[i];
            dst.minX = Math.min(dst.minX, it.minX);
            dst.minY = Math.min(dst.minY, it.minY);
            dst.maxX = Math.min(dst.maxX, it.maxX);
            dst.maxY = Math.min(dst.maxY, it.maxY);
        }
    }

    private int longestAxis(float x, float y) {
        return y > x ? 1 : 0;
    }

    private void subdivide(Node0[] items, int imin, int imax, int trisPerChunk, List<Node1> nodes, int[] inTris) {
        int inum = imax - imin;

        Node1 node = new Node1();
        nodes.add(node);

        if (inum <= trisPerChunk) {

            // Leaf
            calcExtends(items, imin, imax, node);

            // Copy triangles.
            node.i = nodes.size();
            node.tris = new int[inum * 3];

            int dst = 0;
            for (int i = imin; i < imax; ++i) {
                int src = items[i].i * 3;
                node.tris[dst++] = inTris[src];
                node.tris[dst++] = inTris[src + 1];
                node.tris[dst++] = inTris[src + 2];
            }
        } else {

            // Split
            calcExtends(items, imin, imax, node);
            int axis = longestAxis(node.maxX - node.minX, node.maxY - node.minY);

            if (axis == 0) {
                Arrays.sort(items, imin, imax, new CompareItemX());
                // Sort along x-axis
            } else if (axis == 1) {
                Arrays.sort(items, imin, imax, new CompareItemY());
                // Sort along y-axis
            }

            int isplit = imin + inum / 2;

            // Left
            subdivide(items, imin, isplit, trisPerChunk, nodes, inTris);
            // Right
            subdivide(items, isplit, imax, trisPerChunk, nodes, inTris);

            // Negative index means escape.
            node.i = -nodes.size();
        }
    }

    public ChunkyTriMesh(float[] vertices, int[] tris, int ntris, int trisPerChunk) {
        int nchunks = (ntris + trisPerChunk - 1) / trisPerChunk;

        nodes = new ArrayList<>(nchunks);
        this.ntris = ntris;

        // Build tree
        Node0[] items = new Node0[ntris];

        for (int i = 0; i < ntris; i++) {
            int t = i * 3;
            Node0 it = items[i] = new Node0();
            it.i = i;
            // Calc triangle XZ bounds.
            it.minX = it.maxX = vertices[tris[t] * 3];
            it.minY = it.maxY = vertices[tris[t] * 3 + 2];
            for (int j = 1; j < 3; ++j) {
                int v = tris[t + j] * 3;
                if (vertices[v] < it.minX) {
                    it.minX = vertices[v];
                }
                if (vertices[v + 2] < it.minY) {
                    it.minY = vertices[v + 2];
                }

                if (vertices[v] > it.maxX) {
                    it.maxX = vertices[v];
                }
                if (vertices[v + 2] > it.maxY) {
                    it.maxY = vertices[v + 2];
                }
            }
        }

        subdivide(items, 0, ntris, trisPerChunk, nodes, tris);

        // Calc max tris per node.
        maxTrisPerChunk = 0;
        for (Node1 node : nodes) {
            boolean isLeaf = node.i >= 0;
            if (!isLeaf) {
                continue;
            }
            if (node.tris.length / 3 > maxTrisPerChunk) {
                maxTrisPerChunk = node.tris.length / 3;
            }
        }

    }

    private boolean checkOverlapRect(float[] amin, float[] amax, Node1 b) {
        return !(amin[0] > b.maxX) && !(amax[0] < b.minX) && !(amin[1] > b.maxY) && !(amax[1] < b.minY);
    }

    public List<Node1> getChunksOverlappingRect(float[] bmin, float[] bmax) {
        // Traverse tree
        List<Node1> ids = new ArrayList<>();
        int i = 0;
        while (i < nodes.size()) {
            Node1 node = nodes.get(i);
            boolean overlap = checkOverlapRect(bmin, bmax, node);
            boolean isLeafNode = node.i >= 0;

            if (isLeafNode && overlap) {
                ids.add(node);
            }

            if (overlap || isLeafNode) {
                i++;
            } else {
                i = -node.i;
            }
        }
        return ids;
    }

}
