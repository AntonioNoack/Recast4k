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

import org.recast4j.Callback;

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
        public int[] triangles;
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
    int numTriangles;
    int maxTrisPerChunk;

    private void calcExtends(Node0[] items, int startIndex, int endIndex, Node1 dst) {

        Node0 n = items[startIndex];
        dst.minX = n.minX;
        dst.minY = n.minY;
        dst.maxX = n.maxX;
        dst.maxY = n.maxY;

        for (int i = startIndex + 1; i < endIndex; ++i) {
            n = items[i];
            dst.minX = Math.min(dst.minX, n.minX);
            dst.minY = Math.min(dst.minY, n.minY);
            dst.maxX = Math.min(dst.maxX, n.maxX);
            dst.maxY = Math.min(dst.maxY, n.maxY);
        }
    }

    private int longestAxis(float x, float y) {
        return y > x ? 1 : 0;
    }

    private void subdivide(Node0[] items, int startIndex, int endIndex, int trisPerChunk, List<Node1> nodes, int[] inTris) {
        int length = endIndex - startIndex;

        Node1 node = new Node1();
        nodes.add(node);

        if (length <= trisPerChunk) {

            // Leaf
            calcExtends(items, startIndex, endIndex, node);

            // Copy triangles.
            node.i = nodes.size();
            node.triangles = new int[length * 3];

            int dst = 0;
            for (int i = startIndex; i < endIndex; ++i) {
                int src = items[i].i * 3;
                node.triangles[dst++] = inTris[src];
                node.triangles[dst++] = inTris[src + 1];
                node.triangles[dst++] = inTris[src + 2];
            }
        } else {

            // Split
            calcExtends(items, startIndex, endIndex, node);
            int axis = longestAxis(node.maxX - node.minX, node.maxY - node.minY);

            if (axis == 0) {
                Arrays.sort(items, startIndex, endIndex, new CompareItemX());
                // Sort along x-axis
            } else if (axis == 1) {
                Arrays.sort(items, startIndex, endIndex, new CompareItemY());
                // Sort along y-axis
            }

            int splitIndex = startIndex + length / 2;

            // Left
            subdivide(items, startIndex, splitIndex, trisPerChunk, nodes, inTris);
            // Right
            subdivide(items, splitIndex, endIndex, trisPerChunk, nodes, inTris);

            // Negative index means escape.
            node.i = -nodes.size();
        }
    }

    public ChunkyTriMesh(float[] vertices, int[] tris, int numTris, int trisPerChunk) {
        int numChunks = (numTris + trisPerChunk - 1) / trisPerChunk;

        nodes = new ArrayList<>(numChunks);
        this.numTriangles = numTris;

        // Build tree
        Node0[] items = new Node0[numTris];

        for (int i = 0; i < numTris; i++) {
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

        subdivide(items, 0, numTris, trisPerChunk, nodes, tris);

        // Calc max tris per node.
        maxTrisPerChunk = 0;
        for (Node1 node : nodes) {
            boolean isLeaf = node.i >= 0;
            if (!isLeaf) {
                continue;
            }
            if (node.triangles.length / 3 > maxTrisPerChunk) {
                maxTrisPerChunk = node.triangles.length / 3;
            }
        }

    }

    private boolean checkOverlapRect(float[] aMin, float[] aMax, Node1 b) {
        return !(aMin[0] > b.maxX) && !(aMax[0] < b.minX) && !(aMin[1] > b.maxY) && !(aMax[1] < b.minY);
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

    public void foreachChunkOverlappingRect(float[] bmin, float[] bmax, Callback<Node1> callback) {
        // Traverse tree
        int i = 0;
        while (i < nodes.size()) {
            Node1 node = nodes.get(i);
            boolean overlap = checkOverlapRect(bmin, bmax, node);
            boolean isLeafNode = node.i >= 0;
            boolean x = isLeafNode && overlap;
            if (x) callback.call(node);
            i = x ? i + 1 : -node.i;
        }
    }

}
