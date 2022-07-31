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
package org.recast4j.detour;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class NodePool {

    public final Map<Long, List<Node>> nodeMap = new HashMap<>();
    public final ArrayList<Node> nodeList = new ArrayList<>();

    public void clear() {
        nodeList.clear();
        nodeMap.clear();
    }

    List<Node> findNodes(long id) {
        List<Node> nodes = nodeMap.get(id);
        if (nodes == null) {
            nodes = new ArrayList<>();
        }
        return nodes;
    }

    Node findNode(long id) {
        List<Node> nodes = nodeMap.get(id);
        if (nodes != null && !nodes.isEmpty()) {
            return nodes.get(0);
        }
        return null;
    }

    Node getNode(long id, int state) {
        List<Node> nodes = nodeMap.get(id);
        if (nodes != null) {
            for (Node node : nodes) {
                if (node.state == state) {
                    return node;
                }
            }
        }
        return create(id, state);
    }

    protected Node create(long id, int state) {
        Node node = new Node(nodeList.size() + 1);
        node.id = id;
        node.state = state;
        nodeList.add(node);
        List<Node> nodes = nodeMap.computeIfAbsent(id, k -> new ArrayList<>());
        nodes.add(node);
        return node;
    }

    public int getNodeIdx(Node node) {
        return node != null ? node.index : 0;
    }

    public Node getNodeAtIdx(int idx) {
        return idx != 0 ? nodeList.get(idx - 1) : null;
    }

    public Node getNode(long ref) {
        return getNode(ref, 0);
    }

}
