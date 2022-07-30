package org.recast4j.detour.extras.jumplink;

import org.joml.Vector3f;
import org.recast4j.recast.PolyMesh;

import java.util.ArrayList;
import java.util.List;

import static org.recast4j.detour.DetourCommon.copy;
import static org.recast4j.recast.RecastConstants.RC_MESH_NULL_IDX;

class EdgeExtractor {

    Edge[] extractEdges(PolyMesh mesh) {
        List<Edge> edges = new ArrayList<>();
        if (mesh != null) {
            Vector3f orig = mesh.bmin;
            float cs = mesh.cs;
            float ch = mesh.ch;
            for (int i = 0; i < mesh.npolys; i++) {
                int nvp = mesh.nvp;
                int p = i * 2 * nvp;
                for (int j = 0; j < nvp; ++j) {
                    if (mesh.polys[p + j] == RC_MESH_NULL_IDX) {
                        break;
                    }
                    // Skip connected edges.
                    if ((mesh.polys[p + nvp + j] & 0x8000) != 0) {
                        int dir = mesh.polys[p + nvp + j] & 0xf;
                        if (dir == 0xf) {// Border
                            if (mesh.polys[p + nvp + j] != RC_MESH_NULL_IDX) {
                                continue;
                            }
                            int nj = j + 1;
                            if (nj >= nvp || mesh.polys[p + nj] == RC_MESH_NULL_IDX) {
                                nj = 0;
                            }
                            Edge e = new Edge();
                            copy(e.sp, mesh.vertices, mesh.polys[p + nj] * 3);
                            e.sp.mul(cs, ch, cs).add(orig);
                            copy(e.sq, mesh.vertices, mesh.polys[p + j] * 3);
                            e.sq.mul(cs, ch, cs).add(orig);
                            edges.add(e);
                        }
                    }
                }
            }
        }
        return edges.toArray(new Edge[0]);

    }

}
