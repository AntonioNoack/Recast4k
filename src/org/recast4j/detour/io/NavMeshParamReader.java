package org.recast4j.detour.io;

import org.recast4j.detour.NavMeshParams;

import java.nio.ByteBuffer;

public class NavMeshParamReader {

    public NavMeshParams read(ByteBuffer bb) {
        NavMeshParams params = new NavMeshParams();
        params.orig.set(bb.getFloat(), bb.getFloat(), bb.getFloat());
        params.tileWidth = bb.getFloat();
        params.tileHeight = bb.getFloat();
        params.maxTiles = bb.getInt();
        params.maxPolys = bb.getInt();
        return params;
    }

}
