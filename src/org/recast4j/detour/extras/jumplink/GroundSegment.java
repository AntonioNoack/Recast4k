package org.recast4j.detour.extras.jumplink;

import org.joml.Vector3f;

public class GroundSegment {
    public final Vector3f p = new Vector3f();
    public final Vector3f q = new Vector3f();
    public GroundSample[] samples;
    public float height;

}
