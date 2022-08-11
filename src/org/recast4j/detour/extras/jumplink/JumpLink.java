package org.recast4j.detour.extras.jumplink;

public class JumpLink {
    public static final int MAX_SPINE = 8;
    public final int numSpines = MAX_SPINE;
    public final float[] spine0 = new float[MAX_SPINE * 3], spine1 = new float[MAX_SPINE * 3];
    public GroundSample[] startSamples, endSamples;
    public GroundSegment start, end;
    public Trajectory trajectory;

}
