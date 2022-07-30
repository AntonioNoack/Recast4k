package org.recast4j.detour.extras.jumplink;

import static org.recast4j.detour.DetourCommon.vCopy;
import static org.recast4j.detour.DetourCommon.normalize;
import static org.recast4j.detour.DetourCommon.vSet;
import static org.recast4j.detour.DetourCommon.vSub;

import java.util.ArrayList;
import java.util.List;

public class EdgeSampler {

    public final GroundSegment start = new GroundSegment();
    public final List<GroundSegment> end = new ArrayList<>();
    public final Trajectory trajectory;

    final float[] ax = new Vector3f();
    final float[] ay = new Vector3f();
    final float[] az = new Vector3f();

    public EdgeSampler(Edge edge, Trajectory trajectory) {
        this.trajectory = trajectory;
        copy(ax, vSub(edge.sq, edge.sp));
        normalize(ax);
        vSet(az, ax[2], 0, -ax[0]);
        normalize(az);
        vSet(ay, 0, 1, 0);
    }

}
