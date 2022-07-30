package org.recast4j.detour.extras.jumplink;

import org.joml.Vector3f;

import java.util.ArrayList;
import java.util.List;

public class EdgeSampler {

    public final GroundSegment start = new GroundSegment();
    public final List<GroundSegment> end = new ArrayList<>();
    public final Trajectory trajectory;

    final Vector3f ax = new Vector3f();
    final Vector3f ay = new Vector3f();
    final Vector3f az = new Vector3f();

    public EdgeSampler(Edge edge, Trajectory trajectory) {
        this.trajectory = trajectory;
        ax.set(edge.sq).sub(edge.sp).normalize();
        az.set(ax.z, 0, -ax.x).normalize();
        ay.set(0f, 1f, 0f);
    }

}
