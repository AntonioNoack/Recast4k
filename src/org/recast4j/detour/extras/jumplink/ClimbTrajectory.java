package org.recast4j.detour.extras.jumplink;

import org.jetbrains.annotations.NotNull;
import org.joml.Vector3f;

public class ClimbTrajectory implements Trajectory {

    @NotNull
    @Override
    public Vector3f apply(Vector3f start, Vector3f end, float u) {
        return new Vector3f(
                lerp(start.x, end.x, Math.min(2f * u, 1f)),
                lerp(start.y, end.y, Math.max(0f, 2f * u - 1f)),
                lerp(start.z, end.z, Math.min(2f * u, 1f))
        );
    }

    @Override
    public float lerp(float f, float g, float u) {
        return Trajectory.super.lerp(f, g, u);
    }
}
