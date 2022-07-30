package org.recast4j.detour.extras.jumplink;

import org.joml.Vector3f;

public interface Trajectory {

    default float lerp(float f, float g, float u) {
        return u * g + (1f - u) * f;
    }

    Vector3f apply(Vector3f start, Vector3f end, float u);

}
