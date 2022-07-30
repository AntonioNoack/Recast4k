package org.recast4j.detour.extras.jumplink;

public interface Trajectory {

    default float lerp(float f, float g, float u) {
        return u * g + (1f - u) * f;
    }

    float[] apply(Vector3f start, Vector3f end, float u);

}
