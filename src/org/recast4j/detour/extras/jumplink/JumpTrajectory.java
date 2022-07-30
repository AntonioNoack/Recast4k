package org.recast4j.detour.extras.jumplink;

import org.joml.Vector3f;

public class JumpTrajectory implements Trajectory {

    private final float jumpHeight;

    public JumpTrajectory(float jumpHeight) {
        this.jumpHeight = jumpHeight;
    }

    @Override
    public Vector3f apply(Vector3f start, Vector3f end, float u) {
        return new Vector3f(
                lerp(start.x, end.x, u),
                interpolateHeight(start.y, end.y, u),
                lerp(start.z, end.z, u)
        );
    }

    private float interpolateHeight(float ys, float ye, float u) {
        if (u == 0f) {
            return ys;
        } else if (u == 1.0f) {
            return ye;
        }
        float h1, h2;
        if (ys >= ye) { // jump down
            h1 = jumpHeight;
            h2 = jumpHeight + ys - ye;
        } else { // jump up
            h1 = jumpHeight + ys - ye;
            h2 = jumpHeight;
        }
        float t = (float) (Math.sqrt(h1) / (Math.sqrt(h2) + Math.sqrt(h1)));
        if (u <= t) {
            float v = 1.0f - (u / t);
            return ys + h1 - h1 * v * v;
        }
        float v = (u - t) / (1.0f - t);
        return ys + h1 - h2 * v * v;
    }

}
