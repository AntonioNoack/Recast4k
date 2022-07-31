package org.recast4j.detour.extras.jumplink;

import static org.recast4j.Vectors.overlapRange;
import static org.recast4j.Vectors.dist2D;

import org.joml.Vector3f;
import org.recast4j.recast.Heightfield;
import org.recast4j.recast.Span;

class TrajectorySampler {

    void sample(JumpLinkBuilderConfig jlc, Heightfield heightfield, EdgeSampler es) {
        int l = es.start.samples.length;
        for (int i = 0; i < l; ++i) {
            GroundSample s0 = es.start.samples[i];
            for (GroundSegment end : es.end) {
                GroundSample s1 = end.samples[i];
                if (!s0.validHeight || !s1.validHeight) {
                    continue;
                }
                if (!sampleTrajectory(jlc, heightfield, s0.p, s1.p, es.trajectory)) {
                    continue;
                }
                s0.validTrajectory = true;
                s1.validTrajectory = true;
            }
        }
    }

    private boolean sampleTrajectory(JumpLinkBuilderConfig acfg, Heightfield solid, Vector3f pa, Vector3f pb, Trajectory tra) {
        float cs = Math.min(acfg.cellSize, acfg.cellHeight);
        float d = dist2D(pa, pb) + Math.abs(pa.y - pb.y);
        int l = Math.max(2, (int) Math.ceil(d / cs));
        for (int i = 0; i < l; ++i) {
            float u = (float) i / (float) (l - 1);
            Vector3f p = tra.apply(pa, pb, u);
            if (checkHeightfieldCollision(solid, p.x, p.y + acfg.groundTolerance, p.y + acfg.agentHeight, p.z)) {
                return false;
            }
        }
        return true;
    }

    private boolean checkHeightfieldCollision(Heightfield solid, float x, float ymin, float ymax, float z) {
        int w = solid.width;
        int h = solid.height;
        float cs = solid.cellSize;
        float ch = solid.cellHeight;
        Vector3f orig = solid.bmin;
        int ix = (int) Math.floor((x - orig.x) / cs);
        int iz = (int) Math.floor((z - orig.z) / cs);

        if (ix < 0 || iz < 0 || ix > w || iz > h) {
            return false;
        }

        Span s = solid.spans[ix + iz * w];
        if (s == null) {
            return false;
        }

        while (s != null) {
            float syMin = orig.y + s.min * ch;
            float symax = orig.y + s.max * ch;
            if (overlapRange(ymin, ymax, syMin, symax)) {
                return true;
            }
            s = s.next;
        }

        return false;
    }

}
