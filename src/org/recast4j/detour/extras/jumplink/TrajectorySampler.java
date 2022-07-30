package org.recast4j.detour.extras.jumplink;

import static org.recast4j.detour.DetourCommon.overlapRange;
import static org.recast4j.detour.DetourCommon.vDist2D;

import org.joml.Vector3f;
import org.recast4j.recast.Heightfield;
import org.recast4j.recast.Span;

class TrajectorySampler {

    void sample(JumpLinkBuilderConfig acfg, Heightfield heightfield, EdgeSampler es) {
        int nsamples = es.start.gsamples.length;
        for (int i = 0; i < nsamples; ++i) {
            GroundSample ssmp = es.start.gsamples[i];
            for (GroundSegment end : es.end) {
                GroundSample esmp = end.gsamples[i];
                if (!ssmp.validHeight || !esmp.validHeight) {
                    continue;
                }

                if (!sampleTrajectory(acfg, heightfield, ssmp.p, esmp.p, es.trajectory)) {
                    continue;
                }
                ssmp.validTrajectory = true;
                esmp.validTrajectory = true;
            }
        }
    }

    private boolean sampleTrajectory(JumpLinkBuilderConfig acfg, Heightfield solid, Vector3f pa, Vector3f pb, Trajectory tra) {
        float cs = Math.min(acfg.cellSize, acfg.cellHeight);
        float d = vDist2D(pa, pb) + Math.abs(pa.y - pb.y);
        int nsamples = Math.max(2, (int) Math.ceil(d / cs));
        for (int i = 0; i < nsamples; ++i) {
            float u = (float) i / (float) (nsamples - 1);
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
        float cs = solid.cs;
        float ch = solid.ch;
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
            float symin = orig.y + s.smin * ch;
            float symax = orig.y + s.smax * ch;
            if (overlapRange(ymin, ymax, symin, symax)) {
                return true;
            }
            s = s.next;
        }

        return false;
    }

}
