package org.recast4j.detour.extras.jumplink;

import org.joml.Vector3f;
import org.recast4j.detour.Tupple2;

import java.util.function.BiFunction;

import static org.recast4j.detour.DetourCommon.vLerp;

abstract class AbstractGroundSampler implements GroundSampler {

    protected void sampleGround(JumpLinkBuilderConfig acfg, EdgeSampler es,
                                BiFunction<Vector3f, Float, Tupple2<Boolean, Float>> heightFunc) {
        float cs = acfg.cellSize;
        float dist = es.start.p.distance(es.start.q);
        int ngsamples = Math.max(2, (int) Math.ceil(dist / cs));
        sampleGroundSegment(heightFunc, es.start, ngsamples);
        for (GroundSegment end : es.end) {
            sampleGroundSegment(heightFunc, end, ngsamples);
        }
    }

    protected void sampleGroundSegment(BiFunction<Vector3f, Float, Tupple2<Boolean, Float>> heightFunc, GroundSegment seg,
                                       int nsamples) {
        seg.gsamples = new GroundSample[nsamples];

        for (int i = 0; i < nsamples; ++i) {
            float u = i / (float) (nsamples - 1);

            GroundSample s = new GroundSample();
            seg.gsamples[i] = s;
            Vector3f pt = vLerp(seg.p, seg.q, u);
            Tupple2<Boolean, Float> height = heightFunc.apply(pt, seg.height);
            s.p.set(pt);
            s.p.y = height.second;

            if (!height.first) {
                continue;
            }
            s.validHeight = true;
        }
    }

}
