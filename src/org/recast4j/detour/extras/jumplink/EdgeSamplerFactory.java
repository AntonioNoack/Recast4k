package org.recast4j.detour.extras.jumplink;

import org.joml.Vector3f;

import static org.recast4j.Vectors.add;

class EdgeSamplerFactory {

    EdgeSampler get(JumpLinkBuilderConfig cfg, JumpLinkType type, Edge edge) {
        switch (type) {
            case EDGE_JUMP:
                return initEdgeJumpSampler(cfg, edge);
            case EDGE_CLIMB_DOWN:
                return initClimbDownSampler(cfg, edge);
            case EDGE_JUMP_OVER:
            default:
                throw new IllegalArgumentException("Unsupported jump type " + type);
        }
    }

    private EdgeSampler initEdgeJumpSampler(JumpLinkBuilderConfig cfg, Edge edge) {

        EdgeSampler es = new EdgeSampler(edge, new JumpTrajectory(cfg.jumpHeight));
        es.start.height = cfg.agentClimb * 2;
        Vector3f offset = new Vector3f();
        trans2d(offset, es.az, es.ay, cfg.startDistance, -cfg.agentClimb);
        add(es.start.p, edge.a, offset);
        add(es.start.q, edge.b, offset);

        float dx = cfg.endDistance - 2 * cfg.agentRadius;
        float cs = cfg.cellSize;
        int numSamples = Math.max(2, (int) Math.ceil(dx / cs));

        for (int j = 0; j < numSamples; ++j) {
            float v = (float) j / (float) (numSamples - 1);
            float ox = 2 * cfg.agentRadius + dx * v;
            trans2d(offset, es.az, es.ay, ox, cfg.minHeight);
            GroundSegment end = new GroundSegment();
            end.height = cfg.heightRange;
            add(end.p, edge.a, offset);
            add(end.q, edge.b, offset);
            es.end.add(end);
        }
        return es;
    }

    private EdgeSampler initClimbDownSampler(JumpLinkBuilderConfig cfg, Edge edge) {
        EdgeSampler es = new EdgeSampler(edge, new ClimbTrajectory());
        es.start.height = cfg.agentClimb * 2;
        Vector3f offset = new Vector3f();
        trans2d(offset, es.az, es.ay, cfg.startDistance, -cfg.agentClimb);
        add(es.start.p, edge.a, offset);
        add(es.start.q, edge.b, offset);

        trans2d(offset, es.az, es.ay, cfg.endDistance, cfg.minHeight);
        GroundSegment end = new GroundSegment();
        end.height = cfg.heightRange;
        add(end.p, edge.a, offset);
        add(end.q, edge.b, offset);
        es.end.add(end);
        return es;
    }

    private void trans2d(Vector3f dst, Vector3f ax, Vector3f ay, float pt0, float pt1) {
        dst.x = ax.x * pt0 + ay.x * pt1;
        dst.y = ax.y * pt0 + ay.y * pt1;
        dst.z = ax.z * pt0 + ay.z * pt1;
    }

}
