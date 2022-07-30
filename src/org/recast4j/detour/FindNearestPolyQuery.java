package org.recast4j.detour;

import org.joml.Vector3f;

import static org.recast4j.detour.DetourCommon.vLenSqr;
import static org.recast4j.detour.DetourCommon.vSub;

public class FindNearestPolyQuery implements PolyQuery {

    private final NavMeshQuery query;
    private final Vector3f center;
    private long nearestRef;
    private Vector3f nearestPt;
    private boolean overPoly;
    private float nearestDistanceSqr;

    public FindNearestPolyQuery(NavMeshQuery query, Vector3f center) {
        this.query = query;
        this.center = center;
        nearestDistanceSqr = Float.MAX_VALUE;
        nearestPt = new Vector3f(center);
    }

    @Override
    public void process(MeshTile tile, Poly poly, long ref) {
        // Find nearest polygon amongst the nearby polygons.
        Result<ClosestPointOnPolyResult> closest = query.closestPointOnPoly(ref, center);
        boolean posOverPoly = closest.result.isPosOverPoly();
        Vector3f closestPtPoly = closest.result.getClosest();

        // If a point is directly over a polygon and closer than
        // climb height, favor that instead of straight line nearest point.
        float d;
        Vector3f diff = vSub(center, closestPtPoly);
        if (posOverPoly) {
            d = Math.abs(diff.y) - tile.data.header.walkableClimb;
            d = d > 0 ? d * d : 0;
        } else {
            d = diff.lengthSquared();
        }

        if (d < nearestDistanceSqr) {
            nearestPt = closestPtPoly;
            nearestDistanceSqr = d;
            nearestRef = ref;
            overPoly = posOverPoly;
        }

    }

    public FindNearestPolyResult result() {
        return new FindNearestPolyResult(nearestRef, nearestPt, overPoly);
    }

}
