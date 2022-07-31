package org.recast4j.recast;

import org.joml.Vector3f;

public class HeightfieldLayer {
    public final Vector3f bmin = new Vector3f();
    public final Vector3f bmax = new Vector3f();
    public float cellSize;
    public float cellHeight;
    /**
     * The size of the heightfield. (Along the x/z-axis in cell units.)
     */
    public int width, height;
    /**
     * The bounds of usable data.
     */
    public int minX, maxX, minH, maxH, minZ, maxZ;
    /**
     * The heightfield, size: w*h
     */
    public int[] heights;
    /**
     * Area ids. Size same as heights
     */
    public int[] areas;
    /**
     * Packed neighbor connection information. Size same as heights
     */
    public int[] cons;
}