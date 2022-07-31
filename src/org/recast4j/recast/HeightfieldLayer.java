package org.recast4j.recast;

import org.joml.Vector3f;

public class HeightfieldLayer {
    public final Vector3f bmin = new Vector3f();
    public final Vector3f bmax = new Vector3f();
    public float cellSize;
    public float cellHeight;
    public int width; /// < The width of the heightfield. (Along the x-axis in cell units.)
    public int height; /// < The height of the heightfield. (Along the z-axis in cell units.)
    public int minX; /// < The minimum x-bounds of usable data.
    public int maxX; /// < The maximum x-bounds of usable data.
    public int minY; /// < The minimum y-bounds of usable data. (Along the z-axis.)
    public int maxY; /// < The maximum y-bounds of usable data. (Along the z-axis.)
    public int minH; /// < The minimum height bounds of usable data. (Along the y-axis.)
    public int maxH; /// < The maximum height bounds of usable data. (Along the y-axis.)
    public int[] heights; /// < The heightfield. [Size: width * height]
    public int[] areas; /// < Area ids. [Size: Same as #heights]
    public int[] cons; /// < Packed neighbor connection information. [Size: Same as #heights]
}