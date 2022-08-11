/*
Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
recast4j copyright (c) 2015-2019 Piotr Piastucki piotr@jtilia.org

This software is provided 'as-is', without any express or implied
warranty.  In no event will the authors be held liable for any damages
arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:
1. The origin of this software must not be misrepresented; you must not
 claim that you wrote the original software. If you use this software
 in a product, an acknowledgment in the product documentation would be
 appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
 misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
package org.recast4j.detour.crowd;

import java.util.*;

import static java.util.stream.Collectors.toList;

public class ProximityGrid {

    private final float cellSize, invCellSize;
    private final Map<ItemKey, List<CrowdAgent>> items;

    public ProximityGrid(float cellSize) {
        this.cellSize = cellSize;
        invCellSize = 1f / cellSize;
        items = new HashMap<>();
    }

    public void clear() {
        items.clear();
    }

    void addItem(CrowdAgent agent, float minXf, float minYf, float maxXf, float maxYf) {
        int minX = (int) Math.floor(minXf * invCellSize);
        int minY = (int) Math.floor(minYf * invCellSize);
        int maxX = (int) Math.floor(maxXf * invCellSize);
        int maxY = (int) Math.floor(maxYf * invCellSize);
        for (int y = minY; y <= maxY; ++y) {
            for (int x = minX; x <= maxX; ++x) {
                ItemKey key = new ItemKey(x, y);
                List<CrowdAgent> ids = items.computeIfAbsent(key, k -> new ArrayList<>());
                ids.add(agent);
            }
        }
    }

    Set<CrowdAgent> queryItems(float minXf, float minYf, float maxXf, float maxYf) {
        int minX = (int) Math.floor(minXf * invCellSize);
        int minY = (int) Math.floor(minYf * invCellSize);
        int maxX = (int) Math.floor(maxXf * invCellSize);
        int maxY = (int) Math.floor(maxYf * invCellSize);
        Set<CrowdAgent> result = new HashSet<>();
        for (int y = minY; y <= maxY; ++y) {
            for (int x = minX; x <= maxX; ++x) {
                ItemKey key = new ItemKey(x, y);
                List<CrowdAgent> ids = items.get(key);
                if (ids != null) {
                    result.addAll(ids);
                }
            }
        }
        return result;
    }

    @SuppressWarnings("unused")
    public List<int[]> getItemCounts() {
        return items.entrySet().stream().filter(e -> e.getValue() != null && e.getValue().size() > 0)
                .map(e -> new int[]{e.getKey().x, e.getKey().y, e.getValue().size()}).collect(toList());
    }

    public float getCellSize() {
        return cellSize;
    }

    private static class ItemKey {

        int x, y;

        public ItemKey(int x, int y) {
            this.x = x;
            this.y = y;
        }

        @Override
        public int hashCode() {
            final int prime = 31;
            int result = 1;
            result = prime * result + x;
            result = prime * result + y;
            return result;
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj)
                return true;
            if (obj == null)
                return false;
            if (getClass() != obj.getClass())
                return false;
            ItemKey other = (ItemKey) obj;
            if (x != other.x)
                return false;
            return y == other.y;
        }

    }

    ;
}
