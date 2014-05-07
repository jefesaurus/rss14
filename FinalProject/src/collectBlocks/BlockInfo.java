package collectBlocks;

import java.awt.Point;
import java.awt.geom.Point2D;
import java.util.Comparator;

import navigation.Point3D;

public class BlockInfo implements Comparable<BlockInfo> {
	public Point2D centroid;
	public Point3D location3D;
	public float size;
	public String color;
	
	public BlockInfo(Point2D centroid, Point3D location3D, float size, String color) {
		this.centroid = centroid;
		this.location3D = location3D;
		this.size = size;
		this.color = color;
	}

	@Override
	public int compareTo(BlockInfo y) {
		return this.size < y.size ? -1
				: this.size > y.size ? 1
				: 0;
	}
}