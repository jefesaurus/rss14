package cameraProcessing;

import java.awt.Point;
import java.util.Comparator;

public class BlockInfo implements Comparable<BlockInfo> {
	public Point centroid;
	public float size;
	public String color;
	
	public BlockInfo(Point centroid, float size, String color) {
		this.centroid = centroid;
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