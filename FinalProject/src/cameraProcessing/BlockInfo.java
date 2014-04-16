package cameraProcessing;

import java.awt.Point;

public class BlockInfo {
	public Point centroid;
	public float size;
	public String color;
	
	public BlockInfo(Point centroid, float size, String color) {
		this.centroid = centroid;
		this.size = size;
		this.color = color;
	}
}
