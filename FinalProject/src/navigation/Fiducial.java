package navigation;

import java.awt.Color;

public class Fiducial {
	public final Point center;
	public final double radius;
	public final Color color;

	public Fiducial(Point center, double radius, Color color) {
		this.center = center;
		this.radius = radius;
		this.color = color;
	}
}