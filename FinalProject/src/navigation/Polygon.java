package navigation;

import java.awt.geom.Line2D;
import java.util.List;

public class Polygon {
	public final List<Point> points;
	public final BoundingBox boundingBox;
	public final List<Line2D.Double> lines;

	public Polygon(List<Point> points) { // Assumes verticies in increasing
											// angle order
		this.points = points;
		this.lines = GeomUtils.getLines(points);
		this.boundingBox = BoundingBox.computeBoundingBox(points);
	}

	public boolean collides(Polygon other) {
		if (!boundingBox.collides(other.boundingBox)) {
			return false;
		} else if (contains(other.points.get(0))
				|| other.contains(points.get(0))) {
			return true;
		}

		for (Line2D.Double line1 : lines) {
			for (Line2D.Double line2 : other.lines) {
				if (line1.intersectsLine(line2)) { // TODO - line bounding box
													// (but this may already be
													// in the intersects method)
					return true;
				}
			}
		}

		return false;
	}

	public boolean contains(Point point) { // Assumes Convex Polygon
		if (!boundingBox.contains(point)) {
			return false;
		}

		Point previous = points.get(0);
		for (int i = 1; i < points.size(); i++) {
			Point current = points.get(i);
			if (GeomUtils.rightTurn(previous, current, point)) {
				return false;
			}
			previous = current;
		}
		return true;
	}
}