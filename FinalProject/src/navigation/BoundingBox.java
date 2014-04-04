package navigation;

import java.util.List;

public class BoundingBox {
	public final Point min, max;

	public BoundingBox(Point min, Point max) {
		this.min = min;
		this.max = max;
	}

	public boolean collides(BoundingBox other) {
		return !(min.x > other.max.x || max.x < other.min.x
				|| min.y > other.max.y || max.y < other.min.y);
	}

	public boolean contains(Point point) {
		return min.x < point.x && max.x > point.x && min.y < point.y
				&& max.y > point.y;
	}

	public static BoundingBox computeBoundingBox(List<Point> points) {
		if (points.size() == 0) {
			throw new RuntimeException();
		}

		double minX, minY, maxX, maxY;
		minX = maxX = points.get(0).x;
		minY = maxY = points.get(0).y;

		for (int i = 1; i < points.size(); i++) {
			Point point = points.get(i);
			minX = Math.min(minX, point.x);
			minY = Math.min(minY, point.y);
			maxX = Math.max(maxX, point.x);
			maxY = Math.max(maxY, point.y);
		}
		return new BoundingBox(new Point(minX, minY), new Point(maxX, maxY));
	}
}