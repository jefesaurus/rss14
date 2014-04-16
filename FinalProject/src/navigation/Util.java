package navigation;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.LinkedList;
import java.util.List;

public class Util {

	public static double vectorLength(double x, double y) {
		return Math.sqrt(x * x + y * y);
	}

	public static double vectorAngle(double x, double y) {
		return cleanAngle(Math.atan2(y, x));
	}

	public static double angleDistance(double one, double two) {
		return Math.min(Math.abs(one - two), 2 * Math.PI - Math.abs(one - two));
	}

	public static double cleanAngle(double angle) {
		while (angle < 0) {
			angle += 2 * Math.PI;
		}
		while (angle >= 2 * Math.PI) {
			angle -= 2 * Math.PI;
		}
		return angle;
	}

	public static boolean rightTurn(Point p0, Point p1, Point p2) {
		double p02x = p2.x - p0.x;
		double p02y = p2.y - p0.y;
		double p01x = p1.x - p0.x;
		double p01y = p1.y - p0.y;
		return (p02x * p01y - p02y * p01x) > 0.0;
	}

	public static List<Line2D.Double> getLines(List<Point> points) {
		LinkedList<Line2D.Double> lines = new LinkedList<Line2D.Double>();
		Point2D.Double previous = new Point2D.Double(points.get(0).x,
				points.get(0).y);
		for (int i = 1; i < points.size(); i++) {
			Point2D.Double current = new Point2D.Double(points.get(i).x,
					points.get(i).y);
			lines.add(new Line2D.Double(previous, current));
			previous = current;
		}
		lines.add(new Line2D.Double(previous, new Point2D.Double(
				points.get(0).x, points.get(0).y)));
		return lines;
	}
	
	public static List<Point2D.Double> convertToPoint2D(List<Point> points) {
		List<Point2D.Double> points2D = new LinkedList<Point2D.Double>();
		for (Point p : points) {
			points2D.add(new Point2D.Double(p.x, p.y));
		}
		return points2D;
	}
}