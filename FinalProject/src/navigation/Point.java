package navigation;

public class Point {
	public final double x, y;

	public Point(double x, double y) {
		this.x = x;
		this.y = y;
	}

	public double distance(Point other) {
		return Math.sqrt((other.x - x) * (other.x - x) + (other.y - y)
				* (other.y - y));
	}
	
	public Configuration configuration(double theta) {
		return new Configuration(x, y, theta);
	}

	public String toString() {
		return "Point(" + x + ", " + y + ")";
	}
}