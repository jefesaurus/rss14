package navigation;

import java.util.LinkedList;
import java.util.List;

public class Point implements Goal {
	public final double x, y;

	public Point(double x, double y) {
		this.x = x;
		this.y = y;
	}
	
	public Point forwardTransform(Configuration other) {
		double length = Util.vectorLength(x, y);
		double angle = Util.vectorAngle(x, y) + other.theta;
		return new Point(length*Math.cos(angle) + other.x, length*Math.sin(angle) + other.y);	
	}
	
	public Point inverseTransform(Configuration other) {
		double length = Util.vectorLength(x - other.x, y - other.y);
		double angle = Util.vectorAngle(x - other.x, y - other.y) - other.theta;
		return new Point(length*Math.cos(angle), length*Math.sin(angle));	
	}

	public double distance(Point other) {
		return Math.sqrt((other.x - x) * (other.x - x) + (other.y - y)* (other.y - y));
	}
	
	public Configuration configuration(double theta) {
		return new Configuration(x, y, theta);
	}
	
	public List<Configuration> goalConfigurations() {
		List<Configuration> configs = new LinkedList<Configuration>();
		for (double theta = 0.0; theta < 2*Math.PI; theta += Constants.ROTATION_STEP_DISTANCE) {
			configs.add(new Configuration(x, y, theta));
		}
		return configs;
	}
	
	public double goalDistance(Configuration config) {
		return Math.sqrt((config.x - x) * (config.x - x) + (config.y - y)* (config.y - y));
	}

	public String toString() {
		return "Point(" + x + ", " + y + ")";
	}
}