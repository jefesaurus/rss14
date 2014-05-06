package navigation;

import java.util.LinkedList;
import java.util.List;

public class Point implements Goal {
	public final double x, y;

	public Point(double x, double y) {
		this.x = x;
		this.y = y;
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

	public String toString() {
		return "Point(" + x + ", " + y + ")";
	}
}