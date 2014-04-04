package navigation;

import java.util.LinkedList;
import java.util.List;

public class Configuration {
	public final double x, y, theta;

	public Configuration(double x, double y, double theta) {
		this.x = x;
		this.y = y;
		this.theta = GeomUtils.cleanAngle(theta);
	}

	private List<Configuration> interpolateTranslate(double x1, double y1,
			double x2, double y2, double angle) {
		List<Configuration> configs = new LinkedList<Configuration>();
		double translateDistance = GeomUtils.vectorLength(x2 - x1, y2 - y1);
		for (double dist = Constants.stepDistance; dist < translateDistance; dist += Constants.stepDistance) {
			configs.add(new Configuration(x1 + dist * (x2 - x1)
					/ translateDistance, y1 + dist * (y2 - y1)
					/ translateDistance, angle));
		}
		configs.add(new Configuration(x2, y2, angle));

		return configs;
	}

	private List<Configuration> interpolateRotate(double angle1, double angle2,
			double x, double y) {
		List<Configuration> configs = new LinkedList<Configuration>();
		double angleStepDistance = Constants.stepDistance
				/ Constants.robotRadius;
		if (Math.abs(angle1 - angle2) < 2 * Math.PI - Math.abs(angle1 - angle2)) {
			double angleDistance = Math.abs(angle1 - angle2);
			for (double dist = angleStepDistance; dist < angleDistance; dist += angleStepDistance) {
				configs.add(new Configuration(x, y, angle1 + dist
						* Math.signum(angle2 - angle1)));
			}
		} else {
			double dist;
			if (angle1 < angle2) {
				for (dist = angleStepDistance; dist < angle1; dist += angleStepDistance) {
					configs.add(new Configuration(x, y, angle1 - dist));
				}
				for (dist = dist - angle1; dist < 2 * Math.PI - angle2; dist += angleStepDistance) {
					configs.add(new Configuration(x, y, 2 * Math.PI - dist));
				}
			} else {
				for (dist = angleStepDistance; dist < 2 * Math.PI - angle1; dist += angleStepDistance) {
					configs.add(new Configuration(x, y, angle1 + dist));
				}
				for (dist = dist - (2 * Math.PI - angle1); dist < angle2; dist += angleStepDistance) {
					configs.add(new Configuration(x, y, 0 + dist));
				}
			}
		}
		configs.add(new Configuration(x, y, angle2));

		return configs;
	}

	public List<Configuration> interpolatePath(Configuration end) {
		double translateAngle = GeomUtils.vectorAngle(end.x - x, end.y - y);
		List<Configuration> path = interpolateRotate(theta, translateAngle, x,
				y);
		path.addAll(interpolateTranslate(x, y, end.x, end.y, translateAngle));
		path.addAll(interpolateRotate(translateAngle, end.theta, end.x, end.y));
		return path;
	}

	public double distance(Configuration other) {
		double translateAngle = GeomUtils.vectorAngle(other.x - x, other.y - y);
		return Constants.robotRadius
				* GeomUtils.angleDistance(theta, translateAngle)
				+ GeomUtils.vectorLength(other.x - x, other.y - y)
				+ Constants.robotRadius
				* GeomUtils.angleDistance(translateAngle, other.theta);
	}

	public String toString() {
		return "Configuration(" + x + ", " + y + ", " + theta + ")";
	}
}