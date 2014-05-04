package navigation;

import java.util.LinkedList;
import java.util.List;

import navigation.Constants.DriveSystem;

public class Configuration {
	public final double x, y, theta;

	public Configuration(double x, double y, double theta) {
		this.x = x;
		this.y = y;
		this.theta = Util.cleanAngle(theta);
	}
	
	public Configuration forwardTransform(Configuration other) {
		double length = Util.vectorLength(x, y);
		double angle = Util.vectorAngle(x, y) + other.theta;
		return new Configuration(length*Math.cos(angle) + other.x, length*Math.sin(angle) + other.y, theta + other.theta);	
	}
	
	public Configuration inverseTransform(Configuration other) {
		double length = Util.vectorLength(x - other.x, y - other.y);
		double angle = Util.vectorAngle(x - other.x, y - other.y) - other.theta;
		return new Configuration(length*Math.cos(angle), length*Math.sin(angle), theta - other.theta);	
	}

	private List<Configuration> interpolateTranslate(double x1, double y1,
			double x2, double y2, double angle) {
		List<Configuration> configs = new LinkedList<Configuration>();
		double translateDistance = Util.vectorLength(x2 - x1, y2 - y1);
		for (double dist = Constants.TRANSLATION_STEP_DISTANCE; dist < translateDistance; dist += Constants.TRANSLATION_STEP_DISTANCE) {
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
		if (Math.abs(angle1 - angle2) < 2 * Math.PI - Math.abs(angle1 - angle2)) {
			double angleDistance = Math.abs(angle1 - angle2);
			for (double dist = Constants.ROTATION_STEP_DISTANCE; dist < angleDistance; dist += Constants.ROTATION_STEP_DISTANCE) {
				configs.add(new Configuration(x, y, angle1 + dist
						* Math.signum(angle2 - angle1)));
			}
		} else {
			double dist;
			if (angle1 < angle2) {
				for (dist = Constants.ROTATION_STEP_DISTANCE; dist < angle1; dist += Constants.ROTATION_STEP_DISTANCE) {
					configs.add(new Configuration(x, y, angle1 - dist));
				}
				for (dist = dist - angle1; dist < 2 * Math.PI - angle2; dist += Constants.ROTATION_STEP_DISTANCE) {
					configs.add(new Configuration(x, y, 2 * Math.PI - dist));
				}
			} else {
				for (dist = Constants.ROTATION_STEP_DISTANCE; dist < 2 * Math.PI - angle1; dist += Constants.ROTATION_STEP_DISTANCE) {
					configs.add(new Configuration(x, y, angle1 + dist));
				}
				for (dist = dist - (2 * Math.PI - angle1); dist < angle2; dist += Constants.ROTATION_STEP_DISTANCE) {
					configs.add(new Configuration(x, y, 0 + dist));
				}
			}
		}
		configs.add(new Configuration(x, y, angle2));

		return configs;
	}

	public List<Configuration> interpolateForwardPath(Configuration end) {
		double translateAngle = Util.vectorAngle(end.x - x, end.y - y);
		List<Configuration> path = interpolateRotate(theta, translateAngle, x, y);
		path.addAll(interpolateTranslate(x, y, end.x, end.y, translateAngle));
		path.addAll(interpolateRotate(translateAngle, end.theta, end.x, end.y));
		return path;
	}
	
	public List<Configuration> interpolateBackwardPath(Configuration end) {
		double translateAngle = Util.cleanAngle(Util.vectorAngle(end.x - x, end.y - y) + Math.PI);
		List<Configuration> path = interpolateRotate(theta, translateAngle, x, y);
		path.addAll(interpolateTranslate(x, y, end.x, end.y, translateAngle));
		path.addAll(interpolateRotate(translateAngle, end.theta, end.x, end.y));
		return path;
	}
	
	public List<Configuration> interpolateFOBPath(Configuration end) {
		if (distanceForward(end) < distanceBackward(end)) {
			return interpolateForwardPath(end);
		} else {
			return interpolateBackwardPath(end);
		}
	}
	
	public List<Configuration> interpolatePath(Configuration end, DriveSystem drive) {
		switch(drive) {
		case FORWARD: return interpolateForwardPath(end);
		case BACKWARD: return interpolateBackwardPath(end);
		case FOB: return interpolateFOBPath(end);
		default: return null;
		}
	}

	public double distanceForward(Configuration other) {
		double translateAngle = Util.vectorAngle(other.x - x, other.y - y);
		return Constants.ROBOT_RADIUS
				* Util.angleDistance(theta, translateAngle)
				+ Util.vectorLength(other.x - x, other.y - y)
				+ Constants.ROBOT_RADIUS
				* Util.angleDistance(translateAngle, other.theta);
	}
	
	public double distanceBackward(Configuration other) {
		double translateAngle = Util.cleanAngle(Util.vectorAngle(other.x - x, other.y - y) + Math.PI);
		return Constants.ROBOT_RADIUS
				* Util.angleDistance(theta, translateAngle)
				+ Util.vectorLength(other.x - x, other.y - y)
				+ Constants.ROBOT_RADIUS
				* Util.angleDistance(translateAngle, other.theta);
	}
	
	public double distanceFOB(Configuration other) {
		return Math.min(distanceForward(other), distanceBackward(other));
	}
	
	public double distance(Configuration other, DriveSystem drive) {
		switch(drive) {
		case FORWARD: return distanceForward(other);
		case BACKWARD: return distanceBackward(other);
		case FOB: return distanceFOB(other);
		default: return -1.;
		}
	}

	public String toString() {
		return "Configuration(" + x + ", " + y + ", " + theta + ")";
	}
}