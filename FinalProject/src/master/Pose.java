package master;

public class Pose {
	public double x, y, theta;
	public Pose(double x, double y, double theta) {
		this.x = x;
		this.y = y;
		this.theta = theta;
	}
	
	public double distance(Pose other) {
		double dx = this.x - other.x;
		double dy = this.y - other.y;
		return Math.sqrt(dx*dx + dy*dy);
	}
}
