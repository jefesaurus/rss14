package navigation;

import java.awt.geom.Point2D;
import java.util.LinkedList;
import java.util.List;

public class Constants {
	
	// Robot Parameters
	public static final double ERROR_THRESHOLD = .02;
	public static final double FULL_WIDTH = .46;
	public static final double PLATFORM_WIDTH = .38;
	public static final double WHEEL_WIDTH = (FULL_WIDTH - PLATFORM_WIDTH)/2.;
	public static final double PLATFORM_DEPTH = .44;
	public static final double AXLE_DEPTH = .33;
	public static final double WHEEL_RADIUS = .13 / 2.;
	public static final double MID_WIDTH = FULL_WIDTH / 2.;
	public static final double ORIGIN_X = MID_WIDTH;
	public static final double ORIGIN_Y = AXLE_DEPTH;

	public static Shape createRobot() {
		List<Polygon> polygons = new LinkedList<Polygon>();
		
		//Origin of robot at origin of map facing East is Configuration(0, 0, 0)
		
		//Robot platform
		List<Point> points = new LinkedList<Point>();
		points.add(new Point(MID_WIDTH + PLATFORM_WIDTH/2. + ERROR_THRESHOLD, PLATFORM_DEPTH + ERROR_THRESHOLD)); // Front Right
		points.add(new Point(MID_WIDTH - PLATFORM_WIDTH/2. - ERROR_THRESHOLD, PLATFORM_DEPTH + ERROR_THRESHOLD)); // Front Left
		points.add(new Point(MID_WIDTH - PLATFORM_WIDTH/2. - ERROR_THRESHOLD, 0.0 - ERROR_THRESHOLD)); // Back Left
		points.add(new Point(MID_WIDTH + PLATFORM_WIDTH/2. + ERROR_THRESHOLD, 0.0 - ERROR_THRESHOLD)); // Back Right
		polygons.add(new Polygon(points));
		
		//Right wheel
		points = new LinkedList<Point>();
		points.add(new Point(FULL_WIDTH + ERROR_THRESHOLD, AXLE_DEPTH + WHEEL_RADIUS + ERROR_THRESHOLD)); // Front Right
		points.add(new Point(FULL_WIDTH - WHEEL_WIDTH - ERROR_THRESHOLD, AXLE_DEPTH + WHEEL_RADIUS + ERROR_THRESHOLD)); // Front Left
		points.add(new Point(FULL_WIDTH - WHEEL_WIDTH - ERROR_THRESHOLD, AXLE_DEPTH - WHEEL_RADIUS - ERROR_THRESHOLD)); // Back Left
		points.add(new Point(FULL_WIDTH + ERROR_THRESHOLD, AXLE_DEPTH - WHEEL_RADIUS - ERROR_THRESHOLD)); // Back Right
		polygons.add(new Polygon(points));
		
		//Left wheel
		points = new LinkedList<Point>();
		points.add(new Point(0. - ERROR_THRESHOLD, AXLE_DEPTH + WHEEL_RADIUS + ERROR_THRESHOLD)); // Front Right
		points.add(new Point(0. + WHEEL_WIDTH + ERROR_THRESHOLD, AXLE_DEPTH + WHEEL_RADIUS + ERROR_THRESHOLD)); // Front Left
		points.add(new Point(0. + WHEEL_WIDTH + ERROR_THRESHOLD, AXLE_DEPTH - WHEEL_RADIUS - ERROR_THRESHOLD)); // Back Left
		points.add(new Point(0. - ERROR_THRESHOLD, AXLE_DEPTH - WHEEL_RADIUS - ERROR_THRESHOLD)); // Back Right
		polygons.add(new Polygon(points));
		
		//TODO - method to automatically grow a shape
		
		return new Shape(polygons).translate(-ORIGIN_X, -ORIGIN_Y).rotate(-Math.PI/2, new Point(0., 0.));
	}
	
	// RRT
	public static final int RRT_ATTEMPTS = 3;
	public static final int RRT_ITERATIONS = 500;
	public static final int RRT_SMOOTHING = 100;
	public static final double TRANSLATION_STEP_DISTANCE = .2;
	public static final double ROTATION_STEP_DISTANCE = (Math.PI/6);
	public static final double ROBOT_RADIUS = Math.sqrt(PLATFORM_WIDTH*PLATFORM_WIDTH/4 + PLATFORM_DEPTH*PLATFORM_DEPTH/4);	
}