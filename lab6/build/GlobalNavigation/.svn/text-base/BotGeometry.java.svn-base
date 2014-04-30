package GlobalNavigation;

import java.awt.geom.Point2D;
import java.util.LinkedList;
import java.util.List;


public class BotGeometry {
	List<Point2D.Double> ComplexRobotGeometry;
	List<Point2D.Double> SimpleRobotGeometry;
	
	// Main platform is 38cm wide, 44cm long including bump sensors(but not whiskers)
	// 46cm wide including wheels
	// Wheels have diameter of 13 cm
	// Wheel axle is 33 cm from back of platform
	/*
	double FULL_WIDTH = .46;
	double PLATFORM_WIDTH = .38;
	double PLATFORM_DEPTH = .44;
	double AXLE_DEPTH = .33;
	double WHEEL_RADIUS = .13/2.;
	double MID_WIDTH = FULL_WIDTH/2.;
	double ORIGIN_X = MID_WIDTH;
	double ORIGIN_Y = AXLE_DEPTH;
	*/
	double SCALE = 1.15;
	double FULL_WIDTH = .46*SCALE;
	double PLATFORM_WIDTH = .38*SCALE;
	double PLATFORM_DEPTH = .44*SCALE;
	double AXLE_DEPTH = .33*SCALE;
	double WHEEL_RADIUS = .13*SCALE/2.;
	double MID_WIDTH = FULL_WIDTH/2.;
	double ORIGIN_X = MID_WIDTH;
	double ORIGIN_Y = AXLE_DEPTH;
	
	public BotGeometry() {
		ComputeBotGeometry();
		ComputeSimpleBotGeometry();
	}
	
	public PolygonObstacle getBotPoly(double theta) {
		return GeomUtils.convexHull(GeomUtils.RotatePoints(SimpleRobotGeometry, theta, new Point2D.Double(0,0)));
	}
	
	// Polygon that includes wheel detail.
	public void ComputeBotGeometry() {

		List<Point2D.Double> botHull = new LinkedList<Point2D.Double>();

		botHull.add(new Point2D.Double(MID_WIDTH + PLATFORM_WIDTH/2., PLATFORM_DEPTH)); // Front Right
		botHull.add(new Point2D.Double(MID_WIDTH - PLATFORM_WIDTH/2., PLATFORM_DEPTH)); // Front Left
		botHull.add(new Point2D.Double(MID_WIDTH + PLATFORM_WIDTH/2., 0.0)); // Back Right
		botHull.add(new Point2D.Double(MID_WIDTH - PLATFORM_WIDTH/2., 0.0)); // Back Left
		botHull.add(new Point2D.Double(0.0, AXLE_DEPTH + WHEEL_RADIUS)); // Left Wheel Front
		botHull.add(new Point2D.Double(0.0, AXLE_DEPTH - WHEEL_RADIUS)); // Left Wheel Back
		botHull.add(new Point2D.Double(FULL_WIDTH, AXLE_DEPTH + WHEEL_RADIUS)); // Right Wheel Front
		botHull.add(new Point2D.Double(FULL_WIDTH, AXLE_DEPTH - WHEEL_RADIUS)); // Left Wheel Back
		this.ComplexRobotGeometry = GeomUtils.TranslatePoints(botHull, -ORIGIN_X, -ORIGIN_Y);
	}

	// The circumscribed rectangle ( Fewer points than with wheels )
	public void ComputeSimpleBotGeometry() {
		List<Point2D.Double> botHull = new LinkedList<Point2D.Double>();
		  
		botHull.add(new Point2D.Double(FULL_WIDTH, PLATFORM_DEPTH)); // Front Right
		botHull.add(new Point2D.Double(0.0, PLATFORM_DEPTH)); // Front Left
		botHull.add(new Point2D.Double(FULL_WIDTH, 0.0)); // Back Right
		botHull.add(new Point2D.Double(0.0, 0.0)); // Back Left
		this.SimpleRobotGeometry = GeomUtils.TranslatePoints(botHull, -ORIGIN_X, -ORIGIN_Y);
	}
}
