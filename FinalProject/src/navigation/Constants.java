package navigation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

public class Constants {
	
	// Robot Parameters
	public static final double FULL_WIDTH = .46;
	public static final double PLATFORM_WIDTH = .38;
	public static final double WHEEL_WIDTH = (FULL_WIDTH - PLATFORM_WIDTH)/2.;
	public static final double PLATFORM_DEPTH = .44;
	public static final double AXLE_DEPTH = .33;
	public static final double WHEEL_RADIUS = .13 / 2.;
	public static final double MID_WIDTH = FULL_WIDTH / 2.;
	public static final double ORIGIN_X = MID_WIDTH;
	public static final double ORIGIN_Y = AXLE_DEPTH;

	public static Shape createRobot(double grow) { //TODO - multiplicative instead of additive grow?
		List<Polygon> polygons = new LinkedList<Polygon>();
		
		//Origin of robot at origin of map facing East is Configuration(0, 0, 0)
		
		//Robot platform
		List<Point> points = new LinkedList<Point>();
		points.add(new Point(MID_WIDTH + PLATFORM_WIDTH/2. + grow, PLATFORM_DEPTH + grow)); // Front Right
		points.add(new Point(MID_WIDTH - PLATFORM_WIDTH/2. - grow, PLATFORM_DEPTH + grow)); // Front Left
		points.add(new Point(MID_WIDTH - PLATFORM_WIDTH/2. - grow, 0.0 - grow)); // Back Left
		points.add(new Point(MID_WIDTH + PLATFORM_WIDTH/2. + grow, 0.0 - grow)); // Back Right
		polygons.add(new Polygon(points));
				
		//Wheel position moves under growth
		//Right wheel
		/*points = new LinkedList<Point>();
		points.add(new Point(FULL_WIDTH + 2*grow, AXLE_DEPTH + WHEEL_RADIUS + grow)); // Front Right
		points.add(new Point(FULL_WIDTH - WHEEL_WIDTH, AXLE_DEPTH + WHEEL_RADIUS + grow)); // Front Left
		points.add(new Point(FULL_WIDTH - WHEEL_WIDTH, AXLE_DEPTH - WHEEL_RADIUS - grow)); // Back Left
		points.add(new Point(FULL_WIDTH + 2*grow, AXLE_DEPTH - WHEEL_RADIUS - grow)); // Back Right
		polygons.add(new Polygon(points));
		
		//Left wheel
		points = new LinkedList<Point>();
		points.add(new Point(0. - 2*grow, AXLE_DEPTH + WHEEL_RADIUS + grow)); // Front Right
		points.add(new Point(0. + WHEEL_WIDTH, AXLE_DEPTH + WHEEL_RADIUS + grow)); // Front Left
		points.add(new Point(0. + WHEEL_WIDTH, AXLE_DEPTH - WHEEL_RADIUS - grow)); // Back Left
		points.add(new Point(0. - 2*grow, AXLE_DEPTH - WHEEL_RADIUS - grow)); // Back Right
		polygons.add(new Polygon(points));*/
		
		//Wheel position constant under growth
		//Right wheel
		points = new LinkedList<Point>();
		points.add(new Point(FULL_WIDTH + grow, AXLE_DEPTH + WHEEL_RADIUS + grow)); // Front Right
		points.add(new Point(FULL_WIDTH - WHEEL_WIDTH - grow, AXLE_DEPTH + WHEEL_RADIUS + grow)); // Front Left
		points.add(new Point(FULL_WIDTH - WHEEL_WIDTH - grow, AXLE_DEPTH - WHEEL_RADIUS - grow)); // Back Left
		points.add(new Point(FULL_WIDTH + grow, AXLE_DEPTH - WHEEL_RADIUS - grow)); // Back Right
		polygons.add(new Polygon(points));
		
		//Left wheel
		points = new LinkedList<Point>();
		points.add(new Point(0. - grow, AXLE_DEPTH + WHEEL_RADIUS + grow)); // Front Right
		points.add(new Point(0. + WHEEL_WIDTH + grow, AXLE_DEPTH + WHEEL_RADIUS + grow)); // Front Left
		points.add(new Point(0. + WHEEL_WIDTH + grow, AXLE_DEPTH - WHEEL_RADIUS - grow)); // Back Left
		points.add(new Point(0. - grow, AXLE_DEPTH - WHEEL_RADIUS - grow)); // Back Right
		polygons.add(new Polygon(points));
		
		//TODO - method to automatically grow any shape
		
		return new Shape(polygons).translate(-ORIGIN_X, -ORIGIN_Y).rotate(-Math.PI/2, new Point(0., 0.));
	}
	
	public static final double VIEW_CONE_WIDTH = 1.;
	public static final double VIEW_CONE_HEIGHT = 1.;
	
	public static Polygon createViewCone() {
		List<Point> points = new LinkedList<Point>();
		points.add(new Point(.05, 0.)); // Origin
		points.add(new Point(VIEW_CONE_WIDTH/2, VIEW_CONE_HEIGHT)); // Front Right
		points.add(new Point(-VIEW_CONE_WIDTH/2, VIEW_CONE_HEIGHT)); // Front Left
		points.add(new Point(-.05, 0.)); // Origin
		return new Polygon(points).translate(0, ORIGIN_Y/2).rotate(-Math.PI/2, new Point(0., 0.));
	}
	
	// RRT
	public static final int RRT_ATTEMPTS = 2;
	public static final int RRT_ITERATIONS = 500;
	public static final int RRT_SMOOTHING = 200;
	public static final double TRANSLATION_STEP_DISTANCE = .1; //Previously .2
	public static final double ROTATION_STEP_DISTANCE = (Math.PI/6);
	public static final double ROBOT_RADIUS = Math.sqrt(FULL_WIDTH*FULL_WIDTH/4 + PLATFORM_DEPTH*PLATFORM_DEPTH/4);	//TOOD make more accurate/include error
	
	public enum DriveSystem {
	    FORWARD, BACKWARD, FOB
	}
	
	public static DriveSystem computeDrive(DriveSystem drive, boolean reverse) {
		if (!reverse) {
			return drive;
		} else {
			switch (drive) {
			case FORWARD: return DriveSystem.BACKWARD;
			case BACKWARD: return DriveSystem.FORWARD;
			default: return drive;
			}
		}
	}
	
	public enum CollisionCheck {
	    MAPONLY, GRIDONLY, MAPANDGRID
	}
		
	// Planning
	@SuppressWarnings("unchecked")
	public static final List<Pair<DriveSystem, Pair<Double, CollisionCheck>>> PLANNING_ATTEMPTS = new ArrayList<Pair<DriveSystem, Pair<Double, CollisionCheck>>>(Arrays.asList(
		new Pair<DriveSystem, Pair<Double, CollisionCheck>>(DriveSystem.FORWARD, new Pair<Double, CollisionCheck>(.05, CollisionCheck.MAPONLY)),
		new Pair<DriveSystem, Pair<Double, CollisionCheck>>(DriveSystem.FORWARD, new Pair<Double, CollisionCheck>(.025, CollisionCheck.MAPONLY)),
		new Pair<DriveSystem, Pair<Double, CollisionCheck>>(DriveSystem.FOB, new Pair<Double, CollisionCheck>(.025, CollisionCheck.MAPONLY)),
		new Pair<DriveSystem, Pair<Double, CollisionCheck>>(DriveSystem.FOB, new Pair<Double, CollisionCheck>(0., CollisionCheck.MAPONLY))
	));
	
	// Grids
	public static final double gridResolution = .02;
	
	// Motor Controllers
	public static final double TRANSLATION_THRESHOLD = .05; 	//TODO - play with velocities more using interpolation
	public static final double K_TRANSLATE = 2.0;
	public static final double MAX_TV = 1.5;
	public static final double MIN_TV = .25;
	
	public static final double ROTATION_THRESHOLD = .2;
	public static final double K_ROTATE = .5;
	public static final double MAX_RV = 1.0;
	public static final double MIN_RV = 0.2;
}