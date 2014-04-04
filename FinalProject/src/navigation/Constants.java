package navigation;

public class Constants {

	public static final double stepDistance = .05;
	public static final double robotRadius = 1;

	// Robot Parameters
	public static final double SCALE = 1.15;
	public static final double FULL_WIDTH = .46 * SCALE;
	public static final double PLATFORM_WIDTH = .38 * SCALE;
	public static final double PLATFORM_DEPTH = .44 * SCALE;
	public static final double AXLE_DEPTH = .33 * SCALE;
	public static final double WHEEL_RADIUS = .13 * SCALE / 2.;
	public static final double MID_WIDTH = FULL_WIDTH / 2.;
	public static final double ORIGIN_X = MID_WIDTH;
	public static final double ORIGIN_Y = AXLE_DEPTH;

	// RRT
	public static final int RRT_ATTEMPTS = 1;
	public static final int RRT_ITERATIONS = 100;
	public static final int RRT_SMOOTHING = 100;
}