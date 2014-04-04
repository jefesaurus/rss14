package Chassis;

//import MotorControlSolution.*;
import MotorControl.*;

/**
 * <p>A whole-robot position controller.</p>
 **/
public class RobotPositionController {

  /**
   * <p> translationalVelocityToAngularVelocity </p>
   **/
  protected final double translationalVelocityToAngularVelocity = 2*Math.PI/.395;

  /**
   * <p> ticksToMeters </p>
   **/
  protected final double ticksToMeters = translationalVelocityToAngularVelocity/4.8*Math.pow(10, 5);

  /**
   * <p>The whole-robot velocity controller.</p>
   **/
  protected RobotVelocityController robotVelocityController;

  /**
   * <p>Total ticks since reset, positive means corresp side of robot moved
   * forward.</p>
   **/
  protected double[] totalTicks = new double[2];

  /**
   * <p>Total elapsed time since reset in seconds.</p>
   **/
  protected double totalTime = 0.0;

  /**
   * <p>Time in seconds since last update.</p>
   **/
  protected double sampleTime;

  /**
   * <p>An abstract gain; meaning depends on the particular subclass
   * implementation.</p>
   **/
  protected double gain = 1.0;

  /**
   * <p>The robot.</p>
   **/
  protected OdometryRobot robot;

  /**
   * <p>Create a new position controller for a robot.</p>
   *
   * @param robot the robot, not null
   **/
  public RobotPositionController(OdometryRobot robot) {
    this.robot = robot;
  }

  /**
   * <p>Translate at the specified speed for the specified distance.</p>
   *
   * <p>Blocks until the motion is complete or errored.</p>
   *
   * @param speed the desired robot motion speed in m/s
   * @param distance the desired distance to move in meters, relative to the
   * robot's pose at time of call.
   *
   * @return true iff motion was successful
   **/
   // Translation Motion constants
   private final double TRANSLATE_ERROR_FACTOR = .005;
   private final double TRANSLATE_RAMP_DISTANCE = .5;
   private final double TRANSLATE_MIN_SPEED = .05;
  public boolean translate(double speed, double distance) {
    boolean ok = true;

    // Bookkeeping variables for tracking angular/linear speeds and distances
    double angularVelocity = speed*translationalVelocityToAngularVelocity;
    double minAngularVelocity = TRANSLATE_MIN_SPEED*translationalVelocityToAngularVelocity;
    robot.enableMotors(true);
    double[] startTicks = new double[2];
    startTicks[RobotBase.LEFT] = totalTicks[RobotBase.LEFT];
    startTicks[RobotBase.RIGHT] = totalTicks[RobotBase.RIGHT];

    double distanceLeft = 0.0;
    double distanceRight = 0.0;
    double leftError = distance;
    double rightError = distance;
    double leftAngularVelocity = 0.0;
    double rightAngularVelocity = 0.0;

    // Main movement loop
    while (Math.abs(leftError) > TRANSLATE_ERROR_FACTOR*Math.abs(distance) && Math.abs(rightError) > TRANSLATE_ERROR_FACTOR*Math.abs(distance)) {
      synchronized(this) {
        distanceLeft = (totalTicks[RobotBase.LEFT]-startTicks[RobotBase.LEFT])/ticksToMeters;
        distanceRight = (totalTicks[RobotBase.RIGHT]-startTicks[RobotBase.RIGHT])/ticksToMeters;
        leftError = distance - distanceLeft;
        rightError = distance - distanceRight;

        // Ramping the velocity to slow acceleration and minimize slippage
        if (Math.abs(leftError) < TRANSLATE_RAMP_DISTANCE) {
          leftAngularVelocity = angularVelocity*(Math.abs(leftError)/TRANSLATE_RAMP_DISTANCE)*gain;
        } else {
          leftAngularVelocity = angularVelocity*gain;
        }

        if (Math.abs(rightError) < TRANSLATE_RAMP_DISTANCE) {
          rightAngularVelocity = angularVelocity*(Math.abs(rightError)/TRANSLATE_RAMP_DISTANCE)*gain;
        } else {
          rightAngularVelocity = angularVelocity*gain;
        }

        // Clip velocity to a minimum to avoid completely asymptotic approach
        leftAngularVelocity = (leftAngularVelocity < minAngularVelocity)?minAngularVelocity:leftAngularVelocity;
        rightAngularVelocity = (rightAngularVelocity < minAngularVelocity)?minAngularVelocity:rightAngularVelocity;
        robotVelocityController.setDesiredAngularVelocity(Math.signum(leftError)*leftAngularVelocity, Math.signum(rightError)*rightAngularVelocity);
      }
    }
    robotVelocityController.setDesiredAngularVelocity(0.0, 0.0);
    // End Student Code
    return ok;
  }

  /**
   * <p>Rotate at the specified speed for the specified angle.</p>
   *
   * <p>Blocks until the motion is complete or errored.</p>
   *
   * @param speed the desired robot motion speed in radians/s
   * @param angle the desired angle to rotate in radians, relative to the
   * robot's pose at time of call.
   *
   * @return true iff motion was successful
   **/


   // Rotational Motion constants
   private final double WHEEL_RADIUS = .063;
   private final double ROBOT_RADIUS = .22;

   private final double ROTATE_ERROR_FACTOR = .005;
   private final double ROTATE_RAMP_DISTANCE = Math.PI/4*ROBOT_RADIUS;
   private final double ROTATE_MIN_SPEED = (Math.PI/2)/8;

  public boolean rotate(double speed, double angle) {
    boolean ok = true;

    // Rotation bookkeeping variables
    double distance = angle*ROBOT_RADIUS;
    double angularVelocity = speed*ROBOT_RADIUS/WHEEL_RADIUS;
    double minAngularVelocity = ROTATE_MIN_SPEED*ROBOT_RADIUS/WHEEL_RADIUS;
    robot.enableMotors(true);
    double[] startTicks = new double[2];
    startTicks[RobotBase.LEFT] = totalTicks[RobotBase.LEFT];
    startTicks[RobotBase.RIGHT] = totalTicks[RobotBase.RIGHT];

    double distanceLeft = 0.0;
    double distanceRight = 0.0;
    double leftError = -distance;
    double rightError = distance;
    double leftAngularVelocity = 0.0;
    double rightAngularVelocity = 0.0;

    // Main movement loop
    while (Math.abs(leftError) > ROTATE_ERROR_FACTOR*Math.abs(distance) && Math.abs(rightError) > ROTATE_ERROR_FACTOR*Math.abs(distance)) {
      synchronized(this) {
        distanceLeft = (totalTicks[RobotBase.LEFT]-startTicks[RobotBase.LEFT])/ticksToMeters;
        distanceRight = (totalTicks[RobotBase.RIGHT]-startTicks[RobotBase.RIGHT])/ticksToMeters;
        leftError = -distance - distanceLeft;
        rightError = distance - distanceRight;
	
        // Ramping rotation speed to minimize slippage
        if (Math.abs(leftError) < ROTATE_RAMP_DISTANCE) {
          leftAngularVelocity = angularVelocity*(Math.abs(leftError)/ROTATE_RAMP_DISTANCE)*gain;
        } else {
          leftAngularVelocity = angularVelocity*gain;
        }

        if (Math.abs(rightError) < ROTATE_RAMP_DISTANCE) {
          rightAngularVelocity = angularVelocity*(Math.abs(rightError)/ROTATE_RAMP_DISTANCE)*gain;
        } else {
          rightAngularVelocity = angularVelocity*gain;
        }

        // Clip velocity to a minimum to avoid completely asymptotic approach
        leftAngularVelocity = (leftAngularVelocity < minAngularVelocity)?minAngularVelocity:leftAngularVelocity;
        rightAngularVelocity = (rightAngularVelocity < minAngularVelocity)?minAngularVelocity:rightAngularVelocity;
        robotVelocityController.setDesiredAngularVelocity(Math.signum(leftError)*leftAngularVelocity, Math.signum(rightError)*rightAngularVelocity);
      }
    }
    
    robotVelocityController.setDesiredAngularVelocity(0.0, 0.0);
    return ok;	
  }
    

    

  /**
   * <p>If position control is closed-loop, this computes the new left and
   * right velocity commands and issues them to {@link
   * #robotVelocityController}.</p>
   **/
  public synchronized void controlStep() {

    if (robotVelocityController == null)
      return;

    if (!robot.motorsEnabled() || robot.estopped())
      return;

    // Begin Student Code (if implementing closed-loop control)
    // End Student Code (if implementing closed-loop control)
  }

  /**
   * <p>Set the whole-robot velocity controller.</p>
   *
   * <p>This is called automatically by {@link OdometeryRobot}.</p>
   *
   * @param vc the whole-robot velocity controller
   **/
  public void setRobotVelocityController(RobotVelocityController vc) {
    robotVelocityController = vc;
  }

  /**
   * <p>Set {@link #gain}.</p>
   *
   * @param g the new gain
   **/
  public void setGain(double g) {
    gain = g;
  }

  /**
   * <p>Get {@link #gain}.</p>
   *
   * @return gain
   **/
  public double getGain() {
    return gain;
  }

  /**
   * <p>Update feedback and sample time.</p>
   *
   * @param time the time in seconds since the last update, saved to {@link
   * #sampleTime}
   * @param leftTicks left encoder ticks since last update, positive means
   * corresp side of robot rolled forward
   * @param rightTicks right encoder ticks since last update, positive means
   * corresp side of robot robot rolled forward
   **/
  public synchronized void update(double time,
                                  double leftTicks, double rightTicks) {

    sampleTime = time;

    totalTicks[RobotBase.LEFT] += leftTicks;
    totalTicks[RobotBase.RIGHT] += rightTicks;
    totalTime += time;
  }
}
