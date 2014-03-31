package MotorControl;

/**
 * <p>Feed-forward wheel velocity controller.</p>
 *
 * @author vona
 * @author prentice
 **/
public class WheelVelocityControllerFF extends WheelVelocityController {

  /**
   * {@inheritDoc}
   *
   * <p>This impl implements simple feed-forward control.</p>
   **/
  public double controlStep() {
    // result is the PWD command
    double scalingFactor = 255./10.0;
    double desAngVel = this.getDesiredAngularVelocity();
    double result = scalingFactor*desAngVel*this.gain;
    return result;
  }

  /**
   * {@inheritDoc}
   *
   * <p>This impl returns "FF".</p>
   **/
  public String getName() {
    return "FF";
  }
}
