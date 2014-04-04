package MotorControl;

/**
 * <p>Closed-loop integral wheel velocity controller.</p>
 *
 * @author vona
 * @author prentice
 **/
public class WheelVelocityControllerI extends WheelVelocityController {

  /**
   * <p>The result of the previous control step.</p>
   **/
  protected double lastResult = 0;


  /**
   * {@inheritDoc}
   *
   * <p>This impl implements closed-loop integral control.</p>
   **/
  public double controlStep() {

    double result = 0;

    // Integrate past results with current error to get a more intelligent control signal
    result = lastResult + (this.desiredAngularVelocity - this.currentAngularVelocity)*this.gain;

    if (result > MAX_PWM)
      result = MAX_PWM;

    if (result < -MAX_PWM)
      result = -MAX_PWM;

    // Make sure to store current result with past ones
    lastResult = result;

    return result;
  }

  /**
   * {@inheritDoc}
   *
   * <p>This impl returns "I".</p>
   **/
  public String getName() {
    return "I";
  }
}
