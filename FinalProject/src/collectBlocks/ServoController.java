package collectBlocks;

public class ServoController {

	private class ServoInfo {
		public long minPWM, maxPWM, zeroPWM, halfPIPWM;
		
		public ServoInfo(long minPWM, long maxPWM, long zeroPWM, long halfPIPWM) {
			this.minPWM = minPWM;
			this.maxPWM = maxPWM;
			this.zeroPWM = zeroPWM;
			this.halfPIPWM = halfPIPWM;
		}
	}
	
	public static final int SHOULDER_INDEX = 0;
	public static final int WRIST_INDEX = 1;
	public static final int GRIPPER_INDEX = 2;
	
	private ServoInfo shoulder = new ServoInfo(645, 2290, 2097, 1452);
	private ServoInfo wrist = new ServoInfo(677, 2403, 984, 1839);
	private ServoInfo gripper = new ServoInfo(1516, 2113, 1548, 2965);
	
	private static final double MAX_ANGLE_CHANGE = 1;
	private final long minPWM, maxPWM, zeroPWM, halfPIPWM;
	
	private double targetAngle; // in radians
	private double currentAngle; // in radians
	
	public ServoController(int servoIndex, double initialAngle) {
		ServoInfo info = shoulder;
		switch (servoIndex) {
			case 0: info = shoulder;
					break;
			case 1: info = wrist;
					break;
			case 2: info = gripper;
					break;
			default:
					System.out.println("Invalid servo index choice");
		}
		this.minPWM = info.minPWM;
		this.maxPWM = info.maxPWM;
		this.zeroPWM = info.zeroPWM;
		this.halfPIPWM = info.halfPIPWM;
		
		this.currentAngle = initialAngle;
		this.targetAngle = initialAngle;
	}
	
	public void moveTo(double targetAngle) {
		this.targetAngle = targetAngle;
	}
	
	private long clampPWM(long PWM) {
		return Math.min(Math.max(PWM, minPWM), maxPWM);
	}
	
	public long nextPWM() {
		double diff = targetAngle - currentAngle;
		double nextAngle = targetAngle;
		if (Math.abs(diff) > MAX_ANGLE_CHANGE) {
			nextAngle = currentAngle + Math.signum(diff)*MAX_ANGLE_CHANGE;
		}
		long nextPWM = (long)(zeroPWM + (halfPIPWM - zeroPWM)/(Math.PI/2 - 0)*(nextAngle - 0));
		currentAngle = nextAngle;
		return clampPWM(nextPWM);
	}
}
