package master;

public class ServoController {

	private class ServoInfo {
		public long closePWM, openPWM;
		
		public ServoInfo(long closePWM, long openPWM) {
			this.closePWM = closePWM;
			this.openPWM = openPWM;
		}
	}
	
	public static final int BLUE_GATE_INDEX = 0;
	public static final int FRONT_GATE_INDEX = 1;
	
	private ServoInfo blueGate = new ServoInfo(1871, 1387);
	private ServoInfo frontGate = new ServoInfo(1400, 580);
	
	private static final double MAX_PWM_CHANGE = 50;
	private final long closePWM, openPWM;
	
	private long targetPWM;
	private long currentPWM;
	private long maxPWM;
	private long minPWM;
	
	public ServoController(int servoIndex, double initialAngle) {
		ServoInfo info = blueGate;
		switch (servoIndex) {
			case 0: info = blueGate;
					break;
			case 1: info = frontGate;
					break;
			default:
					System.out.println("Invalid servo index choice");
		}
		this.closePWM = info.closePWM;
		this.openPWM = info.openPWM;
		this.maxPWM = Math.max(this.closePWM, this.openPWM);
		this.minPWM = Math.min(this.closePWM, this.openPWM);
	}
	
	public void openGate() {
		this.targetPWM = this.openPWM;
	}
	
	public void closeGate() {
		this.targetPWM = this.closePWM;
	}
	
	private long clampPWM(long PWM) {
		return Math.min(Math.max(PWM, minPWM), maxPWM);
	}
	
	public long nextPWM() {
		long diff = targetPWM - currentPWM;
		long nextPWM = targetPWM;
		if (Math.abs(diff) > MAX_PWM_CHANGE) {
			nextPWM = (long) (currentPWM + Math.signum(diff)*MAX_PWM_CHANGE);
		}
		currentPWM = nextPWM;
		return clampPWM(nextPWM);
	}
}
