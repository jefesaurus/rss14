package Grasping;

import java.util.concurrent.ArrayBlockingQueue;

import org.ros.message.MessageListener;
import LocalNavigation.Pose;
import VisualServo.VisionGUI;

import org.ros.message.rss_msgs.ArmMsg;
import org.ros.message.sensor_msgs.Image; //KN
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

public class Grasping implements NodeMain, Runnable {
	
	private Publisher<ArmMsg> armPub;
	private Publisher<Image> vidPub; //KN
	
	private Subscriber<org.ros.message.rss_msgs.ArmMsg> armSub;
	private Subscriber<org.ros.message.rss_msgs.BreakBeamMsg> beamSub;
	private Subscriber<org.ros.message.sensor_msgs.Image> vidSub;
	private SensorAverage beamAvg;
	public Node node;
	
	private VisionGUI gui;
	private ArrayBlockingQueue<byte[]> visionImage = new ArrayBlockingQueue<byte[]>(
			1);
	
	private ServoController shoulderServo;
	private ServoController wristServo;
	private ServoController gripperServo;
	
	private BlobTracking blobTrack = null; //KN
	
	private final double lengthS = .245;
	private final double lengthW = .19;
		
	private boolean beamBroken = false;
	private boolean visualServoingDone = false; //KN
	
	private static final int width = 160; //KN
	private static final int height = 120; //KN
	
	private PoseNavigator nav;
	private Thread navThread;
	
	public Grasping() {
		System.out.println("Initialized Grasping");
		beamAvg = new SensorAverage(5);
		
		//Can change initial configuration if desired		
		double[] angles = inverseKinematics(.3, 0);

		shoulderServo = new ServoController(ServoController.SHOULDER_INDEX, angles[0]);
		wristServo = new ServoController(ServoController.WRIST_INDEX, angles[1]);
		gripperServo = new ServoController(ServoController.GRIPPER_INDEX, Math.PI/4);
		
		setInitialParams();

		gui = new VisionGUI();
	}
	
	ImageProc src = null;

	protected void setInitialParams() {

	}

	/**
	 * <p>Handle a CameraMessage. Perform blob tracking and
	 * servo robot towards target.</p>
	 * 
	 * @param rawImage a received camera message
	 */
	public void handle(byte[] rawImage) {

		boolean success = visionImage.offer(rawImage);
		//System.out.println("vision Image offer success: " + success);
	}

        public double RANGE_GAIN = .03;
        public double BEARING_GAIN = .01;

        public double desiredRange = 54.0;
        public double desiredBearing = 0.0;
	
	@Override
	public void run() { 
		System.out.println("Initial Position");

		//armGymnastics();
		visualServoing();

		graspAndTransport();
		//searchAndRetrieve();
	}
	
	public void searchAndRetrieve(){
		visualServoing();
		graspAndTransport();
	}
	
	public void graspAndTransport() {
		String state = "start";
		int counter = 0;
		while(true) {
			System.out.println("stating machine");
			if (state.equals("start")) {
				state = "waitingForObject";
			} else if (state.equals("waitingForObject")) {
				nav.activelyNavigate = false;
				graspConfiguration();
				// object in breakbeam
				if (beamAvg.average > .8) {
					state = "grasping";
				}
			} else if (state.equals("grasping")) {
				nav.activelyNavigate = false;
				closeGripper();
				state = "lifting";
				counter = 0;
			} else if (state.equals("lifting")) {
				nav.activelyNavigate = false;
				carryConfiguration();
				// wait 3 cycles before changing state
				if (counter > 3) {
					state = "transporting";
					nav.setGoal(new Pose(0.5,0.5,Math.PI/2.));
					counter = 0;
				} else {
					if (beamAvg.average < .8) {
						state = "waitingForObject";
					} else {
						counter++;
					}
				}
			} else if (state.equals("transporting")) {
				nav.activelyNavigate = true;
				Pose pose = nav.getPose();
				Pose goal = nav.getGoal();
				double angle_error = PoseNavigator.angleError(Math.atan2(goal.y - pose.y, goal.x - pose.x), pose.theta, Math.PI*2);
				System.out.println(String.format("robotx: %.2f roboty: %.2f robott: %.2f", pose.x, pose.y, pose.theta));
				System.out.println(String.format("goalx: %.2f goaly: %.2f goalt: %.2f", goal.x, goal.y, goal.theta));
				System.out.println(String.format("angleError: %.2f", angle_error));
				if (nav.reachedGoal()) {
					state = "depositing";
					counter = 0;
				}
			} else if (state.equals("depositing")) {
				nav.activelyNavigate = false;
				nav.setGoal(nav.getPose()); // so that it really thinks it reached the goal.
				depositConfiguration();
				if (counter > 3) {
					state = "backingUp";
					counter = 0;
				} else {
					counter++;
				}
			} else if (state.equals("lettingGo")) {
				openGripper();
				if (counter > 3) {
					state = "backingUp";
					counter = 0;
				} else {
					counter++;
				}
			} else if (state.equals("backingUp")) {
				nav.activelyNavigate = false;
				nav.setVelocities(-.3, 0);
				openGripper();
				Pose pose = nav.getPose();
				if (counter > 3) {
					state = "goBack";
					nav.setGoal(new Pose(0,0,0));
					counter = 0;
				} else {
					counter++;
				}
			} else if (state.equals("goBack")) {
				nav.activelyNavigate = true;
				nav.setGoal(new Pose(0,0,0));
			} else {
				System.out.println("Unknown state reached");
			}
			System.out.println("State is" + state);
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}
	
	public void armGymnastics() {
		while(true) {
			try {
				Thread.sleep(5000);
			} catch (InterruptedException e) {
			}
			
			System.out.println("Grasping");
			graspConfiguration();
	
			try {
				Thread.sleep(5000);
			} catch (InterruptedException e) {
			}
			
			System.out.println("Grasped");
			closeGripper();
			
			try {
				Thread.sleep(5000);
			} catch (InterruptedException e) {
			}
			
			System.out.println("Picking");
			carryConfiguration();
		}
	}
	
	public void visualServoing() {
		while(visualServoingDone == false) {
			System.out.println("##########################Visual Servoing");

			graspConfiguration();
			try {
				src = new ImageProc(visionImage.take(), width, height);
			} catch (InterruptedException e) {
				e.printStackTrace();
				continue;
			}

			ImageProc dest = new ImageProc(src);
			//blobTrack.apply(src, dest);
			Histogram.getHistogram(src, dest, false);
			

			// update newly formed vision message
			gui.setVisionImage(dest.toArray(), width, height);
			/*

			if (blobTrack.targetDetected) {
				blobTrack.blobFix();
				nav.activelyNavigate = false;
				//double blobx = blobTrack.targetRange*Math.cos(blobTrack.targetBearing);
				//double bloby = blobTrack.targetRange*Math.sin(blobTrack.targetBearing);
				double tv = RANGE_GAIN*(desiredRange-blobTrack.targetRange);
				double rv = BEARING_GAIN*(desiredBearing-blobTrack.targetBearing);
				System.out.println(String.format("Current Range: %.2f | Desired Range: %.2f",blobTrack.targetRange,desiredRange));
				System.out.println(String.format("Current Bearing: %.2f | Desired Bearing: %.2f",blobTrack.targetBearing,desiredBearing));
				System.out.println(String.format("Centroid X: %.2f | Centroid Y: %.2f | tv: %.2f | rv: %.2f | beamAvg: %.2f", blobTrack.targetCentroidX, blobTrack.targetCentroidY, tv, rv, beamAvg.average));
				
				nav.setVelocities(tv,rv);
				if (beamAvg.average > .8) {
					visualServoingDone = true;
				}
			}
			*/
			
			Image pubImage = new Image();
			pubImage.width = width;
			pubImage.height = height;
			pubImage.encoding = "rgb8";
			pubImage.is_bigendian = 0;
			pubImage.step = width*3;
			pubImage.data = dest.toArray();
			vidPub.publish(pubImage);
		}
	}
	
	public void moveToConfiguration(double x, double y) {
		double[] angles = inverseKinematics(x, y);
		shoulderServo.moveTo(angles[0]);
		wristServo.moveTo(angles[1]);
	}
	
	public void moveToConfiguration(double x, double y, double grasp) {
		double[] angles = inverseKinematics(x, y);
		shoulderServo.moveTo(angles[0]);
		wristServo.moveTo(angles[1]);
		gripperServo.moveTo(grasp);		
	}
	
	public void graspConfiguration() {
		double x = .25;
		double y = -.27;
		
		double[] angles = inverseKinematics(x, y);
		shoulderServo.moveTo(angles[0]);
		wristServo.moveTo(angles[1]);
		openGripper();		
	}
	
	public void depositConfiguration() {
		double x = .25;
		double y = -.25;
		
		double[] angles = inverseKinematics(x, y);
		shoulderServo.moveTo(angles[0]);
		wristServo.moveTo(angles[1]);
		closeGripper();	
	}
	
	public void carryConfiguration() {
		double x = -.18;
		double y = .11;
		
		double[] angles = inverseKinematics(x, y);
		shoulderServo.moveTo(angles[0]);
		wristServo.moveTo(angles[1]);
		closeGripper();
	}
	
	public void openGripper() {
		double grasp = Math.PI/6;
		gripperServo.moveTo(grasp);		
	}
	
	public void closeGripper() {
		double grasp = -Math.PI/10;
		gripperServo.moveTo(grasp);		
	}
	
	//TODO - obtain other inverse kinematics solution
	public double[] inverseKinematics(double x, double y) {
		double[] angles = new double[2];
		angles[0] = Math.PI/2 - (Math.atan2(y, x) - Math.acos((lengthS*lengthS + x*x + y*y - lengthW*lengthW)/(2*lengthS*Math.sqrt(x*x + y*y))));
		angles[1] = Math.PI/2 - (Math.PI - Math.acos((lengthS*lengthS - x*x - y*y + lengthW*lengthW)/(2*lengthS*lengthW)));
		return angles;
	}
	
	public void handle(ArmMsg msg) {
		msg.pwms = new long[]{shoulderServo.nextPWM(), wristServo.nextPWM(), gripperServo.nextPWM(), 0, 0, 0};		
		armPub.publish(msg);
	}
	
	@Override
	public void onStart(Node node) {
		this.node = node;
		System.out.println("Started Grasping");
				
		armPub = node.newPublisher("command/Arm", "rss_msgs/ArmMsg");
		vidPub = node.newPublisher("/rss/blobVideo", "sensor_msgs/Image"); //KN
		
		armSub = node.newSubscriber("/rss/ArmStatus", "rss_msgs/ArmMsg");
		armSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.ArmMsg>() {
			@Override
			public synchronized void onNewMessage(
					org.ros.message.rss_msgs.ArmMsg message) {
				//System.out.println("New Arm Message: " + message.pwms[0] + ", " + message.pwms[1] + ", " + message.pwms[2]);
				handle(message);
			}
			
		});
		
		beamSub = node.newSubscriber("/rss/BreakBeam", "rss_msgs/BreakBeamMsg");
		beamSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.BreakBeamMsg>() {
			@Override
			public synchronized void onNewMessage(
					org.ros.message.rss_msgs.BreakBeamMsg message) {
				beamBroken = message.beamBroken;
				beamAvg.step(beamBroken ? 1. : 0.);
				//System.out.println("Beam average: " + beamAvg.average);
			}
		});
		
		final boolean reverseRGB = node.newParameterTree().getBoolean("reverse_rgb", false);
		vidSub = node.newSubscriber("/rss/video", "sensor_msgs/Image");
		vidSub.addMessageListener(new MessageListener<org.ros.message.sensor_msgs.Image>() {
			@Override
			public void onNewMessage(
					org.ros.message.sensor_msgs.Image message) {
				byte[] rgbData;
				//if (reverseRGB) {
					rgbData = ImageProc.RGB2BGR(message.data, (int) message.width, (int) message.height);
				//}
				//else {
				//	rgbData = message.data;
				//}
				assert ((int) message.width == width);
				assert ((int) message.height == height);
				//System.out.println("rgbData length: " + rgbData.length);
				handle(rgbData);
			}
		});
		
		nav = new PoseNavigator(this.node);
		blobTrack = new BlobTracking(width, height); //KN
		navThread = new Thread(nav);
		navThread.start();
		new Thread(this).start();
	}
	
	@Override
	public void onShutdown(Node node) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onShutdownComplete(Node node) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public GraphName getDefaultNodeName() {
		// TODO Auto-generated method stub
		return null;
	}

}
