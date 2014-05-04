package collectBlocks;

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


//import cameraProcessing.BlobTracking;

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
	
	//private BlobTracking blobTrack = null; //KN
	
	private final double lengthS = .245;
	private final double lengthW = .19;
		
	private boolean beamBroken = false;
	private boolean visualServoingDone = false; //KN
	
	private static final int width = 160; //KN
	private static final int height = 120; //KN
	
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

	protected void setInitialParams() {

	}
	
	@Override
	public void run() { 
		System.out.println("Initial Position");

		//graspAndTransport();
		//searchAndRetrieve();
	}
	
	public void searchAndRetrieve(){
		//visualServoing();
		graspAndTransport();
	}
	
	public void graspAndTransport() {
		/*String state = "start";
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
		}*/
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
