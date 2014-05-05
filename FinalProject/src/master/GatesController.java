package master;

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

public class GatesController implements NodeMain {
	
	private Publisher<ArmMsg> armPub;
	
	private Subscriber<org.ros.message.rss_msgs.ArmMsg> armSub;
	public Node node;
	
	private ServoController blueGate;
	private ServoController frontGate;
		
	public GatesController() {
		blueGate = new ServoController(ServoController.BLUE_GATE_INDEX, Math.PI/4);
		frontGate = new ServoController(ServoController.FRONT_GATE_INDEX, 0);
	}
	
	public void handle(ArmMsg msg) {
		msg.pwms = new long[]{blueGate.nextPWM(), 0, frontGate.nextPWM(), 0, 0, 0};		
		armPub.publish(msg);
	}
	
	public void closeBlueGate() {
		blueGate.closeGate();
	}
	
	public void openBlueGate() {
		blueGate.openGate();
	}
	
	public void closeFrontGate() {
		frontGate.closeGate();
	}
	
	public void openFrontGate() {
		frontGate.openGate();
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
