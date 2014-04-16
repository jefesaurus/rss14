package navigation;

import gui.*;
import java.util.LinkedList;
import java.util.List;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.MotionMsg;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

public class WaypointNavigator implements Runnable {
	public Node node;
	public Subscriber<org.ros.message.rss_msgs.OdometryMsg> odoSub;
	public Publisher<MotionMsg> motionPub;
	
	public NavigationGUI gui;
	protected boolean firstUpdate;
	protected Configuration configuration;
	protected List<Configuration> path;

	public WaypointNavigator(Node node, NavigationGUI gui) {
		this.node = node;
		this.gui = gui;
		this.firstUpdate = true;
		//this.configuration = null;
		this.configuration = new Configuration(0,0,0);
		path = new LinkedList<Configuration>();
		setup();
	}
	
	private void setup() {
		this.odoSub = node.newSubscriber("/rss/odometry", "rss_msgs/OdometryMsg");
		odoSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.OdometryMsg>() {
			@Override
			public synchronized void onNewMessage(
					org.ros.message.rss_msgs.OdometryMsg message) {
				if (firstUpdate) {
					firstUpdate = false;
					gui.resetWorldToView(message.x, message.y);
				}
				gui.setRobotPose(message.x, message.y, message.theta);
				configuration = new Configuration(message.x, message.y, message.theta);
			}
		});
		this.motionPub = node.newPublisher("command/Motors", "rss_msgs/MotionMsg");
		new Thread(this).start();
	}
	
	public synchronized Configuration getConfiguration() {
		return configuration;
	}
	
	public synchronized void setPath(List<Configuration> path) {
		this.path = path;
	}
	
	public synchronized Configuration nextConfig() {
		if (path.size() > 0) {
			return path.remove(0);
		} else {
			return null;
		}
	}

	double TRANSLATION_FUZZ = .12;
	double ROTATION_FUZZ = .1;
	double k_spin = .1;
	double k_follow = .2;
	double k_translate = .5;
	double MAX_V = .25;
	double MIN_V = .04;
	@Override
	public void run() {
		System.out.println("Started WaypointNavigator Loop");
		double tv, rv;
		while (true) {
			//if (firstUpdate) {
			//	continue;
			//}
			Configuration waypoint = nextConfig();
			if (waypoint == null) {
				continue;
			}
			System.out.println("New Waypoint");
			while (true) {
				double angle_error = Util.angleDistance(configuration.theta, waypoint.theta);
				double translate_error = Util.vectorLength(waypoint.x - configuration.x, waypoint.y - configuration.y);

				tv = 0.1;
				rv = 0.;
				
				/*if (angle_error > ROTATION_FUZZ) {
					tv = 0.0;
					rv = k_spin*(angle_error);
				} else if (translate_error > TRANSLATION_FUZZ) {
					tv = Math.min(translate_error * k_translate, MAX_V);
					tv = Math.max(tv, MIN_V);
					rv = k_follow*(angle_error);
				} else {
					tv = 0.1;
					rv = 0.;
					break;
				}*/

				MotionMsg msg = new MotionMsg();
				msg.translationalVelocity = tv;
				msg.rotationalVelocity = rv;
                System.out.println("TV: " + tv + ", RV: " + rv);
				motionPub.publish(msg);
			}
		}
	}
}
