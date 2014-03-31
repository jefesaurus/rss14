package GlobalNavigation;

import java.awt.geom.Point2D;
import java.util.List;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.MotionMsg;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import LocalNavigation.Pose;

public class WaypointNavigator implements Runnable {
	public Subscriber<org.ros.message.rss_msgs.OdometryMsg> odoSub;
	public Publisher<MotionMsg> motionPub;
	
	List<Point2D.Double> path;
	
	protected boolean firstUpdate;
	protected Pose pose;
	public MapGUI gui;
	public Node node;
	
	int finishedTo;
	double angle_error = 0.0;
	double translate_error = 0.0;
	
	public WaypointNavigator(Node node, List<Point2D.Double> path, MapGUI gui) {
		this.gui = gui;
		this.node = node;
		this.path = path;
		this.finishedTo = 0;
		this.pose = new Pose(-1, -1, -1);
		setupPubSub();
	}
	
	private void setupPubSub() {
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
				pose = new Pose(message.x, message.y, message.theta);
			}
		});
		
		this.motionPub = node.newPublisher("command/Motors", "rss_msgs/MotionMsg");
	}
	

	
	public synchronized Pose getPose() {
		return pose;
	}
	
	private double angleError(double actualAng, double desiredAng, double modulo) {
		double diff = actualAng - desiredAng;
		diff = diff - (int)(diff/modulo)*modulo;
		if (diff > modulo/2)
			return modulo - diff;
		else if (diff < -modulo/2)
			return modulo + diff;
		else
			return diff;
	}

	double TRANSLATION_FUZZ = .12;
	double ROTATION_FUZZ = .1;
	@Override
	public void run() {
		System.out.println("Starting Run");
		double k_spin = .1;
		double k_follow = .2;
		double k_translate = .5;
		double MAX_V = .25;
		double MIN_V = .04;
		double tv = 0.;
		double rv = 0.;
		for ( int i = 1; i < path.size(); i ++) {
			Point2D.Double waypoint = path.get(i);
			while (true) {
				angle_error = angleError(Math.atan2(waypoint.y - pose.y, waypoint.x - pose.x), pose.theta, Math.PI*2);
				translate_error = waypoint.distance(new Point2D.Double(pose.x, pose.y));

				if (angle_error > ROTATION_FUZZ) {
					tv = 0.0;
					rv = k_spin*(angle_error);
				} else if (translate_error > TRANSLATION_FUZZ) {
					tv = Math.min(translate_error * k_translate, MAX_V);
					tv = Math.max(tv, MIN_V);
					rv = k_follow*(angle_error);
				} else {
					tv = 0.;
					rv = 0.;
					break;
				}

				MotionMsg msg = new MotionMsg();
				msg.translationalVelocity = tv;
				msg.rotationalVelocity = rv;
				motionPub.publish(msg);
			}
		}
		
		tv = 0.;
		rv = 0.;	
		MotionMsg msg = new MotionMsg();
		msg.translationalVelocity = tv;
		msg.rotationalVelocity = rv;
		motionPub.publish(msg);
		System.out.println("Finished Run");
	}
}
