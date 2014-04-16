package Grasping;

import java.awt.geom.Point2D;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.MotionMsg;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import LocalNavigation.Pose;

public class PoseNavigator implements Runnable {
	public Subscriber<org.ros.message.rss_msgs.OdometryMsg> odoSub;
	public Publisher<MotionMsg> motionPub;
	
	protected Pose pose;
	private Pose goal;
	public Node node;
	double angle_error = 0.0;
	double translate_error = 0.0;
	boolean firstUpdate = true;
	public boolean activelyNavigate = true;
	
	public PoseNavigator(Node node) {
		this.node = node;
		this.pose = new Pose(0, 0, 0);
		this.goal = new Pose(0, 0, 0);
		setupPubSub();
	}
	
	
	
	private void setupPubSub() {
		this.odoSub = node.newSubscriber("/rss/odometry", "rss_msgs/OdometryMsg");
		odoSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.OdometryMsg>() {
			@Override
			public synchronized void onNewMessage(
					org.ros.message.rss_msgs.OdometryMsg message) {
				pose = new Pose(message.x, message.y, message.theta);
				if (firstUpdate) {
					goal = new Pose(message.x, message.y, message.theta);
					firstUpdate = false;
				}
			}
		});
		
		this.motionPub = node.newPublisher("command/Motors", "rss_msgs/MotionMsg");
	}
	
	public synchronized Pose getPose() {
		return pose;
	}
	
	public Pose getGoal() {
		return goal;
	}
	
	public void setGoal(Pose g) {
		System.out.println("setting goal");
		this.goal = g;
	}
	
	public static double angleError(double actualAng, double desiredAng, double modulo) {
		double diff = actualAng - desiredAng;
		diff = diff - (int)(diff/modulo)*modulo;
		if (diff > modulo/2)
			return modulo - diff;
		else if (diff < -modulo/2)
			return modulo + diff;
		else
			return diff;
	}
	
	private double euclideanDist(Pose p1, Pose p2) {
		return Math.sqrt((p1.y-p2.y)*(p1.y-p2.y) + (p1.x-p2.x)*(p1.x-p2.x));
	}
	
	public void setVelocities(double tv, double rv) {
		MotionMsg msg = new MotionMsg();
		msg.translationalVelocity = tv;
		msg.rotationalVelocity = rv;
		//motionPub.publish(msg);
		//System.out.println(String.format("tv: %.2f rv: %.2f", tv, rv));
	}
	
	double TRANSLATION_FUZZ = .12;
	double ROTATION_FUZZ = .5;
	public boolean reachedGoal() {
		angle_error = angleError(Math.atan2(goal.y - pose.y, goal.x - pose.x), pose.theta, Math.PI*2);
		translate_error = euclideanDist(pose, goal);

		if (angle_error < ROTATION_FUZZ && translate_error < TRANSLATION_FUZZ) {
			return true;
		} else {
			return false;
		}
	}

	@Override
	public void run() {
		double k_spin = .1;
		double k_follow = .2;
		double k_translate = .5;
		double MAX_V = .25;
		double MIN_V = .04;
		double tv = 0.;
		double rv = 0.;
		while (true) {
			if (activelyNavigate)
			angle_error = angleError(Math.atan2(goal.y - pose.y, goal.x - pose.x), pose.theta, Math.PI*2);
			translate_error = euclideanDist(pose, goal);
			if (Math.abs(angle_error) > ROTATION_FUZZ) {
				tv = 0.0;
				rv = k_spin*(angle_error);
			} else if (translate_error > TRANSLATION_FUZZ) {
				tv = Math.min(translate_error * k_translate, MAX_V);
				tv = Math.max(tv, MIN_V);
				rv = k_follow*(angle_error);
			} else {
				tv = 0.;
				rv = 0.;
			}
			setVelocities(tv, rv);
		}
	}
}
