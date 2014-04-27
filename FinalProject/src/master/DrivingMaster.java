package master;

import java.awt.geom.Point2D;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.MotionMsg;
import org.ros.message.rss_msgs.OdometryMsg;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.*;

public class DrivingMaster implements NodeMain, Runnable {

	public Subscriber<OdometryMsg> odoSub;
	public Publisher<MotionMsg> motionPub;
	public boolean goalOriented;
	private Pose robotPose;
	private Pose goalPose;
	
	private double tv, rv;
	public double ROTATION_FUZZ = .1;
	public double TRANSLATION_FUZZ = .12;
	private double k_spin = .1;
	private double k_follow = .2;
	private double k_translate = .5;
	private double MAX_V = .25;
	private double MIN_V = .04;
	
	protected boolean firstUpdate = true;
	
	public DrivingMaster() {
		goalOriented = false;
		robotPose = new Pose(0,0,0);
		goalPose = new Pose(0,0,0);
		tv = rv = 0.;
	}
	
	/**
	 * set goal point (where final theta doesn't matter)
	 * @param point
	 */
	public void setGoalPoint(Point2D point) {
		setDrivingIsGoalOriented(true);
		goalPose.x = point.getX();
		goalPose.y = point.getY();
		goalPose.theta = Double.NaN;
	}
	
	/**
	 * Set goal pose (where final theta actuall matters)
	 * @param pose
	 */
	public void setGoalPose(Pose pose) {
		setDrivingIsGoalOriented(true);
		goalPose = pose;
	}
	
	public Pose getGoalPose() {
		return goalPose;
	}
	
	public Pose getRobotPose() {
		return robotPose;
	}
	
	/**
	 * Set whether the driving module should simply listen for
	 * tv, rv messages or should actively drive to a goal point
	 * that has been set
	 * @param toggle
	 */
	public void setDrivingIsGoalOriented(boolean toggle) {
		goalOriented = toggle;
	}
	
	public boolean isDrivingGoalOriented() {
		return goalOriented;
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
	
	public void setVelocity(double tv, double rv) {
		setDrivingIsGoalOriented(false);
		this.tv = tv;
		this.rv = rv;
	}
	
	public boolean goalReached(double epsilon) {
		return robotPose.distance(goalPose) < epsilon;
	}
	
	@Override
	public void run() {
		while (true) {
			double tv, rv;
			if (goalOriented) {
				// try to get to goal
				double angle_error = angleError(goalPose.theta, robotPose.theta, Math.PI*2);
				double translate_error = robotPose.distance(goalPose);
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
			} else {
				// simply pass on the rv, tv commands
				tv = this.tv;
				rv = this.rv;
			}
			MotionMsg msg = new MotionMsg();
			msg.translationalVelocity = tv;
			msg.rotationalVelocity = rv;
			motionPub.publish(msg);
		}
	}

	@Override
	public void onStart(Node node) {
		this.odoSub = node.newSubscriber("/rss/odometry", "rss_msgs/OdometryMsg");
		odoSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.OdometryMsg>() {
			@Override
			public synchronized void onNewMessage(OdometryMsg message) {
				if (firstUpdate) {
					firstUpdate = false;
				}
				robotPose = new Pose(message.x, message.y, message.theta);
			}
		});
		this.motionPub = node.newPublisher("command/Motors", "rss_msgs/Motionmsg");
	}
	
	@Override
	public void onShutdown(Node arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onShutdownComplete(Node arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public GraphName getDefaultNodeName() {
		// TODO Auto-generated method stub
		return null;
	}

}
