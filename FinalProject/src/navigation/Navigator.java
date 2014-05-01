package navigation;

import gui.*;

import java.awt.Color;
import java.util.LinkedList;
import java.util.List;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.MotionMsg;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

public class Navigator implements Runnable {
	public Node node;
	public Subscriber<org.ros.message.rss_msgs.OdometryMsg> odoSub;
	public Publisher<MotionMsg> motionPub;
	
	public NavigationGUI gui;
	public World world;
	public MotionPlanner planner;
	protected boolean firstUpdate;
	protected Configuration reference, current, goal;
	protected List<Configuration> path;

	public Navigator(Node node, NavigationGUI gui, World world) {
		this.node = node;
		this.gui = gui;
		this.world = world;
		this.planner = new MotionPlanner(world);
		
		this.firstUpdate = true;
		this.reference = null;
		this.current = null;
		this.goal = null;
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
					gui.clear();
					//gui.resetWorldToView(message.x, message.y);
					gui.draw();
					firstUpdate = false;
					reference = new Configuration(message.x, message.y, message.theta);
				}				
				double length = Util.vectorLength(message.x - reference.x, message.y - reference.y);
				double angle = Util.vectorAngle(message.x - reference.x, message.y - reference.y) - reference.theta;
				double x = length*Math.cos(angle);
				double y = length*Math.sin(angle);
				
				Configuration configuration = new Configuration(message.x, message.y, message.theta).compose(reference);	
				//TODO add the the start angle
				//configuration = new Configuration(configuration.x + world.getStart().x, configuration.y + world.getStart().y, configuration.theta); //Only correct for theta = 0

				//Configuration configuration = new Configuration(message.x, message.y, message.theta).compose(reference).compose(start);	
				gui.setRobotPose(configuration.x, configuration.y, configuration.theta);
				setConfiguration(configuration);
			}
		});
		this.motionPub = node.newPublisher("command/Motors", "rss_msgs/MotionMsg");
		
		long startTime = System.currentTimeMillis();	
		while (firstUpdate) {
			Util.pause(1); //TODO remove and fix concurrency bug
			if ((System.currentTimeMillis() - startTime) % 2000 == 0) {
				System.out.println("Waiting for First Odometry Message");
			}
		}
		
		new Thread(this).start();
	}
	
	public synchronized Configuration getGoal() {
		return goal;
	}
	
	public synchronized void setGoal(Configuration g) {
		goal = g;
		if (goal != null) {
			if (!replan()){ //Could not find path. Do not set it as the goal
				goal = null; //TODO instead just maintain past goal?
				setPath(new LinkedList<Configuration>());
			} else {
				System.out.println("New Goal: " + goal);
			}
		}
	}
	
	public synchronized Configuration getConfiguration() {
		return current;
	}
	
	public synchronized void setConfiguration(Configuration c) {
		current = c;
	}
	
	public boolean replan() { //Grow RRT on grid + obstacles then just obstacles if fails
		Configuration start = getConfiguration();
		//TODO getGoal()?
		List<Configuration> path = planner.findPath(start, goal); 
		
		if (path != null) { //TODO RRT fails to find path failure modes
			System.out.println("Found path of length " + path.size() + " from " + start + " to " + goal);
			setPath(path);
			
			gui.clear();
			gui.draw();
			gui.draw(path);
			gui.draw(planner.tree1.root, Color.BLUE);
			gui.draw(planner.tree2.root, Color.RED);
			
			return true;
		} else {
			System.out.println("Could not find path from " + start + " to " + goal);
			return false;
		}
	}
	
	public synchronized void setPath(List<Configuration> path) { //TODO Replan
		this.path = path;
	}
	
	public synchronized Configuration nextConfig() {
		if (path.size() > 0) {
			return path.remove(0);
		} else {
			return null;
		}
	}

	//TOOD - remove unused launch components
	//TODO - remove netbook window of robot spaz
		
	public void sendVelocityMessage(double tv, double rv) {
		MotionMsg msg = new MotionMsg();
		msg.translationalVelocity = tv;
		msg.rotationalVelocity = rv;
		motionPub.publish(msg);
	}
	
	@Override
	public void run() {
		double tv, rv;
		
		System.out.println("Started Navigator Loop");
		long startTime = System.currentTimeMillis();
		while (true) {
			Util.pause(1); //TODO remove and fix concurrency bug
			Configuration waypoint = nextConfig();
			if (waypoint == null) {
				sendVelocityMessage(0., 0.);
				continue;
			}
			System.out.println("New Waypoint: " + waypoint);
			while (true) {			
				Util.pause(1); //TODO remove and fix concurrency bug
				
				Configuration configuration = getConfiguration();
				double directionTheta =  Util.vectorAngle(waypoint.x - configuration.x ,waypoint.y - configuration.y);
				
				double directionError = Util.angleDistance(configuration.theta, directionTheta);
				double translationError = Util.vectorLength(waypoint.x - configuration.x, waypoint.y - configuration.y);
				double angleError = Util.angleDistance(configuration.theta, Util.cleanAngle(waypoint.theta));				
								
				tv = 0.;
				rv = 0.;				
				if (translationError > Constants.TRANSLATION_THRESHOLD) {
					if (directionError > Constants.ROTATION_THRESHOLD) { 
						double rotationSpeed = Math.max(Math.min(Constants.K_ROTATE*directionError, Constants.MAX_RV), Constants.MIN_RV);
						tv = 0.0;
						rv = rotationSpeed*Util.angleDistanceSign(configuration.theta, directionTheta);
					} 
					else {
						double translationSpeed = Math.max(Math.min(Constants.K_TRANSLATE*translationError, Constants.MAX_TV), Constants.MIN_TV); //Only moves forward
						tv = translationSpeed;
						rv = 0.0;
					}
				} else if (angleError > Constants.ROTATION_THRESHOLD) { 
					double rotationSpeed = Math.max(Math.min(Constants.K_ROTATE*angleError, Constants.MAX_RV), Constants.MIN_RV);
					tv = 0.0;
					rv = rotationSpeed*Util.angleDistanceSign(configuration.theta, Util.cleanAngle(waypoint.theta));
				} else {
					//Reached waypoint
					sendVelocityMessage(0., 0.);
					if (path.size() == 0) {
						//Reached goal
						setGoal(null);
					}
					break;
				}
				
				if ((System.currentTimeMillis() - startTime) % 2000 == 0) {
					System.out.println("Time: " + (System.currentTimeMillis() - startTime)/1000 + " seconds");
					System.out.println("Current: " + configuration);
					System.out.println("Waypoint: " + waypoint);
					System.out.println("Error: " + directionError + ", " + translationError + ", " + angleError);
					System.out.println("Velocity: " + tv + ", " + rv + "\n");
				}
				sendVelocityMessage(tv, rv);
			}
		}
	}
}
