package navigation;

import gui.*;

import java.awt.Color;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

import navigation.Constants.CollisionCheck;
import navigation.Constants.DriveSystem;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.MotionMsg;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

public class Navigator {
	public Node node;
	public Subscriber<org.ros.message.rss_msgs.BumpMsg> bumperSub;
	public Subscriber<org.ros.message.rss_msgs.BumpMsg> backBumperSub;
	public Subscriber<org.ros.message.rss_msgs.OdometryMsg> odoSub;
	public Publisher<MotionMsg> motionPub;
	public Subscriber<org.ros.message.sensor_msgs.PointCloud2> kinSub;
	
	public NavigationGUI gui;
	public World world;
	public MotionPlanner planner;
	protected boolean firstUpdate, frozen, pathReset, immediateLock, replanNeeded;
	protected Configuration reference, initial, current;
	
	protected List<Goal> goals;
	protected List<Waypoint> path;
	protected List<Goal> accomplished;
	
	public Navigator(Node node, NavigationGUI gui, World world) {
		this.node = node;
		this.gui = gui;
		this.world = world;
		this.world.navigator = this;
		this.planner = new MotionPlanner(world);
		
		this.firstUpdate = true;
		this.frozen = false;
		this.pathReset = false;
		this.immediateLock = false;
		this.replanNeeded = false;
		
		this.reference = null;
		//this.initial = world.getStart().configuration(0.); //Alec said that we can assume the robot starts facing east (angle 0) //TODO
		this.initial = world.getStart().configuration(Math.PI); //Alec said that we can assume the robot starts facing east (angle 0)
		this.current = null;
		
		this.goals = Collections.synchronizedList(new LinkedList<Goal>());		
		this.path = new LinkedList<Waypoint>();
		this.accomplished = new LinkedList<Goal>();
		
		setup();
	}
	
	private void setup() {
		this.bumperSub = node.newSubscriber("/rss/BumpSensors", "rss_msgs/BumpMsg");
		bumperSub
				.addMessageListener(new MessageListener<org.ros.message.rss_msgs.BumpMsg>() {
					@Override
					public void onNewMessage(
							org.ros.message.rss_msgs.BumpMsg message) {
						boolean left = message.left;
						boolean right = message.right;
						if ((left || right) && !immediateLock) { //TODO - bumping when not in control
							immediateLock = true;
							if (left) {
								System.out.println("Front Left Bump Sensor Collision! Backing up\n");
							} else {
								System.out.println("Front Right Bump Sensor Collision! Backing up\n");
							}
							
							Configuration start = getConfiguration();
							Waypoint waypoint = planner.safeBackward(start);
							goals.add(0, waypoint.config);
							setPath(new LinkedList<Waypoint>(Arrays.asList(waypoint)));
							draw();
						}
					}
				});
		
		this.backBumperSub = node.newSubscriber("/rss/BackBumpSensors", "rss_msgs/BumpMsg");
		backBumperSub
				.addMessageListener(new MessageListener<org.ros.message.rss_msgs.BumpMsg>() {
					@Override
					public void onNewMessage(
							org.ros.message.rss_msgs.BumpMsg message) {
						boolean left = message.left;
						boolean right = message.right;
						if ((left || right) && !immediateLock) { //TODO - bumping when not in control
							immediateLock = true;
							if (left) {
								System.out.println("Back Left Bump Sensor Collision! Backing up\n");
							} else {
								System.out.println("Back Right Bump Sensor Collision! Backing up\n");
							}
							
							Configuration start = getConfiguration();
							Waypoint waypoint = planner.safeForward(start);
							goals.add(0, waypoint.config);
							setPath(new LinkedList<Waypoint>(Arrays.asList(waypoint)));
							draw(); 
						}
					}
				});
		
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
				Configuration configuration = new Configuration(message.x, message.y, message.theta).inverseTransform(reference).forwardTransform(initial);	
				//gui.setRobotPose(configuration.x, configuration.y, configuration.theta);
				setConfiguration(configuration);
				
				//draw();
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
				
		(new Thread() {
			  public void run() {
				  replanner();
			  }
			 }).start();
		
		(new Thread() {
			  public void run() {
				  motorController();
			  }
			 }).start();
	}
	
	public void resetOdometry(Configuration correct) {
		this.reference = getConfiguration();
		this.initial = correct;
	}
	
	public void processGoals() {
		while (goals.size() != 0) {
			if (!replan()){ //Could not find path. Do not set it as the goal
				System.out.println("Unreachable Goal: " + goals.get(0) + "\n");
				goals.remove(0);
				immediateLock = false;
			} else {
				System.out.println("New Goal: " + goals.get(0) + "\n");
				break;
			}
		}	
		
		if (goals.size() == 0) {
			setPath(new LinkedList<Waypoint>());
		}
		draw();
	}

	public synchronized void immediateGoal(Goal g) {
		if (!immediateLock) {
			immediateLock = true;
			goals.add(0, g);
			processGoals();
		}
	}
	
	public synchronized void newGoal(Goal g) {
		goals.add(g);
		if (goals.size() == 1) {
			processGoals();
		}
	}
	
	public synchronized void clearGoals() {
		goals.clear();
		immediateLock = false;
		processGoals();
	}
	
	public Goal currentGoal() {
		if (goals.size() == 0) {
			return null;
		} else {
			return goals.get(0);
		}
	}
	
	public synchronized Configuration getConfiguration() {
		return current;
	}
	
	public synchronized void setConfiguration(Configuration c) {
		current = c;
	}
	
	public synchronized boolean replan() {
		if (goals.size() == 0) {
			return false;
		}
		
		Configuration start = getConfiguration();
		Goal goal = goals.get(0);		
		for (PlanningParameters param : Constants.PLANNING_ATTEMPTS) {
			List<Waypoint> newPath = planner.plan(start, goal, param); 
			if (newPath != null) {
				System.out.println("Found path of length " + newPath.size() + " from " + start + " to " + goal + "\nTook " + planner.attempts + " attempts, " +
						planner.iterations + " iterations, " + planner.time/1000.0 + " seconds");
				setPath(newPath);
				draw();

				return true;
			} 
		}
		
		System.out.println("Could not find path from " + start + " to " + goal);
		return false;
	}
	
	public void replanner() {
		while (true) {
			Util.pause(10);
			
			if (replanNeeded) {
				replan();
				replanNeeded = false;
			}
		}
	}
	
	public void draw() {
		gui.clear();
		gui.draw(getConfiguration());
		gui.draw();
		if (goals.size() != 0) {
			if (goals.get(0) instanceof Configuration) { //TODO - make less hacky
				gui.draw(world.getRobot((Configuration)goals.get(0)), false, Color.RED);
			} else {
				gui.draw((Point)goals.get(0), Color.RED);
			}
		}	
		gui.draw(path, Color.BLUE);
	}
	
	public synchronized void setPath(List<Waypoint> newPath) { //TODO Replan more frequently
		path = newPath;
		pathReset = true;
	}
	
	public synchronized Waypoint nextWaypoint() {
		if (path.size() > 0) {
			return path.remove(0);
		} else {
			return null;
		}
	}

	public MotionMsg createMotionMsg(double tv, double rv) {
		MotionMsg msg = new MotionMsg();
		msg.translationalVelocity = tv;
		msg.rotationalVelocity = rv;
		return msg;
	}
	
	public synchronized void freeze() {
		if (!frozen) {
			frozen = true;
			System.out.println("Navigation frozen\n");
			Util.pause(10); //TODO remove and fix concurrency bug
			setPath(new LinkedList<Waypoint>());
			motionPub.publish(createMotionMsg(0., 0.));
			draw();
		}
	}
	
	public synchronized void resume() {
		if (frozen) {
			System.out.println("Navigation resumed\n");
			processGoals();
			frozen = false;
		}
	}
	
	
	public boolean safePath(Configuration start, Configuration end, DriveSystem drive, CollisionCheck check) {
		for (Configuration config : start.interpolatePath(end, drive)) {
			if (world.robotCollision(config, 0.0, check)) {
				return false;
			}
		}
		return true;
	}
	
	//TODO - remove unused launch components
	//TODO - remove netbook window of robot spaz
	//TODO - Alec says that the given map is complete wrt obstacles (ie it includes each perfectly), but the block positions may change
	//TODO - If replanning frequently, it will automatically try forward after moving backward out of tough space
	//TODO - trianglulation using two fiducials and an angle differntial. Reset reference and initial
	
	public MotionMsg computeForwardVelocities(Configuration start, Configuration end) {
		if (!safePath(start, end, DriveSystem.FORWARD, CollisionCheck.MAPONLY)) {
			replanNeeded = true;
			return createMotionMsg(0., 0.);
		}
		
		double translateAngle =  Util.vectorAngle(end.x - start.x ,end.y - start.y);
		double directionError = Util.angleDistance(start.theta, translateAngle);
		double translationError = Util.vectorLength(end.x - start.x, end.y - start.y);
		double angleError = Util.angleDistance(start.theta, Util.cleanAngle(end.theta));				
						
		double tv = 0.;
		double rv = 0.; //TODO collision checks to make sure if off path that it doesn't collide
		if (translationError > Constants.TRANSLATION_THRESHOLD) {
			if (directionError > Constants.ROTATION_THRESHOLD) { 
				double rotationSpeed = Math.max(Math.min(Constants.K_ROTATE*directionError, Constants.MAX_RV), Constants.MIN_RV);
				rv = rotationSpeed*Util.angleDistanceSign(start.theta, translateAngle);
			} 
			else {
				double translationSpeed = Math.max(Math.min(Constants.K_TRANSLATE*translationError, Constants.MAX_TV), Constants.MIN_TV);
				tv = translationSpeed;
			}
		} else if (angleError > Constants.ROTATION_THRESHOLD) { 
			double rotationSpeed = Math.max(Math.min(Constants.K_ROTATE*angleError, Constants.MAX_RV), Constants.MIN_RV);
			rv = rotationSpeed*Util.angleDistanceSign(start.theta, end.theta);
		} else {
			//Reached waypoint
			return null;
		}
		return createMotionMsg(tv, rv);
	}
	
	public MotionMsg computeBackwardVelocities(Configuration start, Configuration end) {
		if (!safePath(start, end, DriveSystem.BACKWARD, CollisionCheck.MAPONLY)) {
			replanNeeded = true;
			return createMotionMsg(0., 0.);
		}
		
		double translateAngle = Util.cleanAngle(Util.vectorAngle(end.x - start.x ,end.y - start.y) + Math.PI);
		double directionError = Util.angleDistance(start.theta, translateAngle);
		double translationError = Util.vectorLength(end.x - start.x, end.y - start.y);
		double angleError = Util.angleDistance(start.theta, Util.cleanAngle(end.theta));				
						
		double tv = 0.;
		double rv = 0.;
		if (translationError > Constants.TRANSLATION_THRESHOLD) {
			if (directionError > Constants.ROTATION_THRESHOLD) { 
				double rotationSpeed = Math.max(Math.min(Constants.K_ROTATE*directionError, Constants.MAX_RV), Constants.MIN_RV);
				rv = rotationSpeed*Util.angleDistanceSign(start.theta, translateAngle);
			} 
			else {
				double translationSpeed = Math.max(Math.min(Constants.K_TRANSLATE*translationError, Constants.MAX_TV), Constants.MIN_TV);
				tv = -translationSpeed;
			}
		} else if (angleError > Constants.ROTATION_THRESHOLD) { 
			double rotationSpeed = Math.max(Math.min(Constants.K_ROTATE*angleError, Constants.MAX_RV), Constants.MIN_RV);
			rv = rotationSpeed*Util.angleDistanceSign(start.theta, end.theta);
		} else {
			//Reached waypoint
			return null;
		}
		return createMotionMsg(tv, rv);
	}
	
	public MotionMsg computeFOBVelocities(Configuration start, Configuration end) {
		if (start.distanceForward(end) < start.distanceBackward(end)) {
			return computeForwardVelocities(start, end);
		} else {
			return computeBackwardVelocities(start, end);
		}
	}
	
	//TODO if drive system path is no longer good then replan
	public MotionMsg computeVelocities(Configuration start, Waypoint waypoint) {
		switch(waypoint.drive) {
		case FORWARD: return computeForwardVelocities(start, waypoint.config);
		case BACKWARD: return computeBackwardVelocities(start, waypoint.config);
		case FOB: return computeFOBVelocities(start, waypoint.config);
		default: return null;
		}		
	}
	
	public void motorController() { 		
		System.out.println("Started Navigator Loop");
		long startTime = System.currentTimeMillis();
		while (true) {
			Util.pause(1); //TODO remove and fix concurrency bug
			Waypoint waypoint = nextWaypoint();
			if (waypoint == null) {
				if (!frozen) {
					motionPub.publish(createMotionMsg(0., 0.));
				}
				continue;
			}
			pathReset = false;
			System.out.println("New Waypoint: " + waypoint.config + "\n");
			while (true) {			
				Util.pause(1); //TODO remove and fix concurrency bug
				
				if (pathReset) {
					pathReset = false;
					break;
				}
				
				Configuration configuration = getConfiguration();
				MotionMsg msg = null;
				if (!frozen) {
					msg = computeVelocities(configuration, waypoint); //TODO - only recompute if something has changed
					if (msg == null) {
						if (path.size() == 0) {
							//Reached goal
							accomplished.add(goals.remove(0));
							immediateLock = false;
							processGoals();
						}
						break;
					} 
					motionPub.publish(msg);
				}	
				
				if ((System.currentTimeMillis() - startTime) % 2000 == 0) {
					System.out.println("Time: " + (System.currentTimeMillis() - startTime)/1000 + " seconds");
					System.out.println("Current: " + configuration);
					System.out.println("Waypoint: " + waypoint.config);
					if (msg != null) {
						System.out.println("Velocity: " + msg.translationalVelocity + ", " + msg.rotationalVelocity + "\n");
					} else {
						System.out.println("Velocity: N/A, N/A\n");
					}
				}
			}
		}
	}
}
