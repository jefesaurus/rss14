package navigation;

import gui.*;

import java.awt.Color;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import navigation.Constants.CollisionCheck;
import navigation.Constants.DriveSystem;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.MotionMsg;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

public class Navigator implements Runnable {
	public Node node;
	public Subscriber<org.ros.message.rss_msgs.BumpMsg> bumperSub;
	public Subscriber<org.ros.message.rss_msgs.OdometryMsg> odoSub;
	public Publisher<MotionMsg> motionPub;
	public Subscriber<org.ros.message.sensor_msgs.PointCloud2> kinSub;
	
	public NavigationGUI gui;
	public World world;
	public MotionPlanner planner;
	protected boolean firstUpdate, frozen, pathReset, immediateLock;
	protected Configuration reference, initial, current;
	
	protected List<Goal> goals;
	protected List<Waypoint> path;
	protected DriveSystem pathDrive;
	protected List<Goal> accomplished;
	
	
	// KANENNNCT
	public Pose3D kinectPose;
	HashMap<IntTuple, double[]> occupancy;
	public boolean KINECT_DATA = true;
	public boolean VISION_GUI = false;
	public VisionGUI vgui;
	
	public Navigator(Node node, NavigationGUI gui, World world) {
		this.node = node;
		this.gui = gui;
		this.world = world;
		this.planner = new MotionPlanner(world);
		
		this.firstUpdate = true;
		this.frozen = false;
		this.pathReset = false;
		this.immediateLock = false;
		this.reference = null;
		this.initial = world.getStart().configuration(0.); //Alec said that we can assume the robot starts facing east (angle 0)
		this.current = null;
		
		this.goals = Collections.synchronizedList(new LinkedList<Goal>());		
		this.path = new LinkedList<Waypoint>();
		this.pathDrive = null;
		accomplished = new LinkedList<Goal>();

    //Kinect
    this.occupancy = new HashMap<IntTuple, double[]>();
    this.kinectPose = new Pose3D(new Point3D(0.0, 0.67, 0.0), Math.PI/2, -Math.PI/2. - .571, 0.);
    if (VISION_GUI) {
      this.vgui = new VisionGUI();
    }
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
							freeze();
							if (left) {
								System.out.println("Left Bump Sensor Collision! Backing up\n");
							} else {
								System.out.println("Right Bump Sensor Collision! Backing up\n");
							}
							
							Configuration start = getConfiguration();
							Configuration backward = world.sampleBackwardsConfiguration(start);
							if (backward != null) { //TODO expand on this
								immediateLock = true;
								DriveSystem drive = DriveSystem.BACKWARD; //TODO is it actually moving backward?
								double grow = 0.;
								CollisionCheck check = CollisionCheck.MAPONLY;
								List<Waypoint> newPath = planner.findPath(start, backward, new ConstantGrowthParameters(drive, check, grow)); 
								if (newPath != null) {
									System.out.println("Found path of length " + newPath.size() + " from " + start + " to " + backward);
									goals.add(0, backward);
									setPath(newPath, drive);
									draw();
								}
							}
							resume();
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
				gui.setRobotPose(configuration.x, configuration.y, configuration.theta);
				setConfiguration(configuration);
			}
		});
		this.motionPub = node.newPublisher("command/Motors", "rss_msgs/MotionMsg");
		
		kinSub = node.newSubscriber("/camera/depth_registered/points", "sensor_msgs/PointCloud2");
		kinSub.addMessageListener(new MessageListener<org.ros.message.sensor_msgs.PointCloud2>() {
			@Override
			public synchronized void onNewMessage(org.ros.message.sensor_msgs.PointCloud2 message) {
        unpackPointCloudData((int)message.width, (int)message.height, (int)message.point_step, (int)message.row_step, message.data);
			}
		});
		
		
		long startTime = System.currentTimeMillis();	
		while (firstUpdate) {
			Util.pause(1); //TODO remove and fix concurrency bug
			if ((System.currentTimeMillis() - startTime) % 2000 == 0) {
				System.out.println("Waiting for First Odometry Message");
			}
		}
		
		new Thread(this).start();
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
		draw();
	}

	public synchronized void immediateGoal(Goal g) {
		if (!immediateLock) {
			goals.add(0, g);
			immediateLock = true;
			processGoals();
		}
	}
	
	public synchronized void newGoal(Goal g) {
		goals.add(g);
		if (goals.size() == 1) { //Currently the only goal
			processGoals();
		}
	}
	
	public synchronized void clearGoals() {
		goals.clear();
		immediateLock = false;
		setPath(new LinkedList<Waypoint>(), null); //TODO - include this?
		draw();
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
	
	public boolean replan() {
		if (goals.size() == 0) {
			//setPath(new LinkedList<Configuration>(), null); //TODO - include this?
			//draw();
			return false;
		}
		
		Configuration start = getConfiguration();
		Goal goal = goals.get(0);
		//TODO - sample back up point and see if feasible, then grow to move towards the goal
		
		for (PlanningParameters param : Constants.PLANNING_ATTEMPTS) {
			List<Waypoint> newPath = planner.findPath(start, goal, param); 
			if (newPath != null) {
				System.out.println("Found path of length " + newPath.size() + " from " + start + " to " + goal);
				setPath(newPath, param.drive);
				/*gui.clear();
				gui.draw();
				gui.draw(newPath, grow, Color.BLUE);
				gui.draw(planner.tree1.root, Color.BLUE);
				gui.draw(planner.tree2.root, Color.RED);
				gui.draw(world.getRobot(goal), false, Color.RED);*/
				draw();

				return true;
			} 
		}
		
		System.out.println("Could not find path from " + start + " to " + goal);
		setPath(new LinkedList<Waypoint>(), null);
		/*gui.clear();
		gui.draw();
		gui.draw(planner.tree1.root, Color.BLUE);
		gui.draw(planner.tree2.root, Color.RED);*/
		draw();

		return false;
	}
	
	public void draw() {
		gui.clear();
		gui.draw();
		if (goals.size() != 0) {
			if (goals.get(0) instanceof Configuration) { //TODO - fix this mess
				gui.draw(world.getRobot((Configuration)goals.get(0)), false, Color.RED);
			} else {
				gui.draw((Point)goals.get(0), Color.RED);
			}
		}	
		gui.draw(path, Color.BLUE);
	}
	
	public synchronized void setPath(List<Waypoint> newPath, DriveSystem drive) { //TODO Replan more frequently
		path = newPath;
		pathDrive = drive;
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
			setPath(new LinkedList<Waypoint>(), null);
			motionPub.publish(createMotionMsg(0., 0.));
			draw();
		}
	}
	
	public synchronized void resume() {
		if (frozen) {
			replan();
			System.out.println("Navigation resumed\n");
			frozen = false;
		}
	}
	
	//TODO - remove unused launch components
	//TODO - remove netbook window of robot spaz
	//TODO - automatically replan every couple seconds
	//TODO - Alec says that the given map is complete wrt obstacles (ie it includes each perfectly), but the block positions may change
	//If replanning frequently, it will automatically try forward after moving backward out of tough space
	//TODO - trianglulation using two fiducials and an angle differntial. Reset reference and initial
	
	public MotionMsg computeForwardVelocities(Configuration start, Configuration end) {
		//if (!planner.safePath(start, end, DriveSystem.FORWARD, 0., CollisionCheck.MAPONLY)) {
		//	replan();
		//	return null; //TODO collision checks to make sure if off path that it doesn't collide, else replan
		//}
		
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
		//if (!planner.safePath(start, end, DriveSystem.BACKWARD, 0., CollisionCheck.MAPONLY)) {
		//	replan();
		//	return null; //TODO collision checks to make sure if off path that it doesn't collide
		//}
		
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
	public MotionMsg computeVelocities(Configuration start, Configuration end) {
		switch(pathDrive) {
		case FORWARD: return computeForwardVelocities(start, end);
		case BACKWARD: return computeBackwardVelocities(start, end);
		case FOB: return computeFOBVelocities(start, end);
		default: return null;
		}		
	}
	
	@Override
	public void run() { 		
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
			System.out.println("New Waypoint: " + waypoint + "\n");
			while (true) {			
				Util.pause(1); //TODO remove and fix concurrency bug
				
				if (pathReset) {
					pathReset = false;
					break;
				}
				
				Configuration configuration = getConfiguration();
				MotionMsg msg = null;
				if (!frozen) {
					msg = computeVelocities(configuration, waypoint.config); //TODO - only recompute if something has changed
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
	

int X_OFFSET = 0;
int Y_OFFSET = 4;
int Z_OFFSET = 8;
int R_OFFSET = 18;
int G_OFFSET = 17;
int B_OFFSET = 16;

int START_COL = 100;
int END_COL = 540;
int START_ROW = 140;
int END_ROW = 480;


	  float OCCUPANCY_RESOLUTION = .02f;
	  int OCCUPANCY_THRESHOLD = 3;
	  public void unpackPointCloudData(int width, int height, int pointStep, int rowStep, byte[] data) {
	    int offset, x_i, y_i, z_i, r_i, g_i, b_i;
	    float x, y, z;
	    int r, g, b;
	    Point3D point;
	    double avg_h = 0;
	    double avg_s = 0;
	    double avg_v = 0;
	    int pic_height = this.END_ROW - this.START_ROW;
	    int pic_width = this.END_COL - this.START_COL;
	    Image rep = new Image(pic_width, pic_height);
	    for (int row = START_ROW; row < END_ROW; row ++) {
	      for (int col = START_COL; col < END_COL; col ++) {
	        offset = rowStep*row + pointStep*col;
	        x_i = offset+X_OFFSET;
	        y_i = offset+Y_OFFSET;
	        z_i = offset+Z_OFFSET;
	        r_i = offset+R_OFFSET;
	        g_i = offset+G_OFFSET;
	        b_i = offset+B_OFFSET;
	        x = Float.intBitsToFloat((data[x_i+3] & 0xff) << 24 | (data[x_i+2] & 0xff) << 16 | (data[x_i+1] & 0xff) << 8 | (data[x_i] & 0xff)); 
	        y = Float.intBitsToFloat((data[y_i+3] & 0xff) << 24 | (data[y_i+2] & 0xff) << 16 | (data[y_i+1] & 0xff) << 8 | (data[y_i] & 0xff)); 
	        z = Float.intBitsToFloat((data[z_i+3] & 0xff) << 24 | (data[z_i+2] & 0xff) << 16 | (data[z_i+1] & 0xff) << 8 | (data[z_i] & 0xff)); 
	        r = (data[r_i] & 0xff);
	        g = (data[g_i] & 0xff);
	        b = (data[b_i] & 0xff);
	        float[] hsv = new float[3];
	        if (!Float.isNaN(x) && !Float.isNaN(y) && !Float.isNaN(z)) {
	          point = kinectPose.fromFrame(new Point3D(x, y, z));
	          if (point.z > 0.0) {
	            IntTuple loc = new IntTuple((int)((point.x-.58)/OCCUPANCY_RESOLUTION), (int)(point.y/OCCUPANCY_RESOLUTION));
	            double[] point_data = occupancy.get(loc);
	            Color.RGBtoHSB(r,g,b,hsv);
	            //System.out.println("r: " + r + " g: " +g + " b: " + b);
	            //System.out.println("h: " + hsv[0] + " s: " + hsv[1] + " v: " + hsv[2]);
	            rep.setPixel(col - START_COL, row - START_ROW, data[r_i], data[g_i], data[b_i]);
	            if (point_data == null) {
	              occupancy.put(loc, new double[] {1,point.z,0,0,0});
	            } else {
	              point_data[0] ++;
	              point_data[1] += point.z;
	            }
	          }
	        }
	      }
	    }
	    if (VISION_GUI) {
	      vgui.setVisionImage(rep.toArray(), pic_width, pic_height);
	    }
      if (KINECT_DATA) {
        draw();
      }
	    for (Map.Entry<IntTuple, double[]> cell : occupancy.entrySet()) {
	      double[] point_data = cell.getValue();
	      double num_points = point_data[0];
	      if (point_data[0] > OCCUPANCY_THRESHOLD) {
	        if (KINECT_DATA) {
	          IntTuple loc = cell.getKey();
	          if (point_data[1]/num_points > .05) {
	            gui.addPoint(loc.x*OCCUPANCY_RESOLUTION, loc.y*OCCUPANCY_RESOLUTION, 1, Color.BLACK);
	          } else {
		        gui.addPoint(loc.x*OCCUPANCY_RESOLUTION, loc.y*OCCUPANCY_RESOLUTION, 1, Color.RED);
	          }
	        }
	      }
	    }
	    occupancy.clear();
	  }
}
