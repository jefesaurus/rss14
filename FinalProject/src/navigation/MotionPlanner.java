package navigation;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import navigation.Constants.CollisionCheck;
import navigation.Constants.DriveSystem;

//TODO - options for drive system parameters (ie Forward, Backward, FOB, probabilities proportional to shortest distance)
//TODO - enforce leaving/entering point to drive a certain way
//TODO - return what type of movement supposed to do

public class MotionPlanner {
	private World world;
	
	public RRT tree1, tree2;
	public Configuration rrtInitial;
	public Goal rrtGoal;
	public PlanningParameters rrtParam;
	
	public int iterations;
	public int attempts;
	public long time;
	
	public MotionPlanner(World world) {
		this.world = world;
		this.tree1 = null;
		this.tree2 = null;
		this.rrtInitial = null;
		this.rrtGoal = null;
		this.rrtParam = null;
	}
	
	private Configuration sampleConfiguration() {
		double x = world.getRegion().min.x
				+ (world.getRegion().max.x - world.getRegion().min.x)
				* Math.random();
		double y = world.getRegion().min.y
				+ (world.getRegion().max.y - world.getRegion().min.y)
				* Math.random();
		double theta = 2 * Math.PI * Math.random();

		return new Configuration(x, y, theta);
	}
	
	public List<Waypoint> interpolatePath(Waypoint start, Waypoint end) {
		List<Waypoint> waypoints = new LinkedList<Waypoint>();
		for (Configuration config : start.config.interpolatePath(end.config, rrtParam.drive)) {
			double distance = Math.min(rrtInitial.goalDistance(config), rrtGoal.goalDistance(config));
			waypoints.add(new Waypoint(config, rrtParam.drive, distance, rrtParam.computeGrow(distance)));
		}
		return waypoints;
	}
	
	public boolean checkPath(List<Waypoint> path) {
		for (Waypoint waypoint : path) {
			if (world.robotCollision(waypoint.config, waypoint.grow, rrtParam.check)) {
				return false;
			}
		}
		return true;
	}

	public boolean safePath(Waypoint start, Waypoint end) {
		return checkPath(interpolatePath(start, end));
	}
	
	private List<TreeNode> pathToRoot(TreeNode node) {
		List<TreeNode> path = new LinkedList<TreeNode>();
		node = node.parent;
		while (node != null) {
			path.add(node);
			node = node.parent;
		}
		return path;
	}

	private List<Waypoint> bidirectionalRRT() { //TODO - different drives in the RRT
		tree1 = new RRT(rrtInitial, true, rrtParam.drive, rrtParam.computeGrow(0.));
		if (world.robotCollision(rrtInitial, rrtParam.computeGrow(0.), rrtParam.check)) {
			iterations = 0;
			return null;
		}
		
		tree2 = new RRT(null, false, rrtParam.drive, -1);
		for (Configuration end : rrtGoal.goalConfigurations()) {
			if (!world.robotCollision(end, rrtParam.computeGrow(0.), rrtParam.check)) {
				tree2.root.children.add(new TreeNode(end, rrtParam.drive, rrtParam.computeGrow(0.)));
				tree2.size += 1;
			}
		}
		if (tree2.root.children.size() == 0) {
			iterations = 0;
			return null;
		}
		
		int i;
		for (i = 0; i < Constants.RRT_ITERATIONS; i++) {
			if (tree1.size > tree2.size) {
				RRT temp = tree1;
				tree1 = tree2;
				tree2 = temp;
			}
			Configuration sample = sampleConfiguration();

			TreeNode closestNode1 = tree1.closest(sample);
			TreeNode lastNode1 = closestNode1;
			for (Configuration config : tree1.interpolatePath(closestNode1.config, sample)) {
				double distance = Math.min(rrtInitial.goalDistance(config), rrtGoal.goalDistance(config));
				double grow = rrtParam.computeGrow(distance);
				if (!world.robotCollision(config, grow, rrtParam.check)) {
					lastNode1 = tree1.add(lastNode1, config, rrtParam.drive, distance, grow);
				} else {
					break;
				}
			}

			TreeNode closestNode2 = tree2.closest(lastNode1.config);
			TreeNode lastNode2 = closestNode2;
			for (Configuration config : tree2.interpolatePath(closestNode2.config, lastNode1.config)) {
				double distance = Math.min(rrtInitial.goalDistance(config), rrtGoal.goalDistance(config));
				double grow = rrtParam.computeGrow(distance);
				if (!world.robotCollision(config, grow, rrtParam.check)) {
					lastNode2 = tree2.add(lastNode2, config, rrtParam.drive, distance, grow);
				} else {
					break;
				}
			}

			if (Math.abs(lastNode1.config.x - lastNode2.config.x) + Math.abs(lastNode1.config.y - lastNode2.config.y) + Math.abs(lastNode1.config.theta - lastNode2.config.theta) < .001) {
				if (!tree1.startTree) {
					RRT temp = tree1;
					tree1 = tree2;
					tree2 = temp;
					
					TreeNode t = lastNode1;
					lastNode1 = lastNode2;
					lastNode2 = t;
				}
				
				List<Waypoint> path = new LinkedList<Waypoint>();
				for (TreeNode node : pathToRoot(lastNode1)) {
					path.add(0, new Waypoint(node.config, node.drive, node.distance, node.grow));
				}
				path.add(new Waypoint(lastNode1.config, lastNode1.drive, lastNode1.distance, lastNode1.grow));
				for (TreeNode node : pathToRoot(lastNode2)) {
					path.add(new Waypoint(node.config, node.drive, node.distance, node.grow));
				}
				iterations = i;
				return path;
			}
		}
		iterations = i;
		return null;
	}

	public List<Waypoint> smoothPath(List<Waypoint> path) { 
		for (int i = 0; i < Constants.RRT_SMOOTHING; i++) {
			if (path.size() <= 2) {
				break;
			}
			int a = (int) (path.size()* Math.random());
			int b = (int) (path.size()* Math.random());
			if (Math.abs(a - b) <= 1) {
				continue;
			}

			int start = Math.min(a, b);
			int end = Math.max(a, b);
			List<Waypoint> shortcut = interpolatePath(path.get(start), path.get(end));
			if (checkPath(shortcut)) {
				for (int j = end - 1; j > start; j--) {
					path.remove(j);
				}
				for (int j = 0; j < shortcut.size(); j++) {					
					path.add(start + 1 + j, shortcut.get(j));
				}
			}
		}
		return path;
	}
	
	public List<Waypoint> extractPath(List<Waypoint> path) {
		for (int i = 0; i < Constants.RRT_SMOOTHING; i++) {
			if (path.size() <= 2) {
				break;
			}
			int a = (int) (path.size()* Math.random());
			int b = (int) (path.size()* Math.random());
			if (Math.abs(a - b) <= 1) {
				continue;
			}

			int start = Math.min(a, b);
			int end = Math.max(a, b);
			if (safePath(path.get(start), path.get(end))) { 
				for (int j = end - 1; j > start; j--) {
					path.remove(j);
				}

			}
		}
		return path;
	}
	
	private List<Waypoint> lengthenPath(List<Waypoint> waypoints) { //it underestimates the distance heavily
		double compenstation = .2;
		List<Waypoint> longPath = new LinkedList<Waypoint>();
		longPath.add(waypoints.get(0));
		for (int i = 1; i < waypoints.size(); i ++ ) {
			Waypoint pastWaypoint = waypoints.get(i-1);
			Waypoint currentWaypoint = waypoints.get(i);
			double translateAngle =  Util.vectorAngle(currentWaypoint.config.x - pastWaypoint.config.x, currentWaypoint.config.y - pastWaypoint.config.y);
			Configuration newConfig = new Configuration(currentWaypoint.config.x + compenstation*Math.cos(translateAngle), 
					currentWaypoint.config.y + compenstation*Math.sin(translateAngle), currentWaypoint.config.theta);
			
			longPath.add(new Waypoint(newConfig, currentWaypoint.drive, currentWaypoint.distance, currentWaypoint.grow));
		}
		return longPath;
	}
	
	private List<Waypoint> findPath(Configuration rrtInitial, Goal rrtGoal, PlanningParameters rrtParam) {
		long startTime = System.currentTimeMillis();
		attempts = 0;
		iterations = 0;
		
		this.rrtInitial = rrtInitial;
		this.rrtGoal = rrtGoal;
		this.rrtParam = rrtParam;
		
		Waypoint startWaypoint = new Waypoint(rrtInitial, rrtParam.drive, 0., rrtParam.computeGrow(0.));
		for (Configuration end :  rrtGoal.goalConfigurations()) {
			Waypoint endWaypoint = new Waypoint(end, rrtParam.drive, 0., rrtParam.computeGrow(0.));
			if (safePath(startWaypoint, endWaypoint)) {
				time = System.currentTimeMillis() - startTime;
				return new LinkedList<Waypoint>(Arrays.asList(endWaypoint));
			}
		}		
		
		for (int i = 0; i < Constants.RRT_ATTEMPTS; i++) {
			List<Waypoint> path = bidirectionalRRT();
			if (path != null) {
				//path = smoothPath(path);
				//path = extractPath(path);
				path = extractPath(smoothPath(path));
				
				//path = lengthenPath(path);
				
				path.remove(0);
				
				attempts = i + 1;
				time = System.currentTimeMillis() - startTime;
				return path;
			}
		}
		attempts = Constants.RRT_ATTEMPTS;
		time = System.currentTimeMillis() - startTime;
		return null;
	}
	
	//////////////////////////////////////////////////
	
	public Waypoint safeForward(Configuration start) {
		double minDistance = .1; //Make it move to get out of an object
		double maxDistance = minDistance + 2*Constants.ROBOT_RADIUS;
		double distance;
		for (distance = minDistance; distance < 2*maxDistance; distance += Constants.TRANSLATION_STEP_DISTANCE) {
			Configuration config = new Configuration(start.x + distance*Math.cos(start.theta), start.y + distance*Math.sin(start.theta), start.theta);			
			if (world.robotCollision(config, 0., CollisionCheck.MAPONLY)) {
				break;
			}
		}
		double endDistance = Math.max(Math.min(maxDistance, distance/2.), minDistance);
		Configuration end = new Configuration(start.x + endDistance*Math.cos(start.theta), start.y + endDistance*Math.sin(start.theta), start.theta);
		return new Waypoint(end, DriveSystem.FORWARD, 0., 0.);
	}
	
	public Waypoint safeBackward(Configuration start) {
		double minDistance = .1; //Make it move to get out of an object
		double maxDistance = minDistance + Constants.ROBOT_RADIUS;
		double distance;
		for (distance = minDistance; distance < 2*maxDistance; distance += Constants.TRANSLATION_STEP_DISTANCE) {
			Configuration config = new Configuration(start.x - distance*Math.cos(start.theta), start.y - distance*Math.sin(start.theta), start.theta);			
			if (world.robotCollision(config, 0., CollisionCheck.MAPONLY)) {
				break;
			}
		}
		double endDistance = Math.max(Math.min(maxDistance, distance/2.), minDistance);
		Configuration end = new Configuration(start.x - endDistance*Math.cos(start.theta), start.y - endDistance*Math.sin(start.theta), start.theta);
		return new Waypoint(end, DriveSystem.BACKWARD, 0., 0.);
	}

	public List<Waypoint> plan(Configuration initial, Goal goal, PlanningParameters param) {
		if (param.drive == DriveSystem.FORWARDINITIALREVERSE) {
			Waypoint reverse = safeBackward(initial);
			List<Waypoint> path = findPath(reverse.config, goal, param.changeDrive(DriveSystem.FORWARD));
			if (path != null) {
				path.add(0, reverse);
			}
			return path;
		} else {
			return findPath(initial, goal, param);
		}
	}
}
