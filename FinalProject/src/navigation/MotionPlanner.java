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
	public Configuration startConfig;
	public Configuration goalConfig;
	
	public RRT tree1, tree2;
	public int iterations;
	public int attempts;
	public long time;
	
	public MotionPlanner(World world) {
		this.world = world;
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
	
	public List<Waypoint> interpolatePath(Waypoint start, Waypoint end, PlanningParameters param) {
		List<Waypoint> waypoints = new LinkedList<Waypoint>();
		for (Configuration config : start.config.interpolatePath(end.config, param.drive)) {
			//double distance = Math.min(start.distance + start.config.distance(config, param.drive), end.distance + config.distance(end.config, param.drive));
			double distance = Math.min(config.cartesianDistance(startConfig), config.cartesianDistance(goalConfig));
			waypoints.add(new Waypoint(config, param.drive, distance, param.computeGrow(distance)));
		}
		return waypoints;
	}
	
	public boolean checkPath(List<Waypoint> path, PlanningParameters param) {
		for (Waypoint waypoint : path) {
			if (world.robotCollision(waypoint.config, waypoint.grow, param.check)) {
				return false;
			}
		}
		return true;
	}

	public boolean safePath(Waypoint start, Waypoint end, PlanningParameters param) {
		return checkPath(interpolatePath(start, end, param), param);
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

	private List<Waypoint> bidirectionalRRT(Configuration start, Goal goal, PlanningParameters param) {
		tree1 = new RRT(start, true, param.drive, param.computeGrow(0.));
		if (world.robotCollision(start, param.computeGrow(0.), param.check)) {
			return null;
		}
		
		tree2 = new RRT(null, false, param.drive, -1);
		for (Configuration end : goal.goalConfigurations()) {
			if (!world.robotCollision(end, param.computeGrow(0.), param.check)) {
				tree2.root.children.add(new TreeNode(end, param.computeGrow(0.)));
				tree2.size += 1;
			}
		}
		if (tree2.root.children.size() == 0) {
			return null;
		}
		
		long startTime = System.currentTimeMillis();
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
				double distance = Math.min(config.cartesianDistance(startConfig), config.cartesianDistance(goalConfig));
				//double distance = lastNode1.distance + lastNode1.config.distance(config, tree1.drive);
				double grow = param.computeGrow(distance);
				if (!world.robotCollision(config, grow, param.check)) {
					lastNode1 = tree1.add(lastNode1, config, param.drive, distance, grow);
				} else {
					break;
				}
			}

			TreeNode closestNode2 = tree2.closest(lastNode1.config);
			TreeNode lastNode2 = closestNode2;
			for (Configuration config : tree2.interpolatePath(closestNode2.config, lastNode1.config)) {
				double distance = Math.min(config.cartesianDistance(startConfig), config.cartesianDistance(goalConfig));
				//double distance = lastNode2.distance + lastNode2.config.distance(config, tree2.drive);
				double grow = param.computeGrow(distance);
				if (!world.robotCollision(config, grow, param.check)) {
					lastNode2 = tree2.add(lastNode2, config, param.drive, distance, grow);
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
					path.add(new Waypoint(node.config, node.drive, node.distance, node.grow)); //TODO - ensure correct distances
				}
				time = System.currentTimeMillis() - startTime;
				iterations = i;
				return path;
			}
		}
		time = System.currentTimeMillis() - startTime;
		iterations = i;
		return null;
	}

	public List<Waypoint> smoothPath(List<Waypoint> path, PlanningParameters param) { 
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
			List<Waypoint> shortcut = interpolatePath(path.get(start), path.get(end), param);
			if (checkPath(shortcut, param)) {
				for (int j = end - 1; j > start; j--) {
					path.remove(j);
				}
				for (int j = 0; j < shortcut.size(); j++) {					
					path.add(start + 1 + j, shortcut.get(j));  //TODO - ensure correct distances
				}
			}
		}
		return path;
	}
	
	public List<Waypoint> extractPath(List<Waypoint> path, PlanningParameters param) {
		for (int i = 0; i < Constants.RRT_SMOOTHING; i++) {
			if (path.size() <= 2) {
				break;
			}
			int a = (int) (path.size()* Math.random());
			int b = (int) (path.size()* Math.random());
			if (Math.abs(a - b) <= 1) {
				continue;
			}

			int start = Math.min(a, b); //TODO - ensure correct distances
			int end = Math.max(a, b);
			if (safePath(path.get(start), path.get(end), param)) { 
				for (int j = end - 1; j > start; j--) {
					path.remove(j);
				}

			}
		}
		return path;
	}

	public List<Waypoint> findPath(Configuration start, Goal goal, PlanningParameters param) {
		this.startConfig = start;
		if (goal instanceof Point) { //TODO - sketchy
			this.goalConfig = ((Point)goal).configuration(0);
		} else {
			this.goalConfig = ((Configuration)goal);
		}
		
		Waypoint startWaypoint = new Waypoint(start, param.drive, 0., param.computeGrow(0.));
		for (Configuration end :  goal.goalConfigurations()) {
			Waypoint endWaypoint = new Waypoint(end, param.drive, 0., param.computeGrow(0.));
			if (safePath(startWaypoint, endWaypoint, param)) {
				return new LinkedList<Waypoint>(Arrays.asList(endWaypoint));
			}
		}
		
		for (int i = 0; i < Constants.RRT_ATTEMPTS; i++) {
			List<Waypoint> path = bidirectionalRRT(start, goal, param);
			if (path != null) {
				attempts = i + 1;
				//path = smoothPath(path, param);
				//path = extractPath(path, param);
				path = extractPath(smoothPath(path, param), param);
				path.remove(0);
				return path;
			}
		}
		attempts = Constants.RRT_ATTEMPTS;
		return null;
	}
}
