package navigation;

import java.util.LinkedList;
import java.util.List;

import navigation.Constants.CollisionCheck;
import navigation.Constants.DriveSystem;

//TODO - options for drive system parameters (ie Forward, Backward, FOB, probabilities proportional to shortest distance)
//TODO - combine grow and collision check into 1 parameter

public class MotionPlanner {
	private World world;
	public RRT tree1, tree2;
	public int iterations;
	public int attempts;
	
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
	
	public boolean checkPath(List<Configuration> path, double grow, CollisionCheck check) {
		for (Configuration config : path) {
			if (world.robotCollision(config, grow, check)) {
				return false;
			}
		}
		return true;
	}

	public boolean safePath(Configuration start, Configuration end, DriveSystem drive, double grow, CollisionCheck check) {
		return checkPath(start.interpolatePath(end, drive), grow, check);
	}
	
	private List<Configuration> pathToRoot(TreeNode node) {
		List<Configuration> path = new LinkedList<Configuration>();
		while (node != null) {
			path.add(node.config);
			node = node.parent;
		}
		return path;
	}

	private List<Configuration> bidirectionalRRT(Configuration start, Configuration end, DriveSystem drive, double grow, CollisionCheck check) {
		tree1 = new RRT(start, true, drive);
		tree2 = new RRT(end, false, drive);
		
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
				if (!world.robotCollision(config, grow, check)) {
					lastNode1 = tree1.add(lastNode1, config);
				} else {
					break;
				}
			}

			TreeNode closestNode2 = tree2.closest(lastNode1.config);
			TreeNode lastNode2 = closestNode2;
			for (Configuration config : tree2.interpolatePath(closestNode2.config, lastNode1.config)) {
				if (!world.robotCollision(config, grow, check)) {
					lastNode2 = tree2.add(lastNode2, config);
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
				
				List<Configuration> path = new LinkedList<Configuration>();
				for (Configuration config : pathToRoot(lastNode1)) {
					path.add(0, config);
				}
				path.add(lastNode1.config);
				path.addAll(pathToRoot(lastNode2));
				iterations = i;
				return path;
			}
		}
		iterations = i;
		return null;
	}

	public List<Configuration> smoothPath(List<Configuration> path, DriveSystem drive, double grow, CollisionCheck check) {
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
			List<Configuration> shortcut = path.get(start).interpolatePath(path.get(end), drive);
			if (checkPath(shortcut, grow, check)) {
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
	
	public List<Configuration> extractPath(List<Configuration> path, DriveSystem drive, double grow, CollisionCheck check) {
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
			if (safePath(path.get(start), path.get(end), drive, grow, check)) {
				for (int j = end - 1; j > start; j--) {
					path.remove(j);
				}

			}
		}
		return path;
	}

	public List<Configuration> findPath(Configuration start, Configuration end, DriveSystem drive, double grow, CollisionCheck check) { //TODO - don't return first waypoint
		for (int i = 0; i < Constants.RRT_ATTEMPTS; i++) {
			List<Configuration> path = bidirectionalRRT(start, end, drive, grow, check);
			if (path != null) {
				attempts = i + 1;
				return extractPath(smoothPath(path, drive, grow, check), drive, grow, check); //TODO - try a couple smoothed paths to find the shortest one
			}
		}
		attempts = Constants.RRT_ATTEMPTS;
		return null;
	}
}
