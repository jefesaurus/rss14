package navigation;

import java.util.LinkedList;
import java.util.List;

public class MotionPlanner {
	private World world;
	public RRT tree1, tree2;
	public int iterations;
	
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

	private boolean safePath(List<Configuration> path) {
		for (Configuration config : path) {
			if (world.robotCollision(config)) {
				return false;
			}
		}
		return true;
	}

	private List<Configuration> pathToRoot(TreeNode node) {
		List<Configuration> path = new LinkedList<Configuration>();
		while (node != null) {
			path.add(node.config);
			node = node.parent;
		}
		return path;
	}

	private List<Configuration> bidirectionalRRT(Configuration start, Configuration end) { //TODO MAKE SURE DRIVE IS CORRECT (ie it can move forward or backward for both trees)
		tree1 = new RRT(start, true);
		tree2 = new RRT(end, false);
		
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
				if (!world.robotCollision(config)) {
					lastNode1 = tree1.add(lastNode1, config);
				} else {
					break;
				}
			}

			TreeNode closestNode2 = tree2.closest(lastNode1.config);
			TreeNode lastNode2 = closestNode2;
			for (Configuration config : tree2.interpolatePath(closestNode2.config, lastNode1.config)) {
				if (!world.robotCollision(config)) {
					lastNode2 = tree2.add(lastNode2, config);
				} else {
					break;
				}
			}

			//if (lastNode1.config == lastNode2.config) {
			//if (tree1.distance(lastNode1.config, lastNode2.config) < .001) {
			if (Math.abs(lastNode1.config.x - lastNode2.config.x) + Math.abs(lastNode1.config.y - lastNode2.config.y) + Math.abs(lastNode1.config.theta - lastNode2.config.theta) < .001) { //TODO
				//System.out.println(lastNode1.config.theta + ", " + lastNode2.config.theta);
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

	public List<Configuration> smoothPath(List<Configuration> path) {
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
			List<Configuration> shortcut = path.get(start).interpolatePathForward(path.get(end));
			if (safePath(shortcut)) {
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
	
	public List<Configuration> extractPath(List<Configuration> path) {
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
			if (safePath(path.get(start).interpolatePathForward(path.get(end)))) {
				for (int j = end - 1; j > start; j--) {
					path.remove(j);
				}

			}
		}
		return path;
	}

	public List<Configuration> findPath(Configuration start, Configuration end) {
		for (int i = 0; i < Constants.RRT_ATTEMPTS; i++) {
			List<Configuration> path = bidirectionalRRT(start, end);
			if (path != null) {
				//return path;
				return smoothPath(path);
				//return extractPath(smoothPath(path));
			}
		}
		return null;
	}

	// TODO - process path for navigation

}