package navigation;

import java.util.LinkedList;
import java.util.List;

public class MotionPlanner {
	private Polygon robot;
	private List<Polygon> obstacles;
	private Polygon region;

	public MotionPlanner(Polygon robot, List<Polygon> obstacles, Polygon region) {
		this.robot = robot;
		this.obstacles = obstacles;
		this.region = region;
	}

	private Configuration sampleConfiguration() {
		double x = region.boundingBox.min.x
				+ (region.boundingBox.max.x - region.boundingBox.min.x)
				* Math.random();
		double y = region.boundingBox.min.y
				+ (region.boundingBox.max.y - region.boundingBox.min.y)
				* Math.random();
		double theta = 2 * Math.PI * Math.random();

		return new Configuration(x, y, theta);
	}

	private boolean collides(Configuration config) {
		Polygon placedRobot = robot;//.pose(config); TODO

		// TODO - region contains

		for (Polygon obstacle : obstacles) {
			if (placedRobot.collides(obstacle)) {
				return true;
			}
		}
		return false;
	}

	private boolean safePath(List<Configuration> path) {
		for (Configuration config : path) {
			if (collides(config)) {
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

	private List<Configuration> bidirectionalRRT(Configuration start,
			Configuration end) {
		RRT tree1 = new RRT(start);
		RRT tree2 = new RRT(end);

		for (int i = 0; i < Constants.RRT_ITERATIONS; i++) {
			if (tree1.size > tree2.size) {
				RRT temp = tree1;
				tree1 = tree2;
				tree2 = temp;
			}
			Configuration sample = sampleConfiguration();

			TreeNode closestNode1 = tree1.closest(sample);
			TreeNode lastNode1 = closestNode1;
			for (Configuration config : closestNode1.config
					.interpolatePath(sample)) {
				if (!collides(config)) {
					lastNode1 = tree1.add(lastNode1, config);
				} else {
					break;
				}
			}

			TreeNode closestNode2 = tree1.closest(lastNode1.config);
			TreeNode lastNode2 = closestNode2;
			for (Configuration config : closestNode2.config
					.interpolatePath(lastNode1.config)) {
				if (!collides(config)) {
					lastNode2 = tree2.add(lastNode2, config);
				} else {
					break;
				}
			}

			if (lastNode1.config == lastNode2.config) {
				List<Configuration> path = new LinkedList<Configuration>();
				for (Configuration config : pathToRoot(lastNode1)) {
					path.add(0, config);
				}
				path.add(lastNode1.config);
				path.addAll(pathToRoot(lastNode2));
				return path;
			}
		}
		return null;
	}

	public List<Configuration> smoothPath(List<Configuration> path) {
		for (int i = 0; i < Constants.RRT_SMOOTHING; i++) {
			if (path.size() <= 2) {
				break;
			}
			int a = (int) ((path.size() + 1) * Math.random());
			int b = (int) ((path.size() + 1) * Math.random());
			if (Math.abs(a - b) <= 1) {
				continue;
			}

			int start = Math.min(a, b);
			int end = Math.max(a, b);
			if (safePath(path.get(start).interpolatePath(path.get(end)))) {
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
				return smoothPath(path);
			}
		}
		return null;
	}

	// TODO - process path for navigation

}