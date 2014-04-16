package navigation;

import java.util.List;

public class RRT {
	public TreeNode root;
	public int size;
	public boolean forward;

	public RRT(Configuration start, boolean forward) {
		this.root = new TreeNode(start);
		this.size = 1;
		this.forward = forward;
	}

	public TreeNode add(TreeNode parent, Configuration config) {
		TreeNode child = new TreeNode(config, parent);
		parent.children.add(child);
		size++;
		return child;
	}
	
	public double distance(Configuration start, Configuration end) {
		if (forward) {
			return start.distanceForward(end);
		} else {
			return start.distanceBackward(end);
		}
	}

	private TreeNode closestRecur(Configuration config, TreeNode node) {
		TreeNode closestNode = node;
		double closestDistance = distance(node.config, config);
		for (TreeNode child : node.children) {
			TreeNode closetChild = closestRecur(config, child);
			double distance = distance(closetChild.config, config);
			if (distance < closestDistance) {
				closestNode = closetChild;
				closestDistance = distance;
			}
		}

		return closestNode;
	}

	public TreeNode closest(Configuration config) { // TODO - place Nodes in array
												// to make closest faster
		return closestRecur(config, root);
	}
	
	public List<Configuration> interpolatePath(Configuration start, Configuration end) {
		if (forward) {
			return start.interpolatePathForward(end);
		} else {
			return start.interpolatePathBackward(end);
		}
	}
}
