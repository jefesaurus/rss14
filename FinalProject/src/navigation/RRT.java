package navigation;

public class RRT {
	public TreeNode root;
	public int size;

	public RRT(Configuration start) {
		this.root = new TreeNode(start);
		this.size = 1;
	}

	public TreeNode add(TreeNode parent, Configuration config) {
		TreeNode child = new TreeNode(config, parent);
		parent.children.add(child);
		size++;
		return child;
	}

	private TreeNode closestRecur(Configuration config, TreeNode node) {
		TreeNode closestNode = node;
		double closestDistance = node.config.distance(config);
		for (TreeNode child : node.children) {
			TreeNode closetChild = closestRecur(config, child);
			double distance = closetChild.config.distance(config);
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
}
