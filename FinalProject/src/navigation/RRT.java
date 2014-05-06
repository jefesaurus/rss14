package navigation;

import java.util.List;
import navigation.Constants.DriveSystem;

public class RRT {
	public TreeNode root;
	public int size;
	public boolean startTree;
	public DriveSystem drive;

	public RRT(Configuration start, boolean startTree, DriveSystem drive, double grow) {
		this.root = new TreeNode(start, drive, grow);
		this.size = 1;
		this.startTree = startTree;
		this.drive = Constants.computeDrive(drive, !startTree);
	}

	public TreeNode add(TreeNode parent, Configuration config, DriveSystem localDrive, double distance, double grow) {
		TreeNode child = new TreeNode(config, parent, localDrive, distance, grow);
		parent.children.add(child);
		size++;
		return child;
	}
	
	public double distance(Configuration start, Configuration end) {
		if (start == null || end == null) {
			return Double.POSITIVE_INFINITY;
		} else {
			return start.distance(end, drive);
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

	public TreeNode closest(Configuration config) { // TODO - place Nodes in array to make closest faster
		return closestRecur(config, root);
	}
	
	public List<Configuration> interpolatePath(Configuration start, Configuration end) {
		return start.interpolatePath(end, drive);
	}
}
