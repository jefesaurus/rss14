package navigation;

import java.util.LinkedList;
import java.util.List;

public class TreeNode {
	public Configuration config;
	public List<TreeNode> children;
	public TreeNode parent;

	public TreeNode(Configuration config) {
		this.config = config;
		this.children = new LinkedList<TreeNode>();
		this.parent = null;
	}

	public TreeNode(Configuration config, TreeNode parent) {
		this.config = config;
		this.children = new LinkedList<TreeNode>();
		this.parent = parent;
	}
}