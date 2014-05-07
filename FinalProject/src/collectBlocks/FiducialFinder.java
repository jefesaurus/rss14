package collectBlocks;

import java.awt.Point;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;

import navigation.Point3D;

public class FiducialFinder {

	// Assumes fiducials are pairs of same sized balls on top of one another
	
	public class FidPattern {
		public String topColor;
		public Point2D topCentroid;
		public Point3D topLocation3D;
		public String bottomColor;
		public Point2D bottomCentroid;
		public Point3D bottomLocation3D;
		public FidPattern(String topColor, Point2D topCentroid, Point3D topLocation3D, String bottomColor, Point2D bottomCentroid, Point3D bottomLocation3D) {
			this.topColor = topColor;
			this.topCentroid = topCentroid;
			this.topLocation3D = topLocation3D;
			this.bottomColor = bottomColor;
			this.bottomCentroid = bottomCentroid;
			this.bottomLocation3D = bottomLocation3D;
		}
	}
	
	private final double RADIUS_RATIO_THRESHOLD = .3;
	
	private ConnComp cct;
	
	public FiducialFinder() {
		cct = new ConnComp();
	}
	
	public List<FidPattern> findFids(List<BlockInfo> binfos) {
		List<FidPattern> answer = new ArrayList<FidPattern>();
		// I don't expect many infos so I'm going to check each pair of blocks (brute force)
		for (int i = 0; i < binfos.size()-1; i++) {
			for (int j = i+1; j < binfos.size(); j++) {
				BlockInfo top, bottom;
				// find out which one is on top (remember y pixels increase downwards
				if (binfos.get(i).centroid.getY() < binfos.get(j).centroid.getY()){
					top = binfos.get(i);
					bottom = binfos.get(j);
				} else {
					top = binfos.get(j);
					bottom = binfos.get(i);
				}
				//fiducial balls have to be different colors from each other
				if (top.color == bottom.color) continue;
				//determine an approximate radius used as a multiplier for thresholds
				double topRadius = Math.sqrt((double) (top.size/Math.PI));
				double bottomRadius = Math.sqrt((double) (bottom.size/Math.PI));
				//check if radii are similar enough
//				if (topRadius/bottomRadius > RADIUS_RATIO_THRESHOLD) continue;
				// check if x values close enough
				if (Math.abs(top.centroid.getX() - bottom.centroid.getX()) > Math.max(topRadius, bottomRadius)) 
					continue;
				// check if y values close enough
				if (Math.abs(top.centroid.getY() - bottom.centroid.getY()) < Math.max(topRadius, bottomRadius) || 
						Math.abs(top.centroid.getY() - bottom.centroid.getY()) > Math.max(topRadius, bottomRadius)*3) continue;
				// if everything passed this must be a fiducial
				FidPattern fid = new FidPattern(top.color, top.centroid, top.location3D, bottom.color, bottom.centroid, bottom.location3D);
				answer.add(fid);
				System.out.println("Fiducial top : " + top.color + " bottom : " + bottom.color);
			}
		}
		return answer;
	}
	
	public List<FidPattern> findFids(int[][][] src, double[][][] xyz, boolean[][] mask, int minPixelsPerGroup) {
		List<BlockInfo> binfos = cct.getBlockInfosForFrame(src, xyz, mask, minPixelsPerGroup, false);
		return findFids(binfos);
	}
	
}
