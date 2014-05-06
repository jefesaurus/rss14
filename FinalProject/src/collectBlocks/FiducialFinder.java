package collectBlocks;

import java.awt.Point;
import java.util.ArrayList;
import java.util.List;

public class FiducialFinder {

	// Assumes fiducials are pairs of same sized balls on top of one another
	
	public class FidPattern {
		public String topColor;
		public Point topCentroid;
		public String bottomColor;
		public Point bottomCentroid;
		public FidPattern(String topColor, Point topCentroid, String bottomColor, Point bottomCentroid) {
			this.topColor = topColor;
			this.topCentroid = topCentroid;
			this.bottomColor = bottomColor;
			this.bottomCentroid = bottomCentroid;
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
				if (binfos.get(i).centroid.y < binfos.get(j).centroid.y){
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
				if (Math.abs(top.centroid.x - bottom.centroid.x) > Math.max(topRadius, bottomRadius)) 
					continue;
				// check if y values close enough
				if (Math.abs(top.centroid.y - bottom.centroid.y) < Math.max(topRadius, bottomRadius) || 
						Math.abs(top.centroid.y - bottom.centroid.y) > Math.max(topRadius, bottomRadius)*3) continue;
				// if everything passed this must be a fiducial
				FidPattern fid = new FidPattern(top.color, top.centroid, bottom.color, bottom.centroid);
				answer.add(fid);
				System.out.println("Fiducial top : " + top.color + " bottom : " + bottom.color);
			}
		}
		return answer;
	}
	
	public List<FidPattern> findFids(int[][][] src, boolean[][] mask, int minPixelsPerGroup) {
		List<BlockInfo> binfos = cct.getBlockInfosForFrame(src, mask, minPixelsPerGroup, false);
		return findFids(binfos);
	}
	
}
