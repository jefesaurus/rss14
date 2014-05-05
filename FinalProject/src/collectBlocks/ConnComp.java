package collectBlocks;

import java.awt.Color;
import java.awt.Point;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class ConnComp {
	
	private class HSBRanges {
		public int name;
		public float hueMin;
		public float hueMax;
		public float satMin;
		public float satMax;
		public float brightMin;
		public float brightMax;
		public HSBRanges(int name, float hueMin, float hueMax, float satMin, float satMax, float brightMin, float brightMax) {
			this.name = name;
			this.hueMin = hueMin;
			this.hueMax = hueMax;
			this.satMin = satMin;
			this.satMax = satMax;
			this.brightMin = brightMin;
			this.brightMax = brightMax;
		}
		
		public boolean checkRange(int[][][] src, Point pix) {
			int red = src[pix.x][pix.y][0];
			int green = src[pix.x][pix.y][1];
			int blue = src[pix.x][pix.y][2];
			float[] pixelArray = Color.RGBtoHSB(red, green, blue, null);
			float hue = pixelArray[0];
			float sat = pixelArray[1];
			float bright = pixelArray[2];
			if (hue < hueMin || hue > hueMax) return false;
			if (sat < satMin || sat > satMax) return false;
			if (bright < brightMin || bright > brightMax) return false;
			return true;
		}
	}
	
	public HSBRanges yellowRange = new HSBRanges(0, .1f, .2f, .4f, 1.f, .4f, 1.f);
	public HSBRanges redRange = new HSBRanges(1, .0f, .05f, .5f, 1.f, .5f, 1.f);
	public HSBRanges greenRange = new HSBRanges(2, .35f, .45f, .6f, 1.f, .3f, 1.f);
	public HSBRanges blueRange = new HSBRanges(3, .5f, .7f, .4f, 1.f, .2f, .4f);
	public HSBRanges orangeRange = new HSBRanges(4, .05f, .08f, .5f, 1.f, .4f, 1.f);
	public HSBRanges[] hsbRanges = {yellowRange, redRange, greenRange};//, blueRange};//, orangeRange};
	public String[] RangeMapping = {"yellow", "red", "green", "blue", "orange"}; 
	
	public float blockSatThresh = .6f;
	public float blockBrightThresh = .35f;
	public float hueSimilarityThresh = .05f;
	public int minPixelsPerGroup = 70;
	int[][] colors;
	
	public ConnComp() {
		
	}
	
	/**
	 * 
	 * @param pix
	 * @return Return a subset of the 4 neighboring pixels with the same color labeled
	 */
	private List<Point> colorNeighbors(int[][] colorLabels, Point pix) {
		int x = pix.x;
		int y = pix.y;
		Point[] checkList4 = {new Point(x+1,y), new Point(x-1,y), new Point(x,y+1), new Point(x, y-1)};
		Point[] checkList8 = {new Point(x+1,y), new Point(x-1,y), new Point(x,y+1), new Point(x, y-1),
				new Point (x+1,y+1), new Point(x-1,y+1), new Point(x+1,y-1), new Point(x-1,y-1)};
		List<Point> answer = new ArrayList<Point>();
		for (Point neigh : checkList4) {
			if (neigh.x < colorLabels.length && neigh.x >= 0 && neigh.y < colorLabels[0].length && neigh.y >= 0){
				if (colorLabels[neigh.x][neigh.y] == -1) continue; //ignore background
				if (colorLabels[neigh.x][neigh.y] == colorLabels[pix.x][pix.y]) {
					answer.add(neigh);
				}
			}
		}
		return answer;
	}
	
	/**
	 * @param src
	 * @return
	 */
	public List<ArrayList<Point>> findColorPointGroups(int[][][] src) {
		UF labels = new UF(src.length*src[0].length);
		colors = new int[src.length][src[0].length];
		
		// First pass, determine color of pixel
		for (int x = 0; x<src.length; x++) {
			for (int y = 0; y<src[0].length; y++) {
				boolean background = true;
				for (HSBRanges range : this.hsbRanges) {
					if (range.checkRange(src, new Point(x,y))) {
						colors[x][y] = range.name;
						background = false;
					}
				}
				if (background) {
					colors[x][y] = -1;
				}
			}
		}
		// Second pass, label everything
		for (int x = 0; x<src.length; x++) {
			for (int y = 0; y<src[0].length; y++) {
				Point pix = new Point(x,y);
				//ignore background pixels
				if (colors[x][y] == -1) {
					continue;
				}
				
				// look at neighbors with similar colors. If it has no neighbors, ignore it as an anomaly.
				List<Point> neighbors = colorNeighbors(colors, pix);
				if (neighbors.isEmpty())
					continue;
				
				// otherwise blindly union them all
				for (Point neigh : neighbors) {
					labels.union(pix.x*src[0].length+pix.y, neigh.x*src[0].length+neigh.y);
				}
			}
		}
		// Third pass, extract groups with size greater than 1
		List<ArrayList<Point>> groups = new ArrayList<ArrayList<Point>>();
		for (int i = 0; i<src.length*src[0].length; i++) {
			groups.add(new ArrayList<Point>());
		}
		//System.out.println("size: " + groups.size());
		for (int x = 0; x<src.length; x++) {
			for (int y = 0; y<src[0].length; y++) {
				Point pix = new Point(x,y);
				int component = labels.find(pix.x*src[0].length+pix.y);
				groups.get(component).add(pix);
			}
		}
		return groups;
	}
	
	public List<ArrayList<Point>> filterGroupsBySize(List<ArrayList<Point>> groups, int minNumOfMembers) {
		List<ArrayList<Point>> answer = new ArrayList<ArrayList<Point>>();
		for (ArrayList<Point> g : groups) {
			if (g.size() > minNumOfMembers) {
				answer.add(g);
			}
		}
		return answer;
	}
	
	/**
	 * Collects a list of point groups into Block infos. The size of the block is estimated based on the min and max values
	 * for x and y for that group. Sorted by largest area group first.
	 * @param groups
	 * @return
	 */
	public List<BlockInfo> makeBlockInfos(int[][][] src, List<ArrayList<Point>> groups) {
		List<BlockInfo> answer = new ArrayList<BlockInfo>();
		System.out.println("Number of groupings: " + groups.size());
		for (ArrayList<Point> g : groups) {
			if (g.size() <1) {
				answer.add(new BlockInfo(new Point(0,0), 1, "invalid"));
				continue;
			}
			int allX = 0;
			int allY = 0;
			int minx = 10000;
			int miny = 10000;
			int maxx = -50;
			int maxy = -50;
			for (Point p : g) {
				allX += p.x;
				allY += p.y;
				if (p.x < minx) minx = p.x;
				if (p.y < miny) miny = p.y;
				if (p.x > maxx) maxx = p.x;
				if (p.y > maxy) maxy = p.y;
			}
			int avgX = allX / g.size();
			int avgY = allY / g.size();
			// estimate size based on min and max values for x and y
			int size = (maxx-minx)*(maxy-miny);
			//re-derive color
			int color = -1;
			for (HSBRanges range : this.hsbRanges) {
				if (range.checkRange(src, new Point(g.get(0).x, g.get(0).y))) {
					color = range.name;
				}
			}
			color = colors[g.get(0).x][g.get(0).y];
			String colorName = (color >=0)?RangeMapping[color]:"background";
			BlockInfo bi = new BlockInfo(new Point(avgX, avgY), size, colorName);
			answer.add(bi);
//			System.out.println("x : " + bi.centroid.x + "| y : " + bi.centroid.y + " | color : " + bi.color);
		}
		Collections.sort(answer);
		return answer;
	}
	
	/**
	 * 
	 * @param src the source RGB
	 * @param dest the destination RGB
	 */
	public List<BlockInfo> visualize(int[][][] src, Image dest) {
		Color[] groupColorings = {Color.white, Color.blue, Color.pink, Color.magenta, Color.yellow, Color.green};
		List<ArrayList<Point>> groups = filterGroupsBySize(findColorPointGroups(src), minPixelsPerGroup);
		List<BlockInfo> blockInfos = makeBlockInfos(src, groups);
		System.out.println("Number of groupings: " + groups.size());
		for (int i = 0; i < groups.size(); i++) {
			ArrayList<Point> g = groups.get(i);
			for (Point p : g) {
				Color c = groupColorings[i % groupColorings.length];
				dest.setPixel(p.x, p.y, (byte)c.getRed(), (byte)c.getGreen(), (byte)c.getBlue());				
			}
			BlockInfo bi = blockInfos.get(i);
			Point centroid = bi.centroid;
			System.out.println("x : " + centroid.x + "| y : " + centroid.y + " | color : " + bi.color);
			for(int x =  centroid.x- 4; x <= centroid.x + 4; x++)
				if(x >= 0 && x < dest.getWidth() && centroid.y >= 0 && centroid.y < dest.getHeight())
					dest.setPixel(x, centroid.y, (byte)0, (byte)0, (byte)0);				
			for(int y =  centroid.y- 4; y <= centroid.y + 4; y++)
				if(y >= 0 && y < dest.getHeight() && centroid.x >= 0 && centroid.x < dest.getWidth())				
					dest.setPixel(centroid.x, y, (byte)0, (byte)0, (byte)0);	
		}
		return blockInfos;
	}
	
	/**
	 * Returns the same thing as visualize without the visualization.
	 * @param src
	 * @return
	 */
	public List<BlockInfo> getBlockInfosForFrame(int[][][] src) {
		List<ArrayList<Point>> groups = filterGroupsBySize(findColorPointGroups(src), minPixelsPerGroup);
		return makeBlockInfos(src, groups);
	}
	
	/**
	 * Simple function that takes the average of the middle fifth of the screen. The area is marked with a black outline
	 * @param src
	 * @param dest
	 */
	public void calibrateHelp(int[][][] src, Image dest) {
		float avgHue = 0;
		float avgSat = 0;
		float avgBright = 0;
		for (int x = 2*src.length/5; x<3*src.length/5; x++) {
			for (int y = 2*src[0].length/5; y<3*src[0].length/5; y++) {
				int red = src[x][y][0];
				int green = src[x][y][1];
				int blue = src[x][y][2];
				float[] pixelArray = Color.RGBtoHSB(red, green, blue, null);
				float hue = pixelArray[0];	float sat = pixelArray[1];	float bright = pixelArray[2];
				avgHue += hue;
				avgSat += sat;
				avgBright += bright;
			}
		}
		int totalPixels = src.length*src[0].length/25;
		avgHue = avgHue/totalPixels;
		avgSat /= totalPixels;
		avgBright /= totalPixels;
		System.out.println(String.format("AvgHue: %.5f | AvgSat: %.5f | avgBright: %.5f", avgHue, avgSat, avgBright));
		
		// draw black box around pixels we are checking
		for (int x = 2*src.length/5; x<3*src.length/5; x++) {
			dest.setPixel(x, 2*src[0].length/5, (byte)0, (byte)0, (byte)0);
			dest.setPixel(x, 3*src[0].length/5, (byte)0, (byte)0, (byte)0);
		}
		for (int y = 2*src[0].length/5; y<3*src[0].length/5; y++) {
			dest.setPixel(2*src.length/5, y, (byte)0, (byte)0, (byte)0);
			dest.setPixel(3*src.length/5, y, (byte)0, (byte)0, (byte)0);
		}
	}
	
	/**
	 * Function used to help debug certain things
	 * @param src
	 * @param dest
	 */
	public void debugHelp(Image src, Image dest) {
		for (int x = 0; x<src.getWidth(); x++) {
			byte grey = (byte)(256*x/src.getWidth());
			dest.setPixel(x, src.getHeight()/2, grey, grey, grey);
		}
	}
}
