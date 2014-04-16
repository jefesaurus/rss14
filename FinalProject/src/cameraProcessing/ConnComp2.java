package cameraProcessing;

import java.awt.Color;
import java.awt.Point;
import java.util.ArrayList;
import java.util.List;

public class ConnComp2 {
	
	public class BlockInfo {
		public Point centroid;
		public float size;
		public String color;
		
		public BlockInfo(Point centroid, float size, String color) {
			this.centroid = centroid;
			this.size = size;
			this.color = color;
		}
	}
	
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
		
		public boolean checkRange(Image src, Point pix) {
			int red = (src.getPixelRed(pix.x, pix.y) >= 0)?src.getPixelRed(pix.x, pix.y):256+src.getPixelRed(pix.x, pix.y);
			int green = (src.getPixelGreen(pix.x, pix.y) >= 0)?src.getPixelGreen(pix.x, pix.y):256+src.getPixelGreen(pix.x, pix.y);
			int blue = (src.getPixelBlue(pix.x, pix.y) >= 0)?src.getPixelBlue(pix.x, pix.y):256+src.getPixelBlue(pix.x, pix.y);
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
	
	public HSBRanges yellowRange = new HSBRanges(0, .1f, .2f, .6f, 1.f, .4f, 1.f);
	public HSBRanges redRange = new HSBRanges(1, .0f, .07f, .5f, 1.f, .6f, 1.f);
	public HSBRanges greenRange = new HSBRanges(2, .35f, .45f, .6f, 1.f, .3f, 1.f);
	public HSBRanges[] hsbRanges = {yellowRange, redRange, greenRange};
	public String[] RangeMapping = {"yellow", "red", "green"}; 
	
	public float blockSatThresh = .6f;
	public float blockBrightThresh = .35f;
	public float hueSimilarityThresh = .05f;
	public int minPixelsPerGroup = 40;
	int[][] colors;
	
	public ConnComp2() {
		
	}
	
	/**
	 * Judge whether two pixels are similar to each other or not based on passed in parameters
	 * If either pixel value is invalid (outside the bound of the image), return false
	 * @param pix1
	 * @param pix2
	 * @return
	 */
	private boolean pixelsSimilar(Image src, Point pix1, Point pix2) {
		int red1 = (src.getPixelRed(pix1.x, pix1.y) >= 0)?src.getPixelRed(pix1.x, pix1.y):256+src.getPixelRed(pix1.x, pix1.y);
		int green1 = (src.getPixelGreen(pix1.x, pix1.y) >= 0)?src.getPixelGreen(pix1.x, pix1.y):256+src.getPixelGreen(pix1.x, pix1.y);
		int blue1 = (src.getPixelBlue(pix1.x, pix1.y) >= 0)?src.getPixelBlue(pix1.x, pix1.y):256+src.getPixelBlue(pix1.x, pix1.y);
		float[] pixelArray1 = Color.RGBtoHSB(red1, green1, blue1, null);
		float hue1 = pixelArray1[0];
		float sat1 = pixelArray1[1];
		float bright1 = pixelArray1[2];
		
		int red2 = (src.getPixelRed(pix2.x, pix2.y) >= 0)?src.getPixelRed(pix2.x, pix2.y):256+src.getPixelRed(pix2.x, pix2.y);
		int green2 = (src.getPixelGreen(pix2.x, pix2.y) >= 0)?src.getPixelGreen(pix2.x, pix2.y):256+src.getPixelGreen(pix2.x, pix2.y);
		int blue2 = (src.getPixelBlue(pix2.x, pix2.y) >= 0)?src.getPixelBlue(pix2.x, pix2.y):256+src.getPixelBlue(pix2.x, pix2.y);
		float[] pixelArray2 = Color.RGBtoHSB(red2, green2, blue2, null);
		float hue2 = pixelArray2[0];
		float sat2 = pixelArray2[1];
		float bright2 = pixelArray2[2];
		
		// If either pixel is not a block pixel, return false
		if (!(isBlockPixel(src, pix1) && isBlockPixel(src, pix2)))
			return false;
		// If hue value close to original pixel, considered valid
		float hueDiff = Math.abs(hue1 - hue2);
		hueDiff = (hueDiff > 1)?hueDiff - 1 : hueDiff;
		if (hueDiff < hueSimilarityThresh)
			return true;
		// Otherwise return false
		return false;
	}
	
	/**
	 * Measure of whether or not a pixel is considered to be part of a block
	 * @param src
	 * @param pix
	 * @return
	 */
	private boolean isBlockPixel(Image src, Point pix) {
		int red = (src.getPixelRed(pix.x, pix.y) >= 0)?src.getPixelRed(pix.x, pix.y):256+src.getPixelRed(pix.x, pix.y);
		int green = (src.getPixelGreen(pix.x, pix.y) >= 0)?src.getPixelGreen(pix.x, pix.y):256+src.getPixelGreen(pix.x, pix.y);
		int blue = (src.getPixelBlue(pix.x, pix.y) >= 0)?src.getPixelBlue(pix.x, pix.y):256+src.getPixelBlue(pix.x, pix.y);
		float[] pixelArray = Color.RGBtoHSB(red, green, blue, null);
		float hue = pixelArray[0];
		float sat = pixelArray[1];
		float bright = pixelArray[2];
		return (bright > blockBrightThresh && sat > blockSatThresh);
	}
	
	/**
	 * 
	 * @param pix
	 * @return Return a subset of the 4 neighboring pixels with similar pixels
	 */
	private List<Point> similarNeighbors(Image src, Point pix) {
		int x = pix.x;
		int y = pix.y;
		Point[] checkList4 = {new Point(x+1,y), new Point(x-1,y), new Point(x,y+1), new Point(x, y-1)};
		Point[] checkList8 = {new Point(x+1,y), new Point(x-1,y), new Point(x,y+1), new Point(x, y-1),
				new Point (x+1,y+1), new Point(x-1,y+1), new Point(x+1,y-1), new Point(x-1,y-1)};
		List<Point> answer = new ArrayList<Point>();
		for (Point neigh : checkList4) {
			if (pixelsSimilar(src, pix, neigh)) {
				answer.add(neigh);
			}
		}
		return answer;
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
	 * Uses two pass algorithm from wikipedia page "Connected component labeling". Groups things that have similar
	 * neighboring pixels.
	 * @param src
	 * @return
	 */
	public List<ArrayList<Point>> findSimilarPointGroups(Image src) {
		UF labels = new UF(src.getWidth()*src.getHeight());
		// First pass, label everything
		for (int x = 0; x<src.getWidth(); x++) {
			for (int y = 0; y<src.getHeight(); y++) {
				Point pix = new Point(x,y);
				if (!isBlockPixel(src, pix)) {
					continue;
				}
				
				// look at neighbors with similar colors. If it has no neighbors, ignore it as an anomaly.
				List<Point> neighbors = similarNeighbors(src, pix);
				if (neighbors.isEmpty())
					continue;
				
				// otherwise blindly union them all
				for (Point neigh : neighbors) {
					labels.union(pix.y*src.getHeight()+pix.x, neigh.y*src.getHeight()+neigh.x);
				}
			}
		}
		// second pass, extract groups with size greater than 1
		List<ArrayList<Point>> groups = new ArrayList<ArrayList<Point>>();
		for (int i = 0; i<src.getWidth()*src.getHeight(); i++) {
			groups.add(new ArrayList<Point>());
		}
		System.out.println("size: " + groups.size());
		
		for (int x = 0; x<src.getWidth(); x++) {
			for (int y = 0; y<src.getHeight(); y++) {
				Point pix = new Point(x,y);
				int component = labels.find(pix.y*src.getHeight()+pix.x);
				groups.get(component).add(pix);
			}
		}
		return groups;
	}
	
	/**
	 * @param src
	 * @return
	 */
	public List<ArrayList<Point>> findColorPointGroups(Image src) {
		UF labels = new UF(src.getWidth()*src.getHeight());
		colors = new int[src.getWidth()][src.getHeight()];
		
		// First pass, determine color of pixel
		for (int x = 0; x<src.getWidth(); x++) {
			for (int y = 0; y<src.getHeight(); y++) {
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
		for (int x = 0; x<src.getWidth(); x++) {
			for (int y = 0; y<src.getHeight(); y++) {
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
					labels.union(pix.y*src.getHeight()+pix.x, neigh.y*src.getHeight()+neigh.x);
				}
			}
		}
		// Third pass, extract groups with size greater than 1
		List<ArrayList<Point>> groups = new ArrayList<ArrayList<Point>>();
		for (int i = 0; i<src.getWidth()*src.getHeight(); i++) {
			groups.add(new ArrayList<Point>());
		}
		System.out.println("size: " + groups.size());
		
		for (int x = 0; x<src.getWidth(); x++) {
			for (int y = 0; y<src.getHeight(); y++) {
				Point pix = new Point(x,y);
				int component = labels.find(pix.y*src.getHeight()+pix.x);
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
	 * for x and y for that group.
	 * @param groups
	 * @return
	 */
	public List<BlockInfo> makeBlockInfos(Image src, List<ArrayList<Point>> groups) {
		List<BlockInfo> answer = new ArrayList<BlockInfo>();
		for (ArrayList<Point> g : groups) {
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
			System.out.println(color);
			String colorName = (color >=0)?RangeMapping[color]:"background";
			answer.add(new BlockInfo(new Point(avgX, avgY), size, colorName));
		}
		return answer;
	}
	
	/**
	 * 
	 * @param src the source RGB
	 * @param dest the destination RGB
	 */
	public void visualize(Image src, Image dest) {
		Color[] groupColorings = {Color.black, Color.blue, Color.pink, Color.magenta, Color.yellow, Color.green};
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
	}
	
	/**
	 * Simple function that takes the average of the middle fifth of the screen. The area is marked with a black outline
	 * @param src
	 * @param dest
	 */
	public void calibrateHelp(Image src, Image dest) {
		float avgHue = 0;
		float avgSat = 0;
		float avgBright = 0;
		for (int x = 2*src.getWidth()/5; x<3*src.getWidth()/5; x++) {
			for (int y = 2*src.getHeight()/5; y<3*src.getHeight()/5; y++) {
				int red = (src.getPixelRed(x, y) >= 0)?src.getPixelRed(x, y):256+src.getPixelRed(x, y);
				int green = (src.getPixelGreen(x, y) >= 0)?src.getPixelGreen(x, y):256+src.getPixelGreen(x, y);
				int blue = (src.getPixelBlue(x, y) >= 0)?src.getPixelBlue(x, y):256+src.getPixelBlue(x, y);
				float[] pixelArray = Color.RGBtoHSB(red, green, blue, null);
				float hue = pixelArray[0];	float sat = pixelArray[1];	float bright = pixelArray[2];
				avgHue += hue;
				avgSat += sat;
				avgBright += bright;
			}
		}
		int totalPixels = src.getWidth()*src.getHeight()/25;
		avgHue = avgHue/totalPixels;
		avgSat /= totalPixels;
		avgBright /= totalPixels;
		System.out.println(String.format("AvgHue: %.5f | AvgSat: %.5f | avgBright: %.5f", avgHue, avgSat, avgBright));
		
		// draw black box around pixels we are checking
		for (int x = 2*src.getWidth()/5; x<3*src.getWidth()/5; x++) {
			dest.setPixel(x, 2*src.getHeight()/5, (byte)0, (byte)0, (byte)0);
			dest.setPixel(x, 3*src.getHeight()/5, (byte)0, (byte)0, (byte)0);
		}
		for (int y = 2*src.getHeight()/5; y<3*src.getHeight()/5; y++) {
			dest.setPixel(2*src.getWidth()/5, y, (byte)0, (byte)0, (byte)0);
			dest.setPixel(3*src.getWidth()/5, y, (byte)0, (byte)0, (byte)0);
		}
	}
}
