package cameraProcessing;

import java.awt.Color;
import java.awt.Point;
import java.util.ArrayList;
import java.util.List;

public class ConnCompTwo {
	
	public float blockSatThresh = .6f;
	public float blockBrightThresh = .65f;
	public float hueSimilarityThresh = .05f;
	
	public ConnCompTwo() {
		
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
	 * Uses two pass algorithm from wikipedia page "Connected component labeling"
	 * @param src
	 * @return
	 */
	public List<ArrayList<Point>> findPointGroups(Image src) {
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
		List<ArrayList<Point>> groups = new ArrayList<ArrayList<Point>>(src.getWidth()*src.getHeight());
		
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
	 * 
	 * @param src the source RGB
	 * @param dest the destination RGB
	 */
	public void visualize(Image src, Image dest) {
		List<ArrayList<Point>> groups = filterGroupsBySize(findPointGroups(src), 20);
		System.out.println("Number of groupings: " + groups.size());
		for (ArrayList<Point> g : groups) {
			for (Point p : g) {
				dest.setPixel(p.x, p.y, (byte)50, (byte)50, (byte)50);				
			}
		}
	}
}
