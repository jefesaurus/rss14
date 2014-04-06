package cameraProcessing;

import java.awt.Color;
import java.awt.Point;
import java.util.ArrayList;
import java.util.List;

public class ConnCompTwo {
	
	public ConnCompTwo() {
		
	}
	
	public void setComparisonFunction() {
		
	}
	
	/**
	 * Judge whether two pixels are similar to each other or not based on passed in parameters
	 * @param pix1
	 * @param pix2
	 * @return
	 */
	private boolean pixelsSimilar(int[] pix1, int[] pix2) {
		return false;
	}
	
	public List<Point> findCentroids(Image src) {
		UF unionFind = new UF(src.getWidth()*src.getHeight());
		
		for (int x = 0; x<src.getWidth(); x++) {
			for (int y = 0; y<src.getHeight(); y++) {
				int red = (src.getPixelRed(x, y) >= 0)?src.getPixelRed(x, y):256+src.getPixelRed(x, y);
				int green = (src.getPixelGreen(x, y) >= 0)?src.getPixelGreen(x, y):256+src.getPixelGreen(x, y);
				int blue = (src.getPixelBlue(x, y) >= 0)?src.getPixelBlue(x, y):256+src.getPixelBlue(x, y);
				float[] pixelArray = Color.RGBtoHSB(red, green, blue, null);
				float hue = pixelArray[0];
				float sat = pixelArray[1];
				float bright = pixelArray[2];
				
				// saturation has to pass a certain value hue and sat value to be considered
				if((hue < .05 || hue > .95) && sat > .5) {
					
				}
			}
		}
		return new ArrayList<Point>();
	}
	
	/**
	 * 
	 * @param src the source RGB
	 * @param dest the destination RGB
	 */
	public void visualize(Image src, Image dest) {
		for (int x = 0; x<src.getWidth(); x++) {
			for (int y = 0; y<src.getHeight(); y++) {
				int red = (src.getPixelRed(x, y) >= 0)?src.getPixelRed(x, y):256+src.getPixelRed(x, y);
				int green = (src.getPixelGreen(x, y) >= 0)?src.getPixelGreen(x, y):256+src.getPixelGreen(x, y);
				int blue = (src.getPixelBlue(x, y) >= 0)?src.getPixelBlue(x, y):256+src.getPixelBlue(x, y);
				float[] pixelArray = Color.RGBtoHSB(red, green, blue, null);
				float hue = pixelArray[0];
				float sat = pixelArray[1];
				float bright = pixelArray[2];
				
				dest.setPixel(x, y, (byte)red, (byte)green, (byte)blue);
			}
		}
	}
}
