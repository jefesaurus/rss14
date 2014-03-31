package VisualServo;

import java.awt.Color;

/**
 * BlobTracking performs image processing and tracking for the VisualServo
 * module.  BlobTracking filters raw pixels from an image and classifies blobs,
 * generating a higher-level vision image.
 *
 * @author previous TA's, prentice
 */
public class BlobTracking {
	protected int stepCounter = 0;
	protected double lastStepTime = 0.0;

	public int width;
	public int height;


	// Variables used for velocity controller that are available to calling
	// process.  Visual results are valid only if targetDetected==true; motor
	// velocities should do something sane in this case.
	public boolean targetDetected = false; // set in blobPresent()
	public double targetCentroidX = 0.0; // set in blobPresent()
	public double targetCentroidY = 0.0; // set in blobPresent()
	public double targetArea = 0.0; // set in blobPresent()
	public double targetRadius = 0.0; // set in blobPresent()
	public double targetRange = 0.0; // set in blobFix()
	public double targetBearing = 0.0; // set in blobFix()

	/**
	 * <p>Create a BlobTracking object</p>
	 *
	 * @param width image width
	 * @param height image height
	 */
	public BlobTracking(int width, int height) {

		this.width = width;
		this.height = height;

	}





	/**
	 * <p>Computes frame rate of vision processing</p>
	 */
	private void stepTiming() {
		double currTime = System.currentTimeMillis();
		stepCounter++;
		// if it's been a second, compute frames-per-second
		if (currTime - lastStepTime > 1000.0) {
			//double fps = (double) stepCounter * 1000.0
			// / (currTime - lastStepTime);
			//System.err.println("FPS: " + fps);
			stepCounter = 0;
			lastStepTime = currTime;
		}
	}

	/**
	 * <p>Segment out a blob from the src image (if a good candidate exists).</p>
	 *
	 * <p><code>dest</code> is a packed RGB image for a java image drawing
	 * routine. If it's not null, the blob is highlighted.</p>
	 *
	 * @param src the source RGB image, not packed
	 * @param dest the destination RGB image, packed, may be null
	 */
	public void apply(Image src, Image dest) {

		stepTiming(); // monitors the frame rate

		// Begin Student Code
		
		int w = src.getWidth(); int h = src.getHeight();

		//RGB
		/*double averageRed = 0;
		double averageBlue = 0;
		double averageGreen = 0;
		for(int x = w/3; x < 2*w/3; x++) {
			for(int y = h/3; y < 2*h/3; y++) {
				int red = (src.getPixelRed(x, y) >= 0)?src.getPixelRed(x, y):256+src.getPixelRed(x, y);
				int green = (src.getPixelGreen(x, y) >= 0)?src.getPixelGreen(x, y):256+src.getPixelGreen(x, y);
				int blue = (src.getPixelBlue(x, y) >= 0)?src.getPixelBlue(x, y):256+src.getPixelBlue(x, y);
				averageRed += red;
				averageGreen += green;
				averageBlue += blue;
			}
		}
		averageRed /= (w*h/9);
		averageBlue /= (w*h/9);
		averageGreen /= (w*h/9);

		System.out.println("Average Red: " + averageRed + " | Average Green: " + averageGreen + " | Average Blue: " + averageBlue);
		*/

		//HSB
		/*double averageHue = 0;
		double averageSaturation = 0;
		double averageBrightness = 0;
		for(int x = w/3; x < 2*w/3; x++) {
			for(int y = h/3; y < 2*h/3; y++) {
				int red = (src.getPixelRed(x, y) >= 0)?src.getPixelRed(x, y):256+src.getPixelRed(x, y);
				int green = (src.getPixelGreen(x, y) >= 0)?src.getPixelGreen(x, y):256+src.getPixelGreen(x, y);
				int blue = (src.getPixelBlue(x, y) >= 0)?src.getPixelBlue(x, y):256+src.getPixelBlue(x, y);

				float[] pixelArray = Color.RGBtoHSB(red,
								    green,
					     			    blue,
					     			    null);

				averageHue+= pixelArray[0];
				averageSaturation += pixelArray[1];
				averageBrightness += pixelArray[2];
			}
		}
		averageHue /= (w*h/9);
		averageSaturation /= (w*h/9);
		averageBrightness /= (w*h/9);

		System.out.println("Average Hue: " + averageHue + " | Average Saturation: " + averageSaturation + " | Average Brightness: " + averageBrightness);*/
		dest = Histogram.getHistogram(src, dest, false);

		// visualization
		/*		
		for(int x = 0; x < w; x++) {
			for(int y = 0; y < h; y++) {
				int red = (src.getPixelRed(x, y) >= 0)?src.getPixelRed(x, y):256+src.getPixelRed(x, y);
				int green = (src.getPixelGreen(x, y) >= 0)?src.getPixelGreen(x, y):256+src.getPixelGreen(x, y);
				int blue = (src.getPixelBlue(x, y) >= 0)?src.getPixelBlue(x, y):256+src.getPixelBlue(x, y);

				float[] pixelArray = Color.RGBtoHSB(red,
								    green,
					     			    blue,
					     			    null);

				if((pixelArray[0] < .05 || pixelArray[0] > .95) && pixelArray[1] > .5) {
					red = 255;
					green = 0;
					blue = 0;
				} else {
				}
				dest.setPixel(x, y, (byte)red, (byte)green, (byte)blue);
			}
		}*/


		// End Student Code
	}

	/**
	 * @param src the source RGB image, not packed
	 * @param dest the destination RGB image, packed, may be null
	 */
	public boolean blobPresent(Image src, Image dest) {

		stepTiming(); // monitors the frame rate

		// Begin Student Code

		int centroidX = 0;
		int centroidY = 0;
		int area = 0;		
		int w = src.getWidth(); int h = src.getHeight();
		for(int x = 0; x < w; x++) {
			for(int y = 0; y < h; y++) {
				int red = (src.getPixelRed(x, y) >= 0)?src.getPixelRed(x, y):256+src.getPixelRed(x, y);
				int green = (src.getPixelGreen(x, y) >= 0)?src.getPixelGreen(x, y):256+src.getPixelGreen(x, y);
				int blue = (src.getPixelBlue(x, y) >= 0)?src.getPixelBlue(x, y):256+src.getPixelBlue(x, y);

				float[] pixelArray = Color.RGBtoHSB(red,
								    green,
					     			    blue,
					     			    null);

				if((pixelArray[0] < .05 || pixelArray[0] > .95) && pixelArray[1] > .7) {
					centroidX += x;
					centroidY += y;
					area++;
					red = 255;
					green = 0;
					blue = 0;
				} else {
				}
				dest.setPixel(x, y, (byte)red, (byte)green, (byte)blue);
			}
		}

		targetDetected = area > .01*w*h;
		
		if(targetDetected) {
			centroidX/=area;
			centroidY/=area;
		        targetRadius = Math.sqrt(area/Math.PI);
                        targetCentroidX = centroidX;
                        targetCentroidY = centroidY;
                        blobFix();

			System.out.println("Centroid X: " + targetCentroidX + " | Centroid Y: " + targetCentroidY + " | Area: " + area + " | Radius: " + targetRadius + " | Distance: " + targetRange);
			for(int x =  centroidX- 4; x <= centroidX + 4; x++)
				if(x >= 0 && x < w && centroidY >= 0 && centroidY < h)
					dest.setPixel(x, centroidY, (byte)0, (byte)0, (byte)0);				
			for(int y =  centroidY- 4; y <= centroidY + 4; y++)
				if(y >= 0 && y < h && centroidX >= 0 && centroidX < w)				
					dest.setPixel(centroidX, y, (byte)0, (byte)0, (byte)0);				
		} else {		
			System.out.println("Not present | Area: " + area);
		}

		return targetDetected;
		// End Student Code
	}

        public double RANGE_CONSTANT = 847.9;
        public double BEARING_CONSTANT = 15/2600.0;
	public void blobFix() {
        /*
	public boolean targetDetected = false; // set in blobPresent()
	public double centroidX = 0.0; // set in blobPresent()
	public double centroidY = 0.0; // set in blobPresent()
	public double targetArea = 0.0; // set in blobPresent()
	public double targetRange = 0.0; // set in blobFix()
	public double targetBearing = 0.0; // set in blobFix()
        */
        
                if (targetDetected) {
                        targetRange = RANGE_CONSTANT/targetRadius;
                        targetBearing = -(targetCentroidX - width/2)*targetRange*BEARING_CONSTANT;
                }
                
        }
}
