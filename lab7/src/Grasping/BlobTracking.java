package Grasping;

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
	 * @param src the source RGB image, not packed
	 * @param dest the destination RGB image, packed, may be null
	 */
	public boolean blobPresent(ImageProc src, ImageProc dest) {
		stepTiming(); // monitors the frame rate
		int centroidX = 0;
		int centroidY = 0;
		int area = 0;		
		int w = src.getWidth(); int h = src.getHeight();
		for(int x = 0; x < w; x++) {
			for(int y = 0; y < h; y++) {
				int red = (src.getPixelRed(x, y) >= 0)?src.getPixelRed(x, y):256+src.getPixelRed(x, y);
				int green = (src.getPixelGreen(x, y) >= 0)?src.getPixelGreen(x, y):256+src.getPixelGreen(x, y);
				int blue = (src.getPixelBlue(x, y) >= 0)?src.getPixelBlue(x, y):256+src.getPixelBlue(x, y);

				float[] pixelArray = Color.RGBtoHSB(red, green, blue, null);

				if((pixelArray[0] < .04 || pixelArray[0] > .96) && pixelArray[1] > .5) {
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
	}

        public double RANGE_CONSTANT = 847.9;
        public double BEARING_CONSTANT = 15/2600.0;
	public void blobFix() {
                if (targetDetected) {
                        targetRange = RANGE_CONSTANT/targetRadius;
                        targetBearing = -(targetCentroidX - width/2)*targetRange*BEARING_CONSTANT;
                }
                
        }
}
