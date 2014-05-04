package kinect;

public class RGBDFrame {
  public int width;
  public int height;
  public float r[][];
  public float g[][];
  public float b[][];
  public int d[][];
	public RGBDFrame(float r[][], float g[][], float b[][], int d[][]) {
    this.r = r;
    this.g = g;
    this.b = b;
    this.d = d;
    this.width = r.length;
    this.height = r[0].length;
	}
}
