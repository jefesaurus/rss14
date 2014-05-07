package kinect;

import navigation.Point3D;

public class Pose3D {
  public Point3D point;
  public double yaw, pitch, roll;

  double tMat[][];
  double invTMat[][];
	public Pose3D(Point3D point, double yaw, double pitch, double roll) {
    this.point = point;
		this.yaw = yaw;
		this.pitch = pitch;
		this.roll = roll;
    double cy = Math.cos(roll);
    double sy = Math.sin(roll);
    double ca = Math.cos(yaw);
    double sa = Math.sin(yaw);
    double cb = Math.cos(pitch);
    double sb = Math.sin(pitch);
    this.tMat = new double[][] {{ca*cb, ca*sb*sy-sa*cy, ca*sb*cy+sa*sy, point.x},
                                {sa*cb, sa*sb*sy+ca*cy, sa*sb*cy-ca*sy, point.y},
                                {-sb,   cb*sy,          cb*cy,          point.z}};

    this.invTMat = new double[][] {{ca*cb,          sa*cb,          -sb,   0},
                                   {ca*sb*sy-sa*cy, sa*sb*sy+ca*cy, cb*sy, 0},
                                   {ca*sb*cy+sa*sy, sa*sb*cy-ca*sy, cb*cy, 0}};

    this.invTMat[0][3] = -(invTMat[0][0]*point.x + invTMat[0][1]*point.y + invTMat[0][2]*point.z);
    this.invTMat[1][3]  = -(invTMat[1][0]*point.x + invTMat[1][1]*point.y + invTMat[1][2]*point.z); 
    this.invTMat[2][3]  = -(invTMat[2][0]*point.x + invTMat[2][1]*point.y + invTMat[2][2]*point.z);
    /*
    System.out.println(invTMat[0][3]);
    System.out.println(invTMat[1][3]);
    System.out.println(invTMat[2][3]);
    */
	}

  public Point3D toFrame(Point3D pIn) {
    double nx = tMat[0][0]*pIn.x + tMat[0][1]*pIn.y + tMat[0][2]*pIn.z + tMat[0][3];
    double ny = tMat[1][0]*pIn.x + tMat[1][1]*pIn.y + tMat[1][2]*pIn.z + tMat[1][3];
    double nz = tMat[2][0]*pIn.x + tMat[2][1]*pIn.y + tMat[2][2]*pIn.z + tMat[2][3];
    return new Point3D(nx, ny, nz);
  }

  public Point3D fromFrame(Point3D pIn) {
    double nx = invTMat[0][0]*pIn.x + invTMat[0][1]*pIn.y + invTMat[0][2]*pIn.z + invTMat[0][3];
    double ny = invTMat[1][0]*pIn.x + invTMat[1][1]*pIn.y + invTMat[1][2]*pIn.z + invTMat[1][3];
    double nz = invTMat[2][0]*pIn.x + invTMat[2][1]*pIn.y + invTMat[2][2]*pIn.z + invTMat[2][3];
    return new Point3D(nx, ny, nz);
  }
}
