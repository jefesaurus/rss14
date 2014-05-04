package KinectTest;

public class IntTuple {
  public int x;
  public int y;
  public IntTuple(int x, int y) {
    this.x = x;
    this.y = y;
  }

  @Override
  public boolean equals(Object o) {
      if (this == o) return true;
      if (o == null || getClass() != o.getClass()) return false;
      return this.x == ((IntTuple)o).x && this.y == ((IntTuple)o).y;
  }

  @Override
  public int hashCode() {
    return ((23*31) + x)*31 + y;
  }

  @Override
  public String toString() {
      return "(" + this.x + "," + this.y + ")";
  }
}
