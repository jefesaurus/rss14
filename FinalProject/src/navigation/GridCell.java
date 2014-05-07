package navigation;

public class GridCell {
	public final int x;
	public final int y;
	public final int index;
	public final Point point;
	public double value;
	public long lastUpdate = -1;
	
	public GridCell(int x, int y, int index, Point point) {
		this.x = x;
		this.y = y;
		this.index = index;
		this.point = point;
	}

	@Override
	public int hashCode() {
		return index;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		GridCell other = (GridCell) obj;
		return index == other.index;
	}
}
