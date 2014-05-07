package navigation;

public class GridCell {
	public final int x;
	public final int y;
	public final int index;
	public final Point point;
	private double value;
	private long lastUpdate;
	
	public GridCell(int x, int y, int index, Point point) {
		this.x = x;
		this.y = y;
		this.index = index;
		this.point = point;
		this.value = 0.0;
		this.lastUpdate = -1;
	}
	
	public synchronized double getValue() {
		return value;
	}
	
	public synchronized double getLastUpdate() {
		return lastUpdate;
	}
	
	public synchronized void incrementValue() {
		value++;
		lastUpdate = System.currentTimeMillis();
	}
	
	public synchronized void resetValue() {
		value = 0;
		lastUpdate = System.currentTimeMillis();

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
