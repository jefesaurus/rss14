package navigation;

import java.util.List;

public interface Goal {
	public List<Configuration> goalConfigurations();
	public double goalDistance(Configuration config);
}
