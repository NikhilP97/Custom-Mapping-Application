package roadgraph;

import java.util.Comparator;

public class PQueueForAStar implements Comparator<MapNode> {
	
	@Override
	public int compare(MapNode o1, MapNode o2) {
		// TODO Auto-generated method stub
		if(o1.getFvalue() > o2.getFvalue()) {
			return 1;
		}
		else if(o1.getFvalue() < o2.getFvalue()) {
			return -1;
		}
		else {
			return 0;
		}
		
	}

}
