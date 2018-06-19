package roadgraph;

import java.util.Comparator;

public class PQueueForDijkstra implements Comparator<MapNode> {
	
	@Override
	public int compare(MapNode o1, MapNode o2) {
		// TODO Auto-generated method stub
		if(o1.getDistanceOfNode() > o2.getDistanceOfNode()) {
			return 1;
		}
		else if(o1.getDistanceOfNode() < o2.getDistanceOfNode()) {
			return -1;
		}
		else {
			return 0;
		}
		
	}

}
