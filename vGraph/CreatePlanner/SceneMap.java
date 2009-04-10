package CreatePlanner;

import java.util.ArrayList;
import java.util.LinkedList;


/**
 * SceneMap is a storage container class for storing the Obstacles
 * and the calculated visibility path. 
 * 
 * @author David
 *
 */
public class SceneMap { 
	/**
	 * A LinkedList of Obstacle type objects representing the
	 * obstacles in the scene. 
	 */
	LinkedList obstacles; 
	
	ArrayList visibilityPath; 
	
	/**
	 * the max and min values of the obstacles (which include the out boundary).
	 * these values are used strictly by the PathPlannerMainUI in order to properly
	 * center/scale the graphics defined by the SceneMap
	 */
	float minX, minY, maxX, maxY; 
	
	public SceneMap() { 
		obstacles = new LinkedList(); 
	}
	
	/**
	 * Add an obstacle to the SceneMap
	 * @param o
	 */
	public void addObstacle(Obstacle o) {
		obstacles.add(o);
		if (obstacles.size()==0) {
			minX = o.getMinX();
			minY = o.getMinY();
			maxX = o.getMaxX();
			maxX = o.getMaxY();
		}
		else { 
			if (minX>o.getMinX()) {
				minX = o.getMinX();
			}
			if (minY>o.getMinY()) {
				minY = o.getMinY();
			}
			if (maxX<o.getMaxX()) {
				maxX = o.getMaxX();
			}
			if (maxY<o.getMaxY()) {
				maxY = o.getMaxY();
			}
		}
	}
}
