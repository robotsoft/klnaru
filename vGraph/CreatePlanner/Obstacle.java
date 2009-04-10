package CreatePlanner;

import java.util.LinkedList;
import java.util.ArrayList;

/**
 * Helper class for storing an Obstacle consisting of a LinkedList of edges
 * @author David
 *
 */
public class Obstacle {
	LinkedList edges; 
	
	//contains the bounds of the obstacle, updated as edges are added
	float minX,minY,maxX,maxY; 
	
	/**
	 * @param args
	 */
	public Obstacle() {
		edges = new LinkedList();
	}
	public void addEdge(Edge e) {
		edges.add(e);
		//if this is the first edge being added, then this will be all extremes
		if (edges.size()==1) {		//debug edges.size()==1 by joseph
			minX = e.getMinX();
			minY = e.getMinY();
			maxX = e.getMaxX();
			maxY = e.getMaxY();		//debug maxX = e.getMaxY() by joseph
		}
		else { 
			if (minX>e.getMinX()) {
				minX = e.getMinX();
			}
			if (minY>e.getMinY()) {
				minY = e.getMinY();
			}
			if (maxX<e.getMaxX()) {
				maxX = e.getMaxX();
			}
			if (maxY<e.getMaxY()) {
				maxY = e.getMaxY();
			}
		}
	}
	public Edge[] getEdgeArray() { 
		return (Edge[]) edges.toArray(new Edge[0]);
	}
	public float getMinX() { 
		return this.minX;
	}
	public float getMinY() { 
		return this.minY;
	}
	public float getMaxX() {
		return this.maxX;
	}
	public float getMaxY() { 
		return this.maxY;
	}
	
	public ArrayList listVertices(){
		ArrayList vertices = new ArrayList();
		for(int i = 0; i < edges.size(); i++){
			vertices.add(((Edge)edges.get(i)).v1);
		}
		return vertices;
	}
	
	public void updateMinMax(){
		this.minX=Integer.MAX_VALUE;
		this.maxX=Integer.MIN_VALUE;
		this.minY=Integer.MAX_VALUE;
		this.maxY=Integer.MIN_VALUE;
		for(int i = 0; i < edges.size(); i++){
			if(((Edge)edges.get(i)).v1.x<=minX)
				this.minX=((Edge)edges.get(i)).v1.x;
			if(((Edge)edges.get(i)).v1.x>=maxX)
				this.maxX=((Edge)edges.get(i)).v1.x;
			if(((Edge)edges.get(i)).v1.y<=minY)
				this.minY=((Edge)edges.get(i)).v1.y;
			if(((Edge)edges.get(i)).v1.y>=maxY)
				this.maxY=((Edge)edges.get(i)).v1.y;
		}
	}
}
