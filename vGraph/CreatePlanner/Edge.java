package CreatePlanner;

/**
 * Helper storage class for storing an edge connecting two vertices
 * @author David
 *
 */
public class Edge {
	public Vertex v1, v2; 
	/**
	 * @param args
	 */
	public Edge(Vertex _v1, Vertex _v2) {
		this.v1 = _v1;
		this.v2 = _v2;
	}
	public float getMinX() { 
		return Math.min(v1.x, v2.x);
	}
	public float getMaxX() { 
		return Math.max(v1.x, v2.x);
	}
	public float getMinY() { 
		return Math.min(v1.y, v2.y);
	}
	public float getMaxY() { 
		return Math.max(v1.y, v2.y);		//return Math.min(v1.y, v2.y); debug by joseph
	}
	public String toString() {
		return "("+v1.x+"," + v1.y + ")->(" +v2.x + "," + v2.y + ")";
	}
}
