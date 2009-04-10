package CreatePlanner;

public class Vertex {
	public float x, y; 
	public float angle;		// added by joseph
	
	/**
	 * @param args
	 */
	public Vertex(float _x, float _y) {
		this.x = _x;
		this.y = _y;
	}
	
	public Vertex(String _x, String _y) {
		this.x = new Float(_x).floatValue();
		this.y = new Float(_y).floatValue();
	}
	
	public boolean equals(Object o){
		if(((Vertex)o).x == x && ((Vertex)o).y == y) return true;
		return false;
	}
	
	public void print(){
		System.out.println(x + ", " + y);
	}
	
	public double distanceToVertex(Vertex v){
		return Math.sqrt(Math.pow(x - v.x, 2.0)
				+ Math.pow(y - v.y, 2.0));
	}
	
	public double angleFromStart(Vertex v)
	{
		this.angle=(float)Math.atan2(v.y-this.y,this.x-v.x);
		return this.angle;
	}
}
