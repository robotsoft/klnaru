/**
 * 
 */
package CreatePlanner;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.StringTokenizer;

/**
 * @author joseph
 *
 */
public class RapidlyExploringRandomTree {

	/**
	 * Original obstacles specified in the source file
	 */
	public ArrayList origObstacles;

	/**
	 * Tree
	 */
	public ArrayList<Edge> rrtEdge;

	/**
	 * Shortest path computed from Visibility graph
	 */
	public ArrayList<Edge> shortestPath;

	/**
	 * Starting point for robot
	 */
	public Vertex start;

	/**
	 * Goal of robot
	 */
	public Vertex goal;
	
	public void RapidlyExploringRandomTree(){
		
	}
	
	public ArrayList getObstacles() {
		return origObstacles;
	}
	
	/**
	 * Read in the start and goal vertices from the specified file.
	 * @param filename
	 */
	public void readStartAndGoal(String filename) {
		File inputFile;
		BufferedReader br;
		StringTokenizer st;

		try {
			inputFile = new File(filename);
			br = new BufferedReader(new FileReader(inputFile));

			st = new StringTokenizer(br.readLine());
			start = new Vertex(new Float(st.nextToken()).floatValue(),
					new Float(st.nextToken()).floatValue());

			st = new StringTokenizer(br.readLine());
			goal = new Vertex(new Float(st.nextToken()).floatValue(),
					new Float(st.nextToken()).floatValue());

			br.close();

		} catch (Exception e) {
			System.out.println("error: " + e.getMessage());
		}
	}
	
	/**
	 * Reads the obstacles from the specified text file
	 * @param filename
	 * @return
	 */
	public ArrayList readObstacles(String filename) {
		File inputFile;
		BufferedReader br;
		StringTokenizer st;
		ArrayList obstacles = new ArrayList();
		ArrayList vertices;
		int numObstacles, numVertices;
		Obstacle o;
		Vertex v;

		//Add virtual obstacle for boundary
		o=new Obstacle();
		o.addEdge(new Edge(new Vertex(0,0),new Vertex(600,0)));
		o.addEdge(new Edge(new Vertex(600,0),new Vertex(600,600)));
		o.addEdge(new Edge(new Vertex(600,600),new Vertex(0,600)));
		o.addEdge(new Edge(new Vertex(0,600),new Vertex(0,0)));
		obstacles.add(o);
		
		try {
			inputFile = new File(filename);
			br = new BufferedReader(new FileReader(inputFile));

			numObstacles = new Integer(br.readLine()).intValue();
			obstacles.ensureCapacity(numObstacles);

			for (int i = 0; i < numObstacles; i++) {
				o = new Obstacle();
				vertices = new ArrayList();

				numVertices = new Integer(br.readLine()).intValue();

				for (int j = 0; j < numVertices; j++) {
					st = new StringTokenizer(br.readLine());
					v = new Vertex(new Float(st.nextToken()).floatValue(),
							new Float(st.nextToken()).floatValue());
					vertices.add(v);
				}

				for (int j = 0; j < vertices.size(); j++) {
					if (j == vertices.size() - 1)
						o.addEdge(new Edge((Vertex) vertices.get(j),
								(Vertex) vertices.get(0)));
					else
						o.addEdge(new Edge((Vertex) vertices.get(j),
								(Vertex) vertices.get(j + 1)));
				}

				obstacles.add(o);
			}

			br.close();
		} catch (Exception e) {
			System.out.println("error: " + e.getMessage());
		}

		return obstacles;
	}
	
	/**
	 * Draw RRT for the environment defined by obstaclesFile,
	 * with the start and goal defiend by startGoalFile
	 * @param obstaclesFile
	 * @param startGoalFile
	 */
	public void drawEnvironments(String obstaclesFile,String startGoalFile){
		System.out.println(obstaclesFile+","+startGoalFile);
		//Read in obstacles and start/goal
		origObstacles = readObstacles(obstaclesFile);
		readStartAndGoal(startGoalFile);
	}
	
	/**
	 * Compute RRT for the environment defined by obstaclesFile,
	 * with the start and goal defiend by startGoalFile
	 * @param obstaclesFile
	 * @param startGoalFile
	 */
	public void computeRRT(){
		System.out.println("Run RRT");
		boolean isNotTreeMerged=true;
		ArrayList<Vertex> startVertices=new ArrayList();
		ArrayList<Vertex> goalVertices=new ArrayList();
		ArrayList<Edge> startEdges=new ArrayList();
		ArrayList<Edge> goalEdges=new ArrayList();
		//ArrayList startTree =new ArrayList();
		//ArrayList goalTree =new ArrayList();
		
		startVertices.add(start);
		goalVertices.add(goal);
		
		while(isNotTreeMerged){
			extendRRT(startVertices,vRandom(startVertices));
		}
	}
	
	public Vertex vRandom(ArrayList vertices){
		Vertex rVertex=new Vertex(0,0);
		int maxradious=5;
		int minradious=1;
		
		rVertex.x=(float)(minradious+(maxradious-minradious)*Math.random())+((Vertex)vertices.get(vertices.size())).x;
		rVertex.y=(float)(minradious+(maxradious-minradious)*Math.random())+((Vertex)vertices.get(vertices.size())).y;
		return rVertex;
	}
	public void extendRRT(ArrayList vertices, Vertex v){
		
		//Find closest neighbor of v in T
		int localdistance=Integer.MAX_VALUE;
		for(int i=0;i<vertices.size();i++){
			if(((Vertex)vertices.get(i)).distanceToVertex(v)<localdistance){
				localdistance=(int)((Vertex)vertices.get(i)).distanceToVertex(v);
			}
		}
	}
}
