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

	final int step_size=10;
	/**
	 * Original obstacles specified in the source file
	 */
	public ArrayList origObstacles;

	/**
	 * Tree
	 */
	ArrayList<Vertex> startVertices;
	ArrayList<Vertex> goalVertices;
	ArrayList<Vertex> mergedVertices;
	ArrayList<Edge> startEdges;
	ArrayList<Edge> goalEdges;
	ArrayList<Edge> mergedEdges;
	ArrayList<Edge> shortestEdges;

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
		System.out.println("Run RRT.....Wait for a while!!!....");
		boolean isNotTreeMerged=true;
		startVertices=new ArrayList();
		goalVertices=new ArrayList();
		mergedVertices=new ArrayList();
		startEdges=new ArrayList();
		goalEdges=new ArrayList();
		mergedEdges=new ArrayList();
		shortestEdges=new ArrayList();
		
		//ArrayList startTree =new ArrayList();
		//ArrayList goalTree =new ArraytList();
		
		startVertices.add(start);
		goalVertices.add(goal);
		
//		startEdges.add(new Edge(start,new Vertex(100,100)));
//		startEdges.add(new Edge(start,new Vertex(50,50)));
//		startEdges.add(new Edge(new Vertex(50,50),new Vertex(200,200)));
//		
//		goalEdges.add(new Edge(goal,new Vertex(400,500)));
//		goalEdges.add(new Edge(goal,new Vertex(380,380)));
//		goalEdges.add(new Edge(new Vertex(380,380),new Vertex(250,200)));
		int startMergedVertex=0;
		int goalMergedVertex=0;
		while(isNotTreeMerged){
			extendRRT(startVertices,startEdges,vRandom());
			extendRRT(goalVertices,goalEdges,vRandom());
			
			for(int i=0;i<startVertices.size();i++){
				for(int j=0;j<goalVertices.size();j++){
					if(((Vertex)startVertices.get(i)).distanceToVertex(((Vertex)goalVertices.get(j)))<=step_size){
						isNotTreeMerged=false;
						startMergedVertex=i;
						goalMergedVertex=j;
					}
				}
			}
			
			//System.out.println(startVertices.size());
		}
		System.out.println("Start and Goal Tree is done.");
		
		//Merge Tree : 
		for(int i=0;i<startVertices.size();i++){
			mergedVertices.add(new Vertex(((Vertex)startVertices.get(i)).x,((Vertex)startVertices.get(i)).y));
		}
		for(int i=0;i<startEdges.size();i++){
			mergedEdges.add(new Edge(((Edge)startEdges.get(i)).v1,((Edge)startEdges.get(i)).v2));
		}
		for(int i=0;i<goalVertices.size();i++){
			mergedVertices.add(new Vertex(((Vertex)goalVertices.get(i)).x,((Vertex)goalVertices.get(i)).y));
		}
		mergedEdges.add(new Edge((Vertex)startVertices.get(startMergedVertex),(Vertex)goalVertices.get(goalMergedVertex)));
		for(int i=0;i<goalEdges.size();i++){
			mergedEdges.add(new Edge(((Edge)goalEdges.get(i)).v1,((Edge)goalEdges.get(i)).v2));
		}
		System.out.println("Merged Tree done.");
		//Find Shortest Path
		calculateShortestPath();
	}
	
	public void calculateShortestPath() { 
		/**
		 * Insert your code here
		 */
		double tempDistance=0;
		double localShortestDistance=Integer.MAX_VALUE;
		int indexShortestEdge=0;
		Vertex goalDirection=new Vertex(0,0);
//		ArrayList<Vertex> mergedVertices = new ArrayList();
		Vertex localShortestVertex =new Vertex(0,0);
		
		boolean[] isVertexValid=new boolean[mergedVertices.size()];
		double[] VertexDistance = new double[mergedVertices.size()];
		int[] previousVertex=new int[mergedVertices.size()];
		int currentVertex=0;
		
		for(int i=0;i<mergedVertices.size();i++){
			VertexDistance[i]=Integer.MAX_VALUE;//((Edge)verticesGraph.get(i)).v1.distanceToVertex(((Edge)verticesGraph.get(i)).v2);
			isVertexValid[i]=true;
		}
		VertexDistance[0]=0;		//start point
		
		
		while(!goal.equals(mergedVertices.get(currentVertex))){
			indexShortestEdge=0;
			localShortestDistance=Integer.MAX_VALUE;
			boolean replaceFlag=false;
			
			//Find smallest distance vertex
			for(int i=0;i<mergedVertices.size();i++){
				if(isVertexValid[i]==true){
					if(VertexDistance[i]<localShortestDistance){
						localShortestDistance=VertexDistance[i];
						currentVertex=i;
					}
				}
			}
			
			//Find shortest path from the latest vertices to neighbors
			for(int i=0;i<mergedEdges.size();i++){
//				previousFlag=false;
				if(((Edge)mergedEdges.get(i)).v1.equals(mergedVertices.get(currentVertex))){
					if(isVertexValid[mergedVertices.indexOf(((Edge)mergedEdges.get(i)).v2)]==true){
						tempDistance=VertexDistance[currentVertex]+mergedVertices.get(currentVertex).distanceToVertex(((Edge)mergedEdges.get(i)).v2);
						if(tempDistance<VertexDistance[mergedVertices.indexOf(((Edge)mergedEdges.get(i)).v2)]){
							VertexDistance[mergedVertices.indexOf(((Edge)mergedEdges.get(i)).v2)]=tempDistance;	
							previousVertex[mergedVertices.indexOf(((Edge)mergedEdges.get(i)).v2)]=mergedVertices.indexOf(((Edge)mergedEdges.get(i)).v1);
						}
					}
				}else if(((Edge)mergedEdges.get(i)).v2.equals(mergedVertices.get(currentVertex))){
					if(isVertexValid[mergedVertices.indexOf(((Edge)mergedEdges.get(i)).v1)]==true){
						tempDistance=VertexDistance[currentVertex]+mergedVertices.get(currentVertex).distanceToVertex(((Edge)mergedEdges.get(i)).v1);
						if(tempDistance<VertexDistance[mergedVertices.indexOf(((Edge)mergedEdges.get(i)).v1)]){
							VertexDistance[mergedVertices.indexOf(((Edge)mergedEdges.get(i)).v1)]=tempDistance;	
							previousVertex[mergedVertices.indexOf(((Edge)mergedEdges.get(i)).v1)]=mergedVertices.indexOf(((Edge)mergedEdges.get(i)).v2);
						}
					}
				}
			}	
			isVertexValid[currentVertex]=false;
		}
		
		//make shortest Path
		shortestEdges.add(0, new Edge(mergedVertices.get(previousVertex[mergedVertices.indexOf(goal)]),goal));
		while(!shortestEdges.get(0).v1.equals(start)){
			shortestEdges.add(0, new Edge(mergedVertices.get(previousVertex[mergedVertices.indexOf(shortestEdges.get(0).v1)]),shortestEdges.get(0).v1));
		}

		//Show shortest Path
		//for(int i=0;i<shortestPath.size();i++){
		//	System.out.println(((Edge)shortestPath.get(i)).toString());
		//}
	}
	
	public Vertex vRandom(){
		Vertex rVertex=new Vertex(0,0);
		int maxradious=600;
		int minradious=0;
		
		rVertex.x=(float)(minradious+(maxradious-minradious)*Math.random());//+goal.x;
		rVertex.y=(float)(minradious+(maxradious-minradious)*Math.random());//+goal.y;
		//rVertex.print();
		
		return rVertex;
	}
	
	public void extendRRT(ArrayList vertices, ArrayList edges, Vertex v){
		Vertex q_near=new Vertex(0,0);
		Vertex q_new=new Vertex(0,0);
		
		//q_near <---- Find closest neighbor of v in T
		int localdistance=Integer.MAX_VALUE;
		for(int i=0;i<vertices.size();i++){
			if(((Vertex)vertices.get(i)).distanceToVertex(v)<localdistance){
				localdistance=(int)((Vertex)vertices.get(i)).distanceToVertex(v);
				q_near=(Vertex)vertices.get(i);
			}
		}
		
		//q_new 
		q_new.x=q_near.x+(float)(step_size*Math.cos((double)Math.atan2(v.y-q_near.y,v.x-q_near.x )));
		q_new.y=q_near.y+(float)(step_size*Math.sin((double)Math.atan2(v.y-q_near.y,v.x-q_near.x )));
		
		//Check Collision Free
		Edge tempEdge=new Edge(q_near,q_new);
		if(isCollisonFree(tempEdge)){
			vertices.add(q_new);
			edges.add(tempEdge);
		}
	}
	
	public boolean isCollisonFree(Edge e){
		boolean isCFree=true;
		for(int i=0; i<origObstacles.size();i++){
			for(int j=0;j<((Obstacle)origObstacles.get(i)).edges.size();j++){
				if(lineSegmentIntersection(e.v1,e.v2,((Edge)((Obstacle)origObstacles.get(i)).edges.get(j)))){
					isCFree=false;
					break;
				}
			}
		}
		return isCFree; 
	}
	
	boolean lineSegmentIntersection(Vertex _start, Vertex _end, Edge other_line)
    {
        float denom = ((other_line.v2.y - other_line.v1.y)*(_end.x - _start.x)) -
                      ((other_line.v2.x - other_line.v1.x)*(_end.y - _start.y));

        float nume_a = ((other_line.v2.x - other_line.v1.x)*(_start.y - other_line.v1.y)) -
                       ((other_line.v2.y - other_line.v1.y)*(_start.x - other_line.v1.x));

        float nume_b = ((_end.x - _start.x)*(_start.y - other_line.v1.y)) -
                       ((_end.y - _start.y)*(_start.x - other_line.v1.x));

        if(denom == 0)
        {
            if((nume_a == 0) && (nume_b == 0))
            {
                return false; //COINCIDENT;
            }
            return false; //PARALLEL;
        }

        float ua = nume_a / denom;
        float ub = nume_b / denom;

        if(ua >=0 && ua <=1 && ub >=0 && ub <=1)
        {
            // Get the intersection point.
            //intersection.x_ = begin_.x_ + ua*(end_.x_ - begin_.x_);
            //intersection.y_ = begin_.y_ + ua*(end_.y_ - begin_.y_);

            return true; //INTERESECTING;
        }

        return false; //NOT_INTERESECTING;
    }
}
