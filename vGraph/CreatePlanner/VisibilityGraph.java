package CreatePlanner;

import java.awt.Point;
import java.awt.geom.Line2D;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.*;

public class VisibilityGraph {
	/**
	 * Original obstacles specified in the source file
	 */
	public ArrayList origObstacles;

	/**
	 * Grown obstacles 
	 */
	public ArrayList gObstacles;

	/**
	 * Vertices graph
	 */
	public ArrayList vGraph;

	/**
	 * Shortest path computed from Visibility graph
	 */
	public ArrayList shortestPath;

	/**
	 * Starting point for robot
	 */
	public Vertex start;

	/**
	 * Goal of robot
	 */
	public Vertex goal;

	
	/**
	 * set of computed waypoints to be provided to PERController's followPath()
	 * to direct the PER from the starting point to the goal 
	 */
	//public int[][] waypoints;
	public ArrayList<Point> waypoints;

	public ArrayList getGrownObstacles() {
		return gObstacles;
	}
	
	public ArrayList getVisibilityGraph() {
		return vGraph;
	}
	
	public ArrayList getShortestPath() {
		return shortestPath;
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
	 * Compute the visibility graph for the environment defined by obstaclesFile,
	 * with the start and goal defiend by startGoalFile
	 * @param obstaclesFile
	 * @param startGoalFile
	 */
	public void computeVisibilityGraph(String obstaclesFile,String startGoalFile) {
		//Read in obstacles and start/goal
		origObstacles = readObstacles(obstaclesFile);
		readStartAndGoal(startGoalFile);
		
		gObstacles = new ArrayList(); 
		gObstacles.add(origObstacles.get(0)); //first obstacle in original is the wall/boundary, add it to the grownObstacles as is
		
		/************************************
		 * Implement the below functionality
		 ************************************/
		//calculate grown obstacles
		gObstacles = growObstacles(origObstacles);
				
		//calculate visibility graph
		vGraph = makeVisibilityGraph(gObstacles);
		//for(int i=0;i<vGraph.size();i++)
		//	System.out.println("vGraph "+i+":"+vGraph.get(i).toString());
		
		//calculate shortest path
		shortestPath = DijkstrasAlgorithm.calculateShortestPath(vGraph,start,goal,gObstacles);
		
		//calculate waypoints for PER to follow (from shortestPath)
		waypoints = calculateWaypoints();
		
	}
	public ArrayList calculateWaypoints(){
		ArrayList<Point> localtarget = new ArrayList();
		
		for(int i=0;i<shortestPath.size();i++){
			localtarget.add(new Point((int)((((Edge)shortestPath.get(i)).v2.x+((Edge)shortestPath.get(0)).v1.x*(-1))*1000),(int)((((Edge)shortestPath.get(i)).v2.y+((Edge)shortestPath.get(0)).v1.y*(-1))*1000)));//Change Meter to millimeter
			System.out.println("Point["+i+"] = ("+localtarget.get(i).x+","+localtarget.get(i).y+")");
		}
		return localtarget;
	}

	public ArrayList growObstacles_version1(ArrayList origObstacles)
	{
		for(int i = 1; i < origObstacles.size(); i++){
			Obstacle o = new Obstacle();
			for(int j=0;j<((ArrayList)((Obstacle)origObstacles.get(i)).listVertices()).size();j++){
				if(j==((ArrayList)((Obstacle)origObstacles.get(i)).listVertices()).size()-1)
					o.addEdge(new Edge(new Vertex(((Vertex)((ArrayList)((Obstacle)origObstacles.get(i)).listVertices()).get(j)).x,((Vertex)((ArrayList)((Obstacle)origObstacles.get(i)).listVertices()).get(j)).y)
					                  ,new Vertex(((Vertex)((ArrayList)((Obstacle)origObstacles.get(i)).listVertices()).get(0)).x,((Vertex)((ArrayList)((Obstacle)origObstacles.get(i)).listVertices()).get(0)).y)));
				else
					o.addEdge(new Edge(new Vertex(((Vertex)((ArrayList)((Obstacle)origObstacles.get(i)).listVertices()).get(j)).x,((Vertex)((ArrayList)((Obstacle)origObstacles.get(i)).listVertices()).get(j)).y)
	                                  ,new Vertex(((Vertex)((ArrayList)((Obstacle)origObstacles.get(i)).listVertices()).get(j+1)).x,((Vertex)((ArrayList)((Obstacle)origObstacles.get(i)).listVertices()).get(j+1)).y)));
			}
			gObstacles.add(o);
		}
		
		float minX,minY,maxX,maxY,centerX,centerY;
		float radius=(float) 0.17;	//Creat Diameter : 0.33 [m]
		for(int i = 1; i < gObstacles.size(); i++){
			//Find max and min of vertex
			minX=((Obstacle)gObstacles.get(i)).getMinX();
			minY=((Obstacle)gObstacles.get(i)).getMinY();
			maxX=((Obstacle)gObstacles.get(i)).getMaxX();
			maxY=((Obstacle)gObstacles.get(i)).getMaxY();
			centerX=maxX-Math.abs(maxX-minX)/2;
			centerY=maxY-Math.abs(maxY-minY)/2;
			for(int j=0;j<(((Obstacle)gObstacles.get(i)).getEdgeArray()).length;j++){
				if((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.x>=centerX){
					(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.x+=radius;
					if(checkCrossBoundary((((Obstacle)gObstacles.get(i)).getEdgeArray())[j])){
						(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.x-=radius;
					}	
				}else if((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.x<=centerX){
					(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.x-=radius;
					if(checkCrossBoundary((((Obstacle)gObstacles.get(i)).getEdgeArray())[j])){
						(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.x+=radius;
					}	
				}
				if((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.y>=centerY){
					(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.y+=radius;
					if(checkCrossBoundary((((Obstacle)gObstacles.get(i)).getEdgeArray())[j])){
						(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.y-=radius;
					}	
				}else if((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.y<=centerY){
					(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.y-=radius;
					if(checkCrossBoundary((((Obstacle)gObstacles.get(i)).getEdgeArray())[j])){
						(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.y+=radius;
					}	
				}
				if((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.x>=centerX){
					(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.x+=radius;
					if(checkCrossBoundary((((Obstacle)gObstacles.get(i)).getEdgeArray())[j])){
						(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.x-=radius;
					}	
				}else if((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.x<=centerX){
					(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.x-=radius;
					if(checkCrossBoundary((((Obstacle)gObstacles.get(i)).getEdgeArray())[j])){
						(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.x+=radius;
					}	
				}
				if((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.y>=centerY){
					(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.y+=radius;
					if(checkCrossBoundary((((Obstacle)gObstacles.get(i)).getEdgeArray())[j])){
						(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.y-=radius;
					}	
				}else if((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.y<=centerY){
					(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.y-=radius;
					if(checkCrossBoundary((((Obstacle)gObstacles.get(i)).getEdgeArray())[j])){
						(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.y+=radius;
					}	
				}
			}
		}
		//Update Min, Max
		for(int i = 1; i < gObstacles.size(); i++){
			((Obstacle)gObstacles.get(i)).updateMinMax();
		}
		//check objects Overlapped each other
		//ToDo: !!!!!!
		int[] CrossedObject = new int[gObstacles.size()];		//No boundary
		boolean isCrossedObject=false;
		for(int i = 1; i < gObstacles.size(); i++){
			for(int j=0;j<(((Obstacle)gObstacles.get(i)).getEdgeArray()).length;j++){
				for(int m = 1; m < gObstacles.size(); m++){
					if(i!=m){
						for(int n=0;n<(((Obstacle)gObstacles.get(m)).getEdgeArray()).length;n++){
							if(lineSegmentIntersection((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1,(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2,((Edge)((Obstacle)gObstacles.get(m)).edges.get(n)))){
								isCrossedObject=true;
								CrossedObject[i]=m;
							}
						}
					}
				}
			}
			//System.out.println("CrossedObject["+i+"]="+CrossedObject[i]);
		}
		
		//Merge overlapped object to one object
		int s=gObstacles.size();
		for(int i = 1; i < s; i++){
			if(CrossedObject[i]!=0){
				//Extract Vertex from two objects
				for(int j=0;j<(((Obstacle)gObstacles.get(i)).getEdgeArray()).length;j++){
					for(int n=0;n<(((Obstacle)gObstacles.get(CrossedObject[i])).getEdgeArray()).length;n++){
						if(lineSegmentIntersection((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1,(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2,((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n)))){
							Vertex v = lineSegmentIntersectionVertex((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1,(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2,((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n)));
							if((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.x<=((Obstacle)gObstacles.get(CrossedObject[i])).getMaxX()&&(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.x>=((Obstacle)gObstacles.get(CrossedObject[i])).getMinX()){
								if((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.y<=((Obstacle)gObstacles.get(CrossedObject[i])).getMaxY()&&(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.y>=((Obstacle)gObstacles.get(CrossedObject[i])).getMinY()){
									(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.x=v.x;
									(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.y=v.y;
									if(j==(((Obstacle)gObstacles.get(i)).getEdgeArray()).length-1){
										(((Obstacle)gObstacles.get(i)).getEdgeArray())[0].v1.x=v.x;
										(((Obstacle)gObstacles.get(i)).getEdgeArray())[0].v1.y=v.y;
									}else{
										(((Obstacle)gObstacles.get(i)).getEdgeArray())[j+1].v1.x=v.x;
										(((Obstacle)gObstacles.get(i)).getEdgeArray())[j+1].v1.y=v.y;
									}
								}
							}else if((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.x<=((Obstacle)gObstacles.get(CrossedObject[i])).getMaxX()&&(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.x>=((Obstacle)gObstacles.get(CrossedObject[i])).getMinX()){
								if((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.y<=((Obstacle)gObstacles.get(CrossedObject[i])).getMaxY()&&(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.y>=((Obstacle)gObstacles.get(CrossedObject[i])).getMinY()){
									(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.x=v.x;
									(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.y=v.y;
									if(j==0){
										(((Obstacle)gObstacles.get(i)).getEdgeArray())[(((Obstacle)gObstacles.get(i)).getEdgeArray()).length-1].v2.x=v.x;
										(((Obstacle)gObstacles.get(i)).getEdgeArray())[(((Obstacle)gObstacles.get(i)).getEdgeArray()).length-1].v2.y=v.y;
									}else{
										(((Obstacle)gObstacles.get(i)).getEdgeArray())[j-1].v2.x=v.x;
										(((Obstacle)gObstacles.get(i)).getEdgeArray())[j-1].v2.y=v.y;
									}
								}
							}
							if(((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n)).v1.x<=((Obstacle)gObstacles.get(i)).getMaxX()&&((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n)).v1.x>=((Obstacle)gObstacles.get(i)).getMinX()){
								if(((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n)).v1.y<=((Obstacle)gObstacles.get(i)).getMaxY()&&((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n)).v1.y>=((Obstacle)gObstacles.get(i)).getMinY()){
									((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n)).v1.x=v.x;
									((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n)).v1.y=v.y;
									if(n==0){
										((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get((((Obstacle)gObstacles.get(CrossedObject[i])).getEdgeArray()).length-1)).v2.x=v.x;
										((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get((((Obstacle)gObstacles.get(CrossedObject[i])).getEdgeArray()).length-1)).v2.y=v.y;
									}else{
										((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n-1)).v2.x=v.x;
										((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n-1)).v2.y=v.y;
									}
								}
							}else if(((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n)).v2.x<=((Obstacle)gObstacles.get(i)).getMaxX()&&((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n)).v2.x>=((Obstacle)gObstacles.get(i)).getMinX()){
								if(((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n)).v2.y<=((Obstacle)gObstacles.get(i)).getMaxY()&&((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n)).v2.y>=((Obstacle)gObstacles.get(i)).getMinY()){
									((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n)).v2.x=v.x;
									((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n)).v2.y=v.y;
									if(n==(((Obstacle)gObstacles.get(CrossedObject[i])).getEdgeArray()).length-1){
										((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(0)).v1.x=v.x;
										((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(0)).v1.y=v.y;
									}else{
										((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n+1)).v1.x=v.x;
										((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n+1)).v1.y=v.y;
									}
								}
							}
						}
					}
				}
				
				//Make Merged Object
				Obstacle o = new Obstacle();
				
				for(int j=0;j<(((Obstacle)gObstacles.get(i)).getEdgeArray()).length;j++){
//					for(int n=0;n<(((Obstacle)gObstacles.get(CrossedObject[i])).getEdgeArray()).length;n++){
//						if((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.x==((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n)).v2.x&&(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.x==((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n)).v1.x){
//							if
//						}else{
							o.addEdge((((Obstacle)gObstacles.get(i)).getEdgeArray())[j]);
//						}
//					}
				}
				for(int n=0;n<(((Obstacle)gObstacles.get(CrossedObject[i])).getEdgeArray()).length;n++){
					//if(((Edge)o.edges.getLast()).v2.equals((((Obstacle)gObstacles.get(CrossedObject[i])).getEdgeArray())[n].v1)){
						o.addEdge((((Obstacle)gObstacles.get(CrossedObject[i])).getEdgeArray())[n]);
					//}
				}

				for(int z=0;z<o.getEdgeArray().length;z++){
					System.out.println(o.getEdgeArray()[z].toString());
				}
				//Delete the object and add object
				if(i>CrossedObject[i]){
					gObstacles.remove(i);
					gObstacles.remove(CrossedObject[i]);
				}else{
					gObstacles.remove(CrossedObject[i]);
					gObstacles.remove(i);
				}
				gObstacles.add(o);
				if(CrossedObject[CrossedObject[i]]==i){
					CrossedObject[CrossedObject[i]]=0;
				    CrossedObject[i]=0;
				}
			}
		}
		
		//Debug
		System.out.println("Grown obstacles size : "+gObstacles.size());
		System.out.println("Original Obstacles");
		for(int i = 1; i < origObstacles.size(); i++){
			for(int j=0;j<((ArrayList)((Obstacle)origObstacles.get(i)).listVertices()).size();j++)
				((Vertex)((ArrayList)((Obstacle)origObstacles.get(i)).listVertices()).get(j)).print();
		}
		System.out.println("Grown Obstacles");
		for(int i = 1; i < gObstacles.size(); i++){
			for(int j=0;j<((ArrayList)((Obstacle)gObstacles.get(i)).listVertices()).size();j++)
				((Vertex)((ArrayList)((Obstacle)gObstacles.get(i)).listVertices()).get(j)).print();
		}
		return gObstacles;
	}
	
	public ArrayList growObstacles_version2(ArrayList origObstacles)
	{
		for(int i = 1; i < origObstacles.size(); i++){
			Obstacle o = new Obstacle();
			for(int j=0;j<((ArrayList)((Obstacle)origObstacles.get(i)).listVertices()).size();j++){
				if(j==((ArrayList)((Obstacle)origObstacles.get(i)).listVertices()).size()-1)
					o.addEdge(new Edge(new Vertex(((Vertex)((ArrayList)((Obstacle)origObstacles.get(i)).listVertices()).get(j)).x,((Vertex)((ArrayList)((Obstacle)origObstacles.get(i)).listVertices()).get(j)).y)
					                  ,new Vertex(((Vertex)((ArrayList)((Obstacle)origObstacles.get(i)).listVertices()).get(0)).x,((Vertex)((ArrayList)((Obstacle)origObstacles.get(i)).listVertices()).get(0)).y)));
				else
					o.addEdge(new Edge(new Vertex(((Vertex)((ArrayList)((Obstacle)origObstacles.get(i)).listVertices()).get(j)).x,((Vertex)((ArrayList)((Obstacle)origObstacles.get(i)).listVertices()).get(j)).y)
	                                  ,new Vertex(((Vertex)((ArrayList)((Obstacle)origObstacles.get(i)).listVertices()).get(j+1)).x,((Vertex)((ArrayList)((Obstacle)origObstacles.get(i)).listVertices()).get(j+1)).y)));
			}
			gObstacles.add(o);
		}
		
		float minX,minY,maxX,maxY,centerX,centerY;
		float radius=(float) 0.17;	//Creat Diameter : 0.33 [m]
		for(int i = 1; i < gObstacles.size(); i++){
			//Find max and min of vertex
			minX=((Obstacle)gObstacles.get(i)).getMinX();
			minY=((Obstacle)gObstacles.get(i)).getMinY();
			maxX=((Obstacle)gObstacles.get(i)).getMaxX();
			maxY=((Obstacle)gObstacles.get(i)).getMaxY();
			centerX=maxX-Math.abs(maxX-minX)/2;
			centerY=maxY-Math.abs(maxY-minY)/2;
			for(int j=0;j<(((Obstacle)gObstacles.get(i)).getEdgeArray()).length;j++){
				if((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.x>=centerX){
					(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.x+=radius;
					if(checkCrossBoundary((((Obstacle)gObstacles.get(i)).getEdgeArray())[j])){
						(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.x-=radius;
					}	
				}else if((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.x<=centerX){
					(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.x-=radius;
					if(checkCrossBoundary((((Obstacle)gObstacles.get(i)).getEdgeArray())[j])){
						(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.x+=radius;
					}	
				}
				if((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.y>=centerY){
					(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.y+=radius;
					if(checkCrossBoundary((((Obstacle)gObstacles.get(i)).getEdgeArray())[j])){
						(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.y-=radius;
					}	
				}else if((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.y<=centerY){
					(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.y-=radius;
					if(checkCrossBoundary((((Obstacle)gObstacles.get(i)).getEdgeArray())[j])){
						(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.y+=radius;
					}	
				}
				if((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.x>=centerX){
					(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.x+=radius;
					if(checkCrossBoundary((((Obstacle)gObstacles.get(i)).getEdgeArray())[j])){
						(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.x-=radius;
					}	
				}else if((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.x<=centerX){
					(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.x-=radius;
					if(checkCrossBoundary((((Obstacle)gObstacles.get(i)).getEdgeArray())[j])){
						(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.x+=radius;
					}	
				}
				if((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.y>=centerY){
					(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.y+=radius;
					if(checkCrossBoundary((((Obstacle)gObstacles.get(i)).getEdgeArray())[j])){
						(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.y-=radius;
					}	
				}else if((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.y<=centerY){
					(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.y-=radius;
					if(checkCrossBoundary((((Obstacle)gObstacles.get(i)).getEdgeArray())[j])){
						(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.y+=radius;
					}	
				}
			}
		}
		//Update Min, Max
		for(int i = 1; i < gObstacles.size(); i++){
			((Obstacle)gObstacles.get(i)).updateMinMax();
		}
		//check objects Overlapped each other
		//ToDo: !!!!!!
		int[] CrossedObject = new int[gObstacles.size()];		//No boundary
		boolean isCrossedObject=false;
		for(int i = 1; i < gObstacles.size(); i++){
			for(int j=0;j<(((Obstacle)gObstacles.get(i)).getEdgeArray()).length;j++){
				for(int m = 1; m < gObstacles.size(); m++){
					if(i!=m){
						for(int n=0;n<(((Obstacle)gObstacles.get(m)).getEdgeArray()).length;n++){
							if(lineSegmentIntersection((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1,(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2,((Edge)((Obstacle)gObstacles.get(m)).edges.get(n)))){
								isCrossedObject=true;
								CrossedObject[i]=m;
							}
						}
					}
				}
			}
			System.out.println("CrossedObject["+i+"]="+CrossedObject[i]);
		}
		
		//Merge overlapped object to one object
		int s=gObstacles.size();
		for(int i = 1; i < s; i++){
			if(CrossedObject[i]!=0){
				//Extract Vertex from two objects
				for(int j=0;j<(((Obstacle)gObstacles.get(i)).getEdgeArray()).length;j++){
					for(int n=0;n<(((Obstacle)gObstacles.get(CrossedObject[i])).getEdgeArray()).length;n++){
						if(lineSegmentIntersection((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1,(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2,((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n)))){
							Vertex v = lineSegmentIntersectionVertex((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1,(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2,((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n)));
							if((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.x<=((Obstacle)gObstacles.get(CrossedObject[i])).getMaxX()&&(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.x>=((Obstacle)gObstacles.get(CrossedObject[i])).getMinX()){
								if((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.y<=((Obstacle)gObstacles.get(CrossedObject[i])).getMaxY()&&(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.y>=((Obstacle)gObstacles.get(CrossedObject[i])).getMinY()){
									(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.x=v.x;
									(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.y=v.y;
									if(j==(((Obstacle)gObstacles.get(i)).getEdgeArray()).length-1){
										(((Obstacle)gObstacles.get(i)).getEdgeArray())[0].v1.x=v.x;
										(((Obstacle)gObstacles.get(i)).getEdgeArray())[0].v1.y=v.y;
									}else{
										(((Obstacle)gObstacles.get(i)).getEdgeArray())[j+1].v1.x=v.x;
										(((Obstacle)gObstacles.get(i)).getEdgeArray())[j+1].v1.y=v.y;
									}
								}
							}else if((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.x<=((Obstacle)gObstacles.get(CrossedObject[i])).getMaxX()&&(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.x>=((Obstacle)gObstacles.get(CrossedObject[i])).getMinX()){
								if((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.y<=((Obstacle)gObstacles.get(CrossedObject[i])).getMaxY()&&(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.y>=((Obstacle)gObstacles.get(CrossedObject[i])).getMinY()){
									(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.x=v.x;
									(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.y=v.y;
									if(j==0){
										(((Obstacle)gObstacles.get(i)).getEdgeArray())[(((Obstacle)gObstacles.get(i)).getEdgeArray()).length-1].v2.x=v.x;
										(((Obstacle)gObstacles.get(i)).getEdgeArray())[(((Obstacle)gObstacles.get(i)).getEdgeArray()).length-1].v2.y=v.y;
									}else{
										(((Obstacle)gObstacles.get(i)).getEdgeArray())[j-1].v2.x=v.x;
										(((Obstacle)gObstacles.get(i)).getEdgeArray())[j-1].v2.y=v.y;
									}
								}
							}
							if(((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n)).v1.x<=((Obstacle)gObstacles.get(i)).getMaxX()&&((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n)).v1.x>=((Obstacle)gObstacles.get(i)).getMinX()){
								if(((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n)).v1.y<=((Obstacle)gObstacles.get(i)).getMaxY()&&((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n)).v1.y>=((Obstacle)gObstacles.get(i)).getMinY()){
									((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n)).v1.x=v.x;
									((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n)).v1.y=v.y;
									if(n==0){
										((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get((((Obstacle)gObstacles.get(CrossedObject[i])).getEdgeArray()).length-1)).v2.x=v.x;
										((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get((((Obstacle)gObstacles.get(CrossedObject[i])).getEdgeArray()).length-1)).v2.y=v.y;
									}else{
										((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n-1)).v2.x=v.x;
										((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n-1)).v2.y=v.y;
									}
								}
							}else if(((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n)).v2.x<=((Obstacle)gObstacles.get(i)).getMaxX()&&((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n)).v2.x>=((Obstacle)gObstacles.get(i)).getMinX()){
								if(((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n)).v2.y<=((Obstacle)gObstacles.get(i)).getMaxY()&&((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n)).v2.y>=((Obstacle)gObstacles.get(i)).getMinY()){
									((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n)).v2.x=v.x;
									((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n)).v2.y=v.y;
									if(n==(((Obstacle)gObstacles.get(CrossedObject[i])).getEdgeArray()).length-1){
										((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(0)).v1.x=v.x;
										((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(0)).v1.y=v.y;
									}else{
										((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n+1)).v1.x=v.x;
										((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n+1)).v1.y=v.y;
									}
								}
							}
						}
					}
				}
				
				//Make Merged Object
				Obstacle o = new Obstacle();
				ArrayList<Vertex> mergedVertex = new ArrayList();
				
				for(int j=0;j<(((Obstacle)gObstacles.get(i)).getEdgeArray()).length;j++)
					mergedVertex.add((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1);
				for(int n=0;n<(((Obstacle)gObstacles.get(CrossedObject[i])).getEdgeArray()).length;n++)
					mergedVertex.add(((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n)).v1);
                
				boolean[] isValidMergedVertex=new boolean[mergedVertex.size()];
				for(int v=0;v<mergedVertex.size();v++){
					isValidMergedVertex[v]=true;
					mergedVertex.get(v).print();
				}
				for(int v=1;v<mergedVertex.size();v++){
                	if(mergedVertex.get(0).y==mergedVertex.get(v).y){
                		o.addEdge(new Edge(mergedVertex.get(0),mergedVertex.get(v)));
                		isValidMergedVertex[v]=false;
                	}
				}
				
				//Make new objects
				while(!((Edge)o.edges.getLast()).v2.equals(mergedVertex.get(0))){
					float localminy=Float.MAX_VALUE; 
					float localminx=Float.MAX_VALUE;
					int nextVertexX=mergedVertex.size();
					int nextVertexY=mergedVertex.size();
					for(int v=0;v<mergedVertex.size();v++){
						if(isValidMergedVertex[v]==true){
		                	if((((Edge)o.edges.getLast()).v2.y==mergedVertex.get(v).y)&&(!((Edge)o.edges.getLast()).v1.equals(mergedVertex.get(v)))){
		                		if(Math.abs(((Edge)o.edges.getLast()).v2.x-mergedVertex.get(v).x)<localminy){
		                			localminy=Math.abs(((Edge)o.edges.getLast()).v2.x-mergedVertex.get(v).x);
		                			nextVertexY=v;
		                		}
		                	}else if((((Edge)o.edges.getLast()).v2.x==mergedVertex.get(v).x)&&(!((Edge)o.edges.getLast()).v1.equals(mergedVertex.get(v)))){
		                		if(Math.abs(((Edge)o.edges.getLast()).v2.y-mergedVertex.get(v).y)<localminx){
		                			localminx=Math.abs(((Edge)o.edges.getLast()).v2.y-mergedVertex.get(v).y);
		                			nextVertexX=v;
		                		}
		                	}
						}
					}
					
					if(nextVertexY!=mergedVertex.size()){
						o.addEdge(new Edge(((Edge)o.edges.getLast()).v2,mergedVertex.get(nextVertexY)));
						isValidMergedVertex[nextVertexY]=false;
					}else if(nextVertexX!=mergedVertex.size()){
						o.addEdge(new Edge(((Edge)o.edges.getLast()).v2,mergedVertex.get(nextVertexX)));
						isValidMergedVertex[nextVertexX]=false;
					}
				}

				for(int z=0;z<o.getEdgeArray().length;z++){
					System.out.println(o.getEdgeArray()[z].toString());
				}
				
				//Delete the object and add object
				if(i>CrossedObject[i]){
					gObstacles.remove(i);
					gObstacles.remove(CrossedObject[i]);
				}else{
					gObstacles.remove(CrossedObject[i]);
					gObstacles.remove(i);
				}
				gObstacles.add(o);
				if(CrossedObject[CrossedObject[i]]==i){
					CrossedObject[CrossedObject[i]]=0;
				    CrossedObject[i]=0;
				}
			}
		}
		
		//Debug
		System.out.println("Grown obstacles size : "+gObstacles.size());
		System.out.println("Original Obstacles");
		for(int i = 1; i < origObstacles.size(); i++){
			for(int j=0;j<((ArrayList)((Obstacle)origObstacles.get(i)).listVertices()).size();j++)
				((Vertex)((ArrayList)((Obstacle)origObstacles.get(i)).listVertices()).get(j)).print();
		}
		System.out.println("Grown Obstacles");
		for(int i = 1; i < gObstacles.size(); i++){
			for(int j=0;j<((ArrayList)((Obstacle)gObstacles.get(i)).listVertices()).size();j++)
				((Vertex)((ArrayList)((Obstacle)gObstacles.get(i)).listVertices()).get(j)).print();
		}
		return gObstacles;
	}
	
	public ArrayList growObstacles(ArrayList origObstacles)
	{
		for(int i = 1; i < origObstacles.size(); i++){
			Obstacle o = new Obstacle();
			for(int j=0;j<((ArrayList)((Obstacle)origObstacles.get(i)).listVertices()).size();j++){
				if(j==((ArrayList)((Obstacle)origObstacles.get(i)).listVertices()).size()-1)
					o.addEdge(new Edge(new Vertex(((Vertex)((ArrayList)((Obstacle)origObstacles.get(i)).listVertices()).get(j)).x,((Vertex)((ArrayList)((Obstacle)origObstacles.get(i)).listVertices()).get(j)).y)
					                  ,new Vertex(((Vertex)((ArrayList)((Obstacle)origObstacles.get(i)).listVertices()).get(0)).x,((Vertex)((ArrayList)((Obstacle)origObstacles.get(i)).listVertices()).get(0)).y)));
				else
					o.addEdge(new Edge(new Vertex(((Vertex)((ArrayList)((Obstacle)origObstacles.get(i)).listVertices()).get(j)).x,((Vertex)((ArrayList)((Obstacle)origObstacles.get(i)).listVertices()).get(j)).y)
	                                  ,new Vertex(((Vertex)((ArrayList)((Obstacle)origObstacles.get(i)).listVertices()).get(j+1)).x,((Vertex)((ArrayList)((Obstacle)origObstacles.get(i)).listVertices()).get(j+1)).y)));
			}
			gObstacles.add(o);
		}
		
		float minX,minY,maxX,maxY,centerX,centerY;
		float radius=(float) 0.18;	//Creat Diameter : 0.33 [m]
		for(int i = 1; i < gObstacles.size(); i++){
			//Find max and min of vertex
			minX=((Obstacle)gObstacles.get(i)).getMinX();
			minY=((Obstacle)gObstacles.get(i)).getMinY();
			maxX=((Obstacle)gObstacles.get(i)).getMaxX();
			maxY=((Obstacle)gObstacles.get(i)).getMaxY();
			centerX=maxX-Math.abs(maxX-minX)/2;
			centerY=maxY-Math.abs(maxY-minY)/2;
			for(int j=0;j<(((Obstacle)gObstacles.get(i)).getEdgeArray()).length;j++){
				if((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.x>=centerX){
					(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.x+=radius;
					if(checkCrossBoundary((((Obstacle)gObstacles.get(i)).getEdgeArray())[j])){
						(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.x-=radius;
					}	
				}else if((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.x<=centerX){
					(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.x-=radius;
					if(checkCrossBoundary((((Obstacle)gObstacles.get(i)).getEdgeArray())[j])){
						(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.x+=radius;
					}	
				}
				if((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.y>=centerY){
					(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.y+=radius;
					if(checkCrossBoundary((((Obstacle)gObstacles.get(i)).getEdgeArray())[j])){
						(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.y-=radius;
					}	
				}else if((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.y<=centerY){
					(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.y-=radius;
					if(checkCrossBoundary((((Obstacle)gObstacles.get(i)).getEdgeArray())[j])){
						(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1.y+=radius;
					}	
				}
				if((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.x>=centerX){
					(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.x+=radius;
					if(checkCrossBoundary((((Obstacle)gObstacles.get(i)).getEdgeArray())[j])){
						(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.x-=radius;
					}	
				}else if((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.x<=centerX){
					(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.x-=radius;
					if(checkCrossBoundary((((Obstacle)gObstacles.get(i)).getEdgeArray())[j])){
						(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.x+=radius;
					}	
				}
				if((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.y>=centerY){
					(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.y+=radius;
					if(checkCrossBoundary((((Obstacle)gObstacles.get(i)).getEdgeArray())[j])){
						(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.y-=radius;
					}	
				}else if((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.y<=centerY){
					(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.y-=radius;
					if(checkCrossBoundary((((Obstacle)gObstacles.get(i)).getEdgeArray())[j])){
						(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2.y+=radius;
					}	
				}
			}
		}
		//Update Min, Max
		for(int i = 1; i < gObstacles.size(); i++){
			((Obstacle)gObstacles.get(i)).updateMinMax();
		}
		//check objects Overlapped each other
		//ToDo: !!!!!!
		boolean isCrossedObject;
		do{
			int[] CrossedObject = new int[gObstacles.size()];		//No boundary
			isCrossedObject=false;
			for(int i = 1; i < gObstacles.size(); i++){
				for(int j=0;j<(((Obstacle)gObstacles.get(i)).getEdgeArray()).length;j++){
					for(int m = 1; m < gObstacles.size(); m++){
						if(i!=m){
							for(int n=0;n<(((Obstacle)gObstacles.get(m)).getEdgeArray()).length;n++){
								if(lineSegmentIntersection((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1,(((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v2,((Edge)((Obstacle)gObstacles.get(m)).edges.get(n)))){
									isCrossedObject=true;
									CrossedObject[i]=m;
								}
							}
						}
					}
				}
				//System.out.println("CrossedObject["+i+"]="+CrossedObject[i]);
			}
		
			//Merge overlapped object to one object
			if(isCrossedObject){
				int s=gObstacles.size();
				for(int i = 1; i < s; i++){
					if(CrossedObject[i]!=0){
						//Make Merged Object using Graham's Algorithm (Convex hull)
						Obstacle o = new Obstacle();
						ArrayList<Vertex> mergedVertex = new ArrayList();

						for(int j=0;j<(((Obstacle)gObstacles.get(i)).getEdgeArray()).length;j++)
							mergedVertex.add((((Obstacle)gObstacles.get(i)).getEdgeArray())[j].v1);
						for(int n=0;n<(((Obstacle)gObstacles.get(CrossedObject[i])).getEdgeArray()).length;n++)
							if(mergedVertex.indexOf(((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n)).v1)==-1)
								mergedVertex.add(((Edge)((Obstacle)gObstacles.get(CrossedObject[i])).edges.get(n)).v1);
		                
						boolean[] isValidMergedVertex=new boolean[mergedVertex.size()];
						for(int v=0;v<mergedVertex.size();v++){
							isValidMergedVertex[v]=true;
							//mergedVertex.get(v).print();
						}
						
						//Find rightmost, lowest point
						//Angular sort
						//Push Pn-1 and P0 into the stack
						//while i<N do
						// if Pi is strictly left of the line formed by top 2 stack entries, 
						//     then push pi onto the stack
						// else pop the stack (remove the p_top)
						Vertex rightLow=new Vertex(0,0);
						rightLow.x=Float.MIN_VALUE;
						rightLow.y=Float.MIN_VALUE;
						int rightLowIndex=0;
						for(int v=0;v<mergedVertex.size();v++){
		                	if(mergedVertex.get(v).x>rightLow.x && mergedVertex.get(v).y>rightLow.y){
		                		rightLow.x=mergedVertex.get(v).x;
		                		rightLow.y=mergedVertex.get(v).y;
		                		rightLowIndex=v;
		                	}
						}
						//System.out.println("rightLow : " + rightLowIndex);
						
						for(int v=0;v<mergedVertex.size();v++){
							mergedVertex.get(v).angleFromStart(mergedVertex.get(rightLowIndex));
						}
						Vertex tempVertex=new Vertex(0,0);
						for(int n=0; n<mergedVertex.size();n++)
							for(int j=0; j<(mergedVertex.size()-1);j++)
								if(((Vertex)mergedVertex.get(j)).angle>((Vertex)mergedVertex.get(j+1)).angle){
									tempVertex=((Vertex)mergedVertex.get(j+1));
									mergedVertex.set(j+1, (Vertex)mergedVertex.get(j));
									mergedVertex.set(j,tempVertex);
								}
						//for(int n=0; n<mergedVertex.size();n++){
							//System.out.println("Sorted Vertice angle :"+((Vertex)mergedVertex.get(n)).angle);
							//mergedVertex.get(n).print();
						//}
						
						//Make new objects
						Stack convexhull = new Stack();
						convexhull.push(mergedVertex.get(mergedVertex.size()-1));	//last vertex
						convexhull.push(mergedVertex.get(0));
						//((Vertex)convexhull.get(0)).print();   //top-1
						//((Vertex)convexhull.get(1)).print();   //top
		
						for(int n=1; n<mergedVertex.size();n++){
							//check the point is left using ( (b.x - a.x)*(c.y - a.y)-(c.x - a.x)*(b.y - a.y));c=new point, a=top-1, b=top
							//System.out.println(((Vertex)convexhull.lastElement()).x);
							//System.out.println(((Vertex)convexhull.get(convexhull.size()-2)).x);
							//System.out.println((((Vertex)convexhull.get(1)).x - ((Vertex)convexhull.get(0)).x)*(mergedVertex.get(n).y - ((Vertex)convexhull.get(0)).y)-(mergedVertex.get(n).x - ((Vertex)convexhull.get(0)).x)*(((Vertex)convexhull.get(1)).y - ((Vertex)convexhull.get(0)).y));
							//System.out.println((((Vertex)convexhull.lastElement()).x - ((Vertex)convexhull.get(convexhull.size()-2)).x)*(mergedVertex.get(n).y - ((Vertex)convexhull.get(convexhull.size()-2)).y)-(mergedVertex.get(n).x - ((Vertex)convexhull.get(convexhull.size()-2)).x)*(((Vertex)convexhull.lastElement()).y - ((Vertex)convexhull.get(convexhull.size()-2)).y));
							if((((Vertex)convexhull.lastElement()).x - ((Vertex)convexhull.get(convexhull.size()-2)).x)*(mergedVertex.get(n).y - ((Vertex)convexhull.get(convexhull.size()-2)).y)-(mergedVertex.get(n).x - ((Vertex)convexhull.get(convexhull.size()-2)).x)*(((Vertex)convexhull.lastElement()).y - ((Vertex)convexhull.get(convexhull.size()-2)).y)<0){
								convexhull.push(mergedVertex.get(n));
							}else{
								convexhull.pop();
								n--;
							}
						}
						for(int c=0;c<convexhull.size();c++){
							if(c==(convexhull.size()-1)){
								o.addEdge(new Edge((Vertex)convexhull.get(c),(Vertex)convexhull.get(0)));
							}else{
								o.addEdge(new Edge((Vertex)convexhull.get(c),(Vertex)convexhull.get(c+1)));
							}
						}
		
						//for(int z=0;z<o.getEdgeArray().length;z++){
						//	System.out.println(o.getEdgeArray()[z].toString());
						//}
						
						//Delete the object and add object
						if(i>CrossedObject[i]){
							gObstacles.remove(i);
							gObstacles.remove(CrossedObject[i]);
						}else{
							gObstacles.remove(CrossedObject[i]);
							gObstacles.remove(i);
						}
						gObstacles.add(o);
						if(CrossedObject[CrossedObject[i]]==i){
							CrossedObject[CrossedObject[i]]=0;
						    CrossedObject[i]=0;
						}
						break;
					}
				}
			}
		}while(isCrossedObject);
		
		//Debug
		//System.out.println("Grown obstacles size : "+gObstacles.size());
		//System.out.println("Original Obstacles");
		//for(int i = 1; i < origObstacles.size(); i++){
		//	for(int j=0;j<((ArrayList)((Obstacle)origObstacles.get(i)).listVertices()).size();j++)
		//		((Vertex)((ArrayList)((Obstacle)origObstacles.get(i)).listVertices()).get(j)).print();
		//}
		//System.out.println("Grown Obstacles");
		//for(int i = 1; i < gObstacles.size(); i++){
		//	for(int j=0;j<((ArrayList)((Obstacle)gObstacles.get(i)).listVertices()).size();j++)
		//		((Vertex)((ArrayList)((Obstacle)gObstacles.get(i)).listVertices()).get(j)).print();
		//}
		return gObstacles;
	}
	
	public boolean checkCrossBoundary(Edge e){
		boolean isCrossedBoundary=false;
		for(int j=0;j<((Obstacle)gObstacles.get(0)).edges.size();j++){
			if(lineSegmentIntersection(e.v1,e.v2,((Edge)((Obstacle)gObstacles.get(0)).edges.get(j)))){
				isCrossedBoundary=true;
				break;
			}
		}
		return isCrossedBoundary; 
	}
	
	//Implement Algorithm 5 Rotational Plane Sweep Algorithm in the text book (page 114)
	public ArrayList makeVisibilityGraph_version1(ArrayList gObstacles)
	{
		ArrayList vertices = new ArrayList();
		vGraph = new ArrayList<Edge>();
		
		//debug
		System.out.println(gObstacles.size());
		System.out.println(gObstacles.toString());
		
		//Read all vertex from all grown objects
		vertices.add(start);
		vertices.add(goal);
		for(int i = 1; i < gObstacles.size(); i++){
			for(int j=0;j<((ArrayList)((Obstacle)gObstacles.get(i)).listVertices()).size();j++){
				vertices.add(((ArrayList)((Obstacle)gObstacles.get(i)).listVertices()).get(j));//make the list of vertices from all obstacles
			}
		}
		//debug
		System.out.println("Number of vertice : "+vertices.size());
		for(int i=0; i<vertices.size();i++)
			((Vertex)vertices.get(i)).print();
		
		for(int m=0; m<5;m++){
			//Calculate angle for each vertex and sort by angle
			System.out.println("m="+m+";"+((Vertex)vertices.get(m)).x+","+((Vertex)vertices.get(m)).y);
			
			for(int i=0; i<vertices.size();i++){
				((Vertex)vertices.get(i)).angleFromStart((Vertex)vertices.get(m));
				//System.out.println(((Vertex)vertices.get(i)).angle);
			}
			
			//Sort the Vertices by angle in increasing order
			Vertex tempVertex = new Vertex(0,0);
			ArrayList sortedVertices = new ArrayList();
			//sortedVertices = vertices;
			for(int i=0;i<vertices.size();i++)
				sortedVertices.add((Vertex)vertices.get(i));
				
			for(int i=0; i<sortedVertices.size();i++)
				for(int j=0; j<(sortedVertices.size()-1);j++)
					if(((Vertex)sortedVertices.get(j)).angle>((Vertex)sortedVertices.get(j+1)).angle){
						tempVertex=((Vertex)sortedVertices.get(j+1));
						sortedVertices.set(j+1, (Vertex)sortedVertices.get(j));
						sortedVertices.set(j,tempVertex);
					}
			//for(int i=0; i<sortedVertices.size();i++){
			//	System.out.println("Sorted Vertice angle :"+((Vertex)sortedVertices.get(i)).angle);
			//}
			
			//Create the active list S, containing the sorted list of edges that intersect the horizontal half-line emanating from start point
			ArrayList<Edge> SweepLines = new ArrayList<Edge>();
			ArrayList<Edge> NotSweepLines = new ArrayList<Edge>();
			boolean isIdenticalObject;
			for(int i = 1; i < gObstacles.size(); i++){			//Except for boundary : gObstacles[0]
				//System.out.println(((Obstacle)gObstacles.get(i)).edges.size());
				isIdenticalObject=false;
				for(int k=0;k<((Obstacle)gObstacles.get(i)).edges.size();k++){
					if(((Obstacle)gObstacles.get(i)).getEdgeArray()[k].equals((Vertex)vertices.get(m))){
						isIdenticalObject=true;
						break;
					}
				}
				if(isIdenticalObject==false){	
					for(int j=0;j<((Obstacle)gObstacles.get(i)).edges.size();j++){
						//System.out.println(start.y);
						//System.out.println(((Edge)((Obstacle)gObstacles.get(i)).edges.get(j)).toString());
						//System.out.println(((Edge)((Obstacle)gObstacles.get(i)).edges.get(j)).getMaxY());
						//System.out.println(((Edge)((Obstacle)gObstacles.get(i)).edges.get(j)).getMinY());
						if(((Vertex)vertices.get(m)).y<=((Edge)((Obstacle)gObstacles.get(i)).edges.get(j)).getMaxY() &&
								((Vertex)vertices.get(m)).y>=((Edge)((Obstacle)gObstacles.get(i)).edges.get(j)).getMinY())
							SweepLines.add((Edge)((Obstacle)gObstacles.get(i)).edges.get(j));//make the list of SweepLine from all obstacles
						else
							NotSweepLines.add((Edge)((Obstacle)gObstacles.get(i)).edges.get(j));//make the list of NotSweepLine from all obstacles
						
					}
				}
			}
			
			System.out.println("No. of Sweep Line : "+ SweepLines.size());
			for(int i = 0; i < SweepLines.size(); i++){
				System.out.println("Sweep Line : "+(SweepLines.get(i)).toString());
			}
			
			//Rotational Plane Sweep Algorithm
			int[] temp = new int[100];
			boolean isIntersection=false;
			for(int i=1; i<sortedVertices.size();i++){
				if(findObjectNumber((Vertex)sortedVertices.get(i))!=findObjectNumber((Vertex)sortedVertices.get(0))){
					//if v_i is visible to v then add the edge (v,v_i) to the visibility graph
					for (int j=0;j<SweepLines.size();j++){
						//check it out intersection b/w edge(v,v_i) and SweepLines
						if(lineSegmentIntersection((Vertex)sortedVertices.get(0),(Vertex)sortedVertices.get(i),(Edge)SweepLines.get(j))){
							isIntersection=true;
							break;
						}
						//if((Vertex)vertices.get(i))
					}
					//Add Visibility graph
					if(isIntersection==false){
						//check identical vGraph
						boolean isIndentialGraph=false;
						if(vGraph.size()>=1){
							int tempSize=vGraph.size();
							for(int n=0;n<tempSize;n++){
						//		if((((((Edge)vGraph.get(n)).v1.x==((Vertex)sortedVertices.get(0)).x)&&(((Edge)vGraph.get(n)).v1.y==((Vertex)sortedVertices.get(0)).y))
						//			&&(((Edge)vGraph.get(n)).v2.x==((Vertex)sortedVertices.get(i)).x)&&(((Edge)vGraph.get(n)).v2.y==((Vertex)sortedVertices.get(i)).y))
						//			)//||
								   //((((((Edge)vGraph.get(n)).v2.x==((Vertex)sortedVertices.get(0)).x)&&(((Edge)vGraph.get(n)).v2.y==((Vertex)sortedVertices.get(0)).y))
									//&&((Edge)vGraph.get(n)).v1.x==((Vertex)sortedVertices.get(i)).x)&&(((Edge)vGraph.get(n)).v1.y==((Vertex)sortedVertices.get(i)).y)))
						//		{
								if((((Edge)vGraph.get(n)).v1.equals((Vertex)sortedVertices.get(0))&&((Edge)vGraph.get(n)).v2.equals((Vertex)sortedVertices.get(i)))
								   ||
								   (((Edge)vGraph.get(n)).v2.equals((Vertex)sortedVertices.get(0))&&((Edge)vGraph.get(n)).v1.equals((Vertex)sortedVertices.get(i)))){
									isIndentialGraph=true;
									break;
								}
							}
							if(isIndentialGraph==false){
					//			System.out.println("("+((Vertex)sortedVertices.get(0)).x + ","+((Vertex)sortedVertices.get(0)).y+")");
					//			System.out.println("("+((Vertex)sortedVertices.get(i)).x+","+((Vertex)sortedVertices.get(i)).y+")");
					//			System.out.println(((Edge)vGraph.get(n)).v1.x+","+((Edge)vGraph.get(n)).v1.y);
					//			System.out.println(((Edge)vGraph.get(n)).v2.x+","+((Edge)vGraph.get(n)).v2.y);
								vGraph.add(new Edge(((Vertex)sortedVertices.get(0)),(Vertex)sortedVertices.get(i)));
							}
						}else{
							vGraph.add(new Edge(((Vertex)sortedVertices.get(0)),(Vertex)sortedVertices.get(i)));
						}
						
					}
						
					isIntersection=false;
					
					// if v_i is the beginning of an edge, not in S then Insert the E into S
					// debug here!!
					// exact definition of end of Edge
					int k=0;
					for (int j=0;j<NotSweepLines.size();j++){
						if(sortedVertices.get(i).equals((Vertex)(((Edge)NotSweepLines.get(j)).v1))){
							SweepLines.add((Edge)NotSweepLines.get(j));
							temp[k]=j;
							k++;
						}	
					}
					for (int j=0;j<k;j++)
						NotSweepLines.remove(temp[j]);
					
					// if v_i is the end of an edge in S,  delete the edge from S
					k=0;
					for (int j=0;j<SweepLines.size();j++){
						if(sortedVertices.get(i).equals((Vertex)(((Edge)SweepLines.get(j)).v2))){
							NotSweepLines.add((Edge)SweepLines.get(j));
							temp[k]=j;
							k++;
						}	
					}
					for (int j=0;j<k;j++)
						SweepLines.remove(temp[j]);
				}
			}
		}
		return vGraph;
	}
	
	//Implement simple algorithm to find out vGraph
	public ArrayList makeVisibilityGraph(ArrayList gObstacles)
	{
		ArrayList vertices = new ArrayList();
		vGraph = new ArrayList<Edge>();
		
		//debug
		//System.out.println(gObstacles.size());
		//System.out.println(gObstacles.toString());
		
		//Read all vertex from all grown objects
		vertices.add(start);
		vertices.add(goal);
		for(int i = 1; i < gObstacles.size(); i++){
			for(int j=0;j<((ArrayList)((Obstacle)gObstacles.get(i)).listVertices()).size();j++){
				vertices.add(((ArrayList)((Obstacle)gObstacles.get(i)).listVertices()).get(j));//make the list of vertices from all obstacles
			}
		}
		//debug
		//System.out.println("Number of vertice : "+vertices.size());
		//for(int i=0; i<vertices.size();i++)
		//	((Vertex)vertices.get(i)).print();
		
		//Find whole vGraph
		vGraph.add(new Edge(start,goal));
		for(int i=0; i<vertices.size();i++){
			for(int j=0; j<vertices.size();j++){
				if(i!=j){
					if(findObjectNumber((Vertex)vertices.get(i))!=findObjectNumber((Vertex)vertices.get(j))){
						boolean isIndentialGraph=false;
						if(vGraph.size()>=1){
							int tempSize=vGraph.size();
							for(int n=0;n<tempSize;n++){
								if((((Edge)vGraph.get(n)).v1.equals((Vertex)vertices.get(i))&&((Edge)vGraph.get(n)).v2.equals((Vertex)vertices.get(j)))
								   ||
								   (((Edge)vGraph.get(n)).v2.equals((Vertex)vertices.get(i))&&((Edge)vGraph.get(n)).v1.equals((Vertex)vertices.get(j)))){
									isIndentialGraph=true;
									break;
								}
							}
							if(isIndentialGraph==false){
					//			System.out.println("("+((Vertex)sortedVertices.get(0)).x + ","+((Vertex)sortedVertices.get(0)).y+")");
					//			System.out.println("("+((Vertex)sortedVertices.get(i)).x+","+((Vertex)sortedVertices.get(i)).y+")");
					//			System.out.println(((Edge)vGraph.get(n)).v1.x+","+((Edge)vGraph.get(n)).v1.y);
					//			System.out.println(((Edge)vGraph.get(n)).v2.x+","+((Edge)vGraph.get(n)).v2.y);
								vGraph.add(new Edge(((Vertex)vertices.get(i)),(Vertex)vertices.get(j)));
							}
						}else{
							vGraph.add(new Edge(((Vertex)vertices.get(i)),(Vertex)vertices.get(j)));
						}
					}
				}
			}
		}
		
		//for(int i=0;i<vGraph.size();i++)
		//	System.out.println("vGraph "+i+":"+vGraph.get(i).toString());
		
		//Check it out intersection b/w vGraph and edge of obstacle
		int[] temp = new int[1000];
		int k=0;
		boolean isRemoved;
		for(int m=0; m<vGraph.size();m++){
			isRemoved=false;
			for(int i = 0; i < gObstacles.size(); i++){			
				//System.out.println(((Obstacle)gObstacles.get(i)).edges.size());
				for(int j=0;j<((Obstacle)gObstacles.get(i)).edges.size();j++){
					if(lineSegmentIntersection(((Edge)vGraph.get(m)).v1,((Edge)vGraph.get(m)).v2,((Edge)((Obstacle)gObstacles.get(i)).edges.get(j)))){
						//vGraph.remove(m);
						temp[k]=m;
						k++;
						isRemoved=true;
						break;
					}
				}
				if(isRemoved)
					break;
			}
		}
		
		for (int j=k-1;j>=0;j--)
			vGraph.remove(temp[j]);
		
		temp=null;
		
		//Add edges of obstacles to vGraph
		for(int i = 1; i < gObstacles.size(); i++){		//Except for boundary : gObstacles[0]
			for(int j=0;j<((Obstacle)gObstacles.get(i)).edges.size();j++)
				vGraph.add(((Edge)((Obstacle)gObstacles.get(i)).edges.get(j)));
		}
		
		return vGraph;
	}
	
	int findObjectNumber(Vertex v){
		int i;
		boolean isFind=false;
		for(i = 1; i < gObstacles.size(); i++){			//Except for boundary : gObstacles[0]
			for(int k=0;k<((Obstacle)gObstacles.get(i)).edges.size();k++){
				if(v.equals(((Obstacle)gObstacles.get(i)).getEdgeArray()[k].v1) || v.equals(((Obstacle)gObstacles.get(i)).getEdgeArray()[k].v2)){
					isFind=true;
					break;
				}
			}
			if(isFind)
				break;
		}
		
		if(isFind){
			//System.out.println("Object Number : " + i);
			return i;
		}else{
			//System.out.println("Object Number : " + 0);
			return 0;
		}
	}
	
	boolean lineSegmentIntersection(Vertex _start, Vertex end, Edge other_line)
    {
        float denom = ((other_line.v2.y - other_line.v1.y)*(end.x - _start.x)) -
                      ((other_line.v2.x - other_line.v1.x)*(end.y - _start.y));

        float nume_a = ((other_line.v2.x - other_line.v1.x)*(_start.y - other_line.v1.y)) -
                       ((other_line.v2.y - other_line.v1.y)*(_start.x - other_line.v1.x));

        float nume_b = ((end.x - _start.x)*(_start.y - other_line.v1.y)) -
                       ((end.y - _start.y)*(_start.x - other_line.v1.x));

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

        if(ua >0 && ua <0.9999 && ub >0 && ub <0.9999)
        {
            // Get the intersection point.
            //intersection.x_ = begin_.x_ + ua*(end_.x_ - begin_.x_);
            //intersection.y_ = begin_.y_ + ua*(end_.y_ - begin_.y_);

            return true; //INTERESECTING;
        }

        return false; //NOT_INTERESECTING;
    }
	
	Vertex lineSegmentIntersectionVertex(Vertex _start, Vertex end, Edge other_line)
    {
		Vertex v=new Vertex(0,0);
		
        float denom = ((other_line.v2.y - other_line.v1.y)*(end.x - _start.x)) -
                      ((other_line.v2.x - other_line.v1.x)*(end.y - _start.y));

        float nume_a = ((other_line.v2.x - other_line.v1.x)*(_start.y - other_line.v1.y)) -
                       ((other_line.v2.y - other_line.v1.y)*(_start.x - other_line.v1.x));

        float nume_b = ((end.x - _start.x)*(_start.y - other_line.v1.y)) -
                       ((end.y - _start.y)*(_start.x - other_line.v1.x));

        if(denom == 0)
        {
            if((nume_a == 0) && (nume_b == 0))
            {
                return v; //COINCIDENT;
            }
            return v; //PARALLEL;
        }

        float ua = nume_a / denom;
        float ub = nume_b / denom;

        if(ua >0 && ua <0.9999 && ub >0 && ub <0.9999)
        {
            // Get the intersection point.
            v.x = _start.x + ua*(end.x - _start.x);
            v.y = _start.y + ua*(end.y - _start.y);
            v.x=((int)(v.x*1000000))/(float)1000000.0;
            v.y=((int)(v.y*1000000))/(float)1000000.0;
            return v; //INTERESECTING;
        }

        return v; //NOT_INTERESECTING;
    }
}