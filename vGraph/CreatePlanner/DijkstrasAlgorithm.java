package CreatePlanner;

import java.util.*;

public class DijkstrasAlgorithm {
	
	/**
	 * Implement dijkstra's algorithm, calculating the shortest path between start and goal given the provided list of vertices
	 * @param verticesGraph
	 * @param start
	 * @param goal
	 * @return
	 */

	/**
	 *	 1  function Dijkstra(Graph, source):
		 2      for each vertex v in Graph:           // Initializations
		 3          dist[v] := infinity               // Unknown distance function from source to v
		 4          previous[v] := undefined          // Previous node in optimal path from source
		 5      dist[source] := 0                     // Distance from source to source
		 6      Q := the set of all nodes in Graph    // All nodes in the graph are unoptimized - thus are in Q
		 7      while Q is not empty:                 // The main loop
		 8          u := vertex in Q with smallest dist[]
		 9          remove u from Q
		10          for each neighbor v of u:         // where v has not yet been removed from Q.
		11              alt := dist[u] + dist_between(u, v)       // be careful in 1st step - dist[u] is infinity yet
		12              if alt < dist[v]              // Relax (u,v,a)
		13                  dist[v] := alt
		14                  previous[v] := u
		15      return previous[]
	 */
	public static ArrayList calculateShortestPath_version1(ArrayList verticesGraph, Vertex start, Vertex goal,ArrayList gObstacles) { 
		/**
		 * Insert your code here
		 */
		final int MAX_PATH=1500;//verticesGraph.size()*verticesGraph.size();
		final int MAX_LEVEL=100;
		//System.out.println("Maximum Path : "+MAX_PATH);
		//ArrayList<Edge>[] edgePath = new ArrayList[MAX_PATH];
		int[][] Path = new int[MAX_PATH][MAX_LEVEL];
		ArrayList<Edge> shortestPath = new ArrayList();
		double[] distance=new double[MAX_PATH];
		//int[] previousVertices=new int[MAX_PATH];
		int numberOfPath=0;
		Vertex goalDirection=new Vertex(0,0);
		boolean isGoalDirection=false;
		int[] subNode=new int[MAX_PATH];
		boolean[] isPathComplete=new boolean[MAX_PATH];
		double shortestDistance=0;
		int shortestPathNumber=0;
		
		goalDirection.x=goal.x-start.x;
		goalDirection.y=goal.y-start.y;
		
		//Rearrange vGraph
		Vertex tempV=new Vertex(0,0);
		for(int i=0;i<verticesGraph.size();i++){
			if(goalDirection.x>=0){
				if(((((Edge)verticesGraph.get(i)).v2.x-((Edge)verticesGraph.get(i)).v1.x) < 0)){
					tempV=((Edge)verticesGraph.get(i)).v2;
					((Edge)verticesGraph.get(i)).v2=((Edge)verticesGraph.get(i)).v1;
					((Edge)verticesGraph.get(i)).v1=tempV;
				}
			}else{
				if(((((Edge)verticesGraph.get(i)).v2.x-((Edge)verticesGraph.get(i)).v1.x) > 0)){
					tempV=((Edge)verticesGraph.get(i)).v2;
					((Edge)verticesGraph.get(i)).v2=((Edge)verticesGraph.get(i)).v1;
					((Edge)verticesGraph.get(i)).v1=tempV;
				}
			}
		}
		
		//for(int i=0;i<verticesGraph.size();i++)
		//	System.out.println("vGraph "+i+":"+verticesGraph.get(i).toString());
		
		for(int i=0;i<MAX_PATH;i++){
			for(int j=0;j<MAX_LEVEL;j++)
				Path[i][j]=0;
			subNode[i]=0;
			isPathComplete[i]=false;
			distance[i]=0;
			//edgePath[i].clear();
		}
		
		//Make All path
		for(int i=0;i<verticesGraph.size();i++){
			if(start.equals(((Edge)verticesGraph.get(i)).v1)){
				//Path[numberOfPath].add((Edge)(verticesGraph.get(i)));
				Path[numberOfPath][0]=i;
				subNode[numberOfPath]=1;
				//edgePath[numberOfPath].add((Edge)verticesGraph.get(i+1));
				numberOfPath++;
			}
		}
		
		int p=numberOfPath;
		int numberOfFindGoal=0;
		int level=1;
		boolean isContinue=false;
		do{
			isContinue=false;
			for(int m=0;m<p;m++){
				if(isPathComplete[m]==false){
					for(int i=0;i<verticesGraph.size();i++){
						if(((Edge)verticesGraph.get(Path[m][level-1])).v1.equals(((Edge)verticesGraph.get(i)).v1)			//check identical
							&&((Edge)verticesGraph.get(Path[m][level-1])).v2.equals(((Edge)verticesGraph.get(i)).v2)){
						}else{		
							if(((Edge)verticesGraph.get(Path[m][level-1])).v2.equals(((Edge)verticesGraph.get(i)).v1)
							   /*||((Edge)verticesGraph.get(Path[m][level-1])).v2.equals(((Edge)verticesGraph.get(i)).v2)*/){
								if(((Edge)verticesGraph.get(Path[m][level-1])).v2.equals(((Edge)verticesGraph.get(i)).v1)){
									if(((goalDirection.x>0)&&((((Edge)verticesGraph.get(i)).v2.x-((Edge)verticesGraph.get(i)).v1.x)>=0))){
										isGoalDirection=true;
									}else{
										isGoalDirection=false;
									}
								}else{
									if(((goalDirection.x>0)&&((((Edge)verticesGraph.get(i)).v1.x-((Edge)verticesGraph.get(i)).v2.x)>=0))){
										isGoalDirection=true;		
									}else{
										isGoalDirection=false;
									}
								}
								if(isGoalDirection==true){
									if(goal.equals(((Edge)verticesGraph.get(i)).v1)		//Check goal point
									  ||goal.equals(((Edge)verticesGraph.get(i)).v2)){
										/*if(((Edge)verticesGraph.get(Path[m][level-1])).v2.equals(((Edge)verticesGraph.get(i)).v1)){
											edgePath[numberOfPath].add((Edge)verticesGraph.get(i+1));
										}else{
											Edge tempEdge=new Edge(new Vertex(0,0),new Vertex(0,0));
											tempEdge.v1=((Edge)verticesGraph.get(i+1)).v2;
											tempEdge.v2=((Edge)verticesGraph.get(i+1)).v1;
											edgePath[numberOfPath].add(tempEdge);
										}*/
										
										Path[m][subNode[m]]=i;
										numberOfFindGoal++;
										subNode[m]++;
										isPathComplete[m]=true;
										break;
									}else{
										if(subNode[m]<=level){
											/*if(((Edge)verticesGraph.get(Path[m][level-1])).v2.equals(((Edge)verticesGraph.get(i)).v1)){
												edgePath[numberOfPath].add((Edge)verticesGraph.get(i));
											}else{
												Edge tempEdge=new Edge(new Vertex(0,0),new Vertex(0,0));
												tempEdge.v1=((Edge)verticesGraph.get(i+1)).v2;
												tempEdge.v2=((Edge)verticesGraph.get(i+1)).v1;
												edgePath[numberOfPath].add(tempEdge);
											}*/
											Path[m][subNode[m]]=i;
											subNode[m]++;
										}else{		//make new path
											for(int k=0;k<(subNode[m]-1);k++){
												//edgePath[numberOfPath].add((Edge)edgePath[m].get(k+1));
												Path[numberOfPath][k]=Path[m][k];
												subNode[numberOfPath]++;
											}
											/*if(((Edge)verticesGraph.get(Path[m][level-1])).v2.equals(((Edge)verticesGraph.get(i)).v1)){
												edgePath[numberOfPath].add((Edge)verticesGraph.get(i+1));
											}else{
												Edge tempEdge=new Edge(new Vertex(0,0),new Vertex(0,0));
												tempEdge.v1=((Edge)verticesGraph.get(i+1)).v2;
												tempEdge.v2=((Edge)verticesGraph.get(i+1)).v1;
												edgePath[numberOfPath].add(tempEdge);
											}*/
											Path[numberOfPath][(subNode[m]-1)]=i;
											subNode[numberOfPath]++;
											numberOfPath++;
										}
									}
								}
							}
						}
					}
				}
			}
			p=numberOfPath;
			level++;
			//System.out.println("numberOfFindGoal : " + numberOfFindGoal);
			//System.out.println("numberOfPaht : " + numberOfPath);
			//System.out.println("level :"+level);
			//for(int i=0;i<numberOfPath;i++)
			//	for(int j=0;j<subNode[i];j++)
			//		System.out.println("Path["+i+"]["+j+"] = "+Path[i][j]);
			for(int n=0;n<p;n++){
				if(isPathComplete[n]==false){
					isContinue=true;
					break;
				}
			}
			
		}while(isContinue);
		
		//for(int i=0;i<numberOfPath;i++)
		//	for(int j=0;j<subNode[i];j++)
		//		System.out.println("Path["+i+"]["+j+"] = "+Path[i][j]);
		
		//Find distance of all path
		for(int i=0;i<numberOfPath;i++){
			for(int j=0;j<subNode[i];j++)
				distance[i]+=((Edge)verticesGraph.get(Path[i][j])).v1.distanceToVertex(((Edge)verticesGraph.get(Path[i][j])).v2);
			if(i==0){
				shortestDistance=distance[0];
				shortestPathNumber=0;
			}else{
				if(shortestDistance>distance[i]){
					shortestDistance=distance[i];
					shortestPathNumber=i;
				}
			}
			//System.out.println("Distance["+i+"] = "+distance[i]);
		}
		//System.out.println("Shortest Distance["+shortestPathNumber+"] = "+distance[shortestPathNumber]);
		for(int i=0;i<subNode[shortestPathNumber];i++)
			shortestPath.add((Edge)verticesGraph.get(Path[shortestPathNumber][i]));
		
		
		return shortestPath;
		//return null;
	}
	
	public static ArrayList calculateShortestPath_version2(ArrayList verticesGraph, Vertex start, Vertex goal,ArrayList gObstacles) { 
		/**
		 * Insert your code here
		 */
		ArrayList<Edge> shortestPath = new ArrayList();
		ArrayList<Edge> Relaxed=new ArrayList();
		double tempDistance=0;
		double localShortestDistance=Integer.MAX_VALUE;
		int indexShortestEdge=0;
		Vertex goalDirection=new Vertex(0,0);
		int[] previousEdge=new int[verticesGraph.size()];
		double[] EdgeDistance = new double[verticesGraph.size()];
		
		goalDirection.x=goal.x-start.x;
		goalDirection.y=goal.y-start.y;
		
		//Rearrange vGraph
		Vertex tempV=new Vertex(0,0);
		for(int i=0;i<verticesGraph.size();i++){
			if(goalDirection.x>=0){
				if(((((Edge)verticesGraph.get(i)).v2.x-((Edge)verticesGraph.get(i)).v1.x) < 0)){
					tempV=((Edge)verticesGraph.get(i)).v2;
					((Edge)verticesGraph.get(i)).v2=((Edge)verticesGraph.get(i)).v1;
					((Edge)verticesGraph.get(i)).v1=tempV;
				}
			}else{
				if(((((Edge)verticesGraph.get(i)).v2.x-((Edge)verticesGraph.get(i)).v1.x) > 0)){
					tempV=((Edge)verticesGraph.get(i)).v2;
					((Edge)verticesGraph.get(i)).v2=((Edge)verticesGraph.get(i)).v1;
					((Edge)verticesGraph.get(i)).v1=tempV;
				}
			}
		}
		//Get vetices from the vGraph
		/*for(int i=0;i<verticesGraph.size();i++){	
			v.add(((Edge)verticesGraph.get(i)).v1);
			v.add(((Edge)verticesGraph.get(i)).v2);
		}
		for(int i=0;i<v.size();i++){
			for(int j=0;j<v.size();j++)
				if(v.get(i).equals(v.get(j)))
					v.remove(i);
		}
		for(int i=0;i<v.size();i++){
			System.out.println(v.get(i).x+","+v.get(i).y);
		}*/
		//Get EdgeDistance
		
		for(int i=0;i<verticesGraph.size();i++){
			EdgeDistance[i]=Integer.MAX_VALUE;//((Edge)verticesGraph.get(i)).v1.distanceToVertex(((Edge)verticesGraph.get(i)).v2);
			//System.out.println("EdgeDistance["+i+"] = "+EdgeDistance[i]);
		}
		
		//find shortest path from source to neighbors
		for(int i=0;i<verticesGraph.size();i++){
			if(((Edge)verticesGraph.get(i)).v1.equals(start)){
				EdgeDistance[i]=start.distanceToVertex(((Edge)verticesGraph.get(i)).v2);
				if(EdgeDistance[i]<localShortestDistance){
					localShortestDistance=EdgeDistance[i];
					indexShortestEdge=i;
				}
			}
			System.out.println("EdgeDistance["+i+"] = "+EdgeDistance[i]);
		}
		previousEdge[0]=indexShortestEdge;
		//previousEdge=indexShortestEdge;
		shortestPath.add((Edge)verticesGraph.get(indexShortestEdge));
		System.out.println("# of edge in shortest Path : "+shortestPath.size());
		System.out.println("EdgeDistance["+indexShortestEdge+"] = "+EdgeDistance[indexShortestEdge]);
		System.out.println("Previous Edge["+(shortestPath.size()-1)+"] = " + previousEdge[shortestPath.size()-1]);
		
		while(!goal.equals(((Edge)shortestPath.get(shortestPath.size()-1)).v2)){
			indexShortestEdge=0;
			localShortestDistance=Integer.MAX_VALUE;
			boolean previousFlag=false;
			
			//Find shortest path from the latest vertices to neighbors
			for(int i=0;i<verticesGraph.size();i++){
				previousFlag=false;
				if(((Edge)verticesGraph.get(i)).v1.equals(((Edge)shortestPath.get(shortestPath.size()-1)).v2)
					/*|| ((Edge)verticesGraph.get(i)).v2.equals(((Edge)shortestPath.get(shortestPath.size()-1)).v2)*/){
					for(int n=0;n<shortestPath.size();n++){
						if(previousEdge[n]==i)
							previousFlag=true;
					}
					if(previousFlag==false){
						tempDistance=((Edge)shortestPath.get(shortestPath.size()-1)).v2.distanceToVertex(((Edge)verticesGraph.get(i)).v2);
						if(tempDistance<localShortestDistance){
							localShortestDistance=tempDistance;
							indexShortestEdge=i;
							//EdgeDistance[indexShortestEdge]=tempDistance;
						}
					}
				}
			}
			
			//Update Edge distance from u
			//Debug here !!!
			boolean removeFlag=true;
			for(int i=0;i<verticesGraph.size();i++){
				if(i!=indexShortestEdge){
					if(((Edge)verticesGraph.get(i)).v2.equals(((Edge)verticesGraph.get(indexShortestEdge)).v2)
					   /*||((Edge)verticesGraph.get(i)).v2.equals(((Edge)verticesGraph.get(indexShortestEdge)).v1)*/){
						if(EdgeDistance[i]<localShortestDistance+EdgeDistance[previousEdge[shortestPath.size()-1]]){
							for(int k=0;k<shortestPath.size();k++)
								if(shortestPath.get(k).v1.equals(((Edge)verticesGraph.get(i)).v1)){
									for(int j=shortestPath.size()-1;j>=k;j--)
										shortestPath.remove(j);
									break;
								}
							shortestPath.add((Edge)verticesGraph.get(i));
							previousEdge[shortestPath.size()-1]=i;
							removeFlag=false;
							indexShortestEdge=i;
							break;
						}
					}
				}
			}
			
			if(removeFlag){
				EdgeDistance[indexShortestEdge]=localShortestDistance+EdgeDistance[previousEdge[shortestPath.size()]];
				shortestPath.add((Edge)verticesGraph.get(indexShortestEdge));
				previousEdge[shortestPath.size()-1]=indexShortestEdge;
			}
			System.out.println("# of edge in shortest Path : "+shortestPath.size());
			System.out.println("EdgeDistance["+indexShortestEdge+"] = "+EdgeDistance[indexShortestEdge]);
			System.out.println("Previous Edge["+(shortestPath.size()-1)+"] = " + previousEdge[shortestPath.size()-1]);
			//Check it relaxed vertices
			/*for(int i=0;i<verticesGraph.size();i++){
				for(int j=0;j<shortestPath.size();j++)
				if(((Edge)verticesGraph.get(i)).v1.equals(((Edge)shortestPath.get(j)).v1)
					&&((Edge)verticesGraph.get(i)).v2.equals(((Edge)verticesGraph.get(indexShortestEdge)).v2)){
					tempDistance=((Edge)verticesGraph.get(i)).v1.distanceToVertex(((Edge)verticesGraph.get(i)).v2);
					localShortestDistance=0;
					for(int k=j; k<shortestPath.size();k++)
						localShortestDistance+=((Edge)shortestPath.get(k)).v1.distanceToVertex(((Edge)shortestPath.get(k)).v2);
					localShortestDistance+=((Edge)verticesGraph.get(indexShortestEdge)).v1.distanceToVertex(((Edge)verticesGraph.get(indexShortestEdge)).v2);
					if(tempDistance<=localShortestDistance){
						localShortestDistance=tempDistance;
						indexShortestEdge=i;
					}
				}
			}*/
			
			//shortestPath.add((Edge)verticesGraph.get(indexShortestEdge));
			//System.out.println("# of edge in shortest Path : "+shortestPath.size());
			//Remove the previous vertex
		}
		
		//Show shortest Path
		for(int i=0;i<shortestPath.size();i++){
			System.out.println(((Edge)shortestPath.get(i)).toString());
		}
		
		return shortestPath;
	}

	//Dijkstra algorithm using the shorted path
	public static ArrayList calculateShortestPath_version3(ArrayList verticesGraph, Vertex start, Vertex goal,ArrayList gObstacles) { 
		/**
		 * Insert your code here
		 */
		ArrayList<Edge> shortestPath = new ArrayList();
		double tempDistance=0;
		double localShortestDistance=Integer.MAX_VALUE;
		int indexShortestEdge=0;
		Vertex goalDirection=new Vertex(0,0);
		ArrayList<Vertex> vertices = new ArrayList();
		Vertex localShortestVertex =new Vertex(0,0);
		//int localShortestVertexIndex=0;
		
		//Read all vertex from all grown objects
		vertices.add(start);
		vertices.add(goal);
		for(int i = 1; i < gObstacles.size(); i++){
			for(int j=0;j<((ArrayList)((Obstacle)gObstacles.get(i)).listVertices()).size();j++){
//				for(int k=0;k<verticesGraph.size();k++){
//					if(((Edge)verticesGraph.get(k)).v1.equals((Vertex) ((ArrayList)((Obstacle)gObstacles.get(i)).listVertices()).get(j))
//					 ||((Edge)verticesGraph.get(k)).v2.equals((Vertex) ((ArrayList)((Obstacle)gObstacles.get(i)).listVertices()).get(j))){
						vertices.add((Vertex) ((ArrayList)((Obstacle)gObstacles.get(i)).listVertices()).get(j));//make the list of vertices from all obstacles
//					}
//				}
			}
		}
		for(int i=0;i<vertices.size();i++){
			System.out.println(vertices.get(i).x+","+vertices.get(i).y);
		}
		
		boolean[] isVertexValid=new boolean[vertices.size()];
		//double[] EdgeDistance = new double[verticesGraph.size()];
		double[] VertexDistance = new double[vertices.size()];
		int[] previousVertex=new int[vertices.size()];
		
		for(int i=0;i<vertices.size();i++){
			VertexDistance[i]=Integer.MAX_VALUE;//((Edge)verticesGraph.get(i)).v1.distanceToVertex(((Edge)verticesGraph.get(i)).v2);
			//System.out.println("EdgeDistance["+i+"] = "+EdgeDistance[i]);
			isVertexValid[i]=true;
		}
		VertexDistance[0]=0;		//start point
		
		//Get EdgeDistance
		//for(int i=0;i<verticesGraph.size();i++){
		//	EdgeDistance[i]=Integer.MAX_VALUE;//((Edge)verticesGraph.get(i)).v1.distanceToVertex(((Edge)verticesGraph.get(i)).v2);
		//	//System.out.println("EdgeDistance["+i+"] = "+EdgeDistance[i]);
		//}
		
		//find shortest path from source to neighbors
		for(int i=0;i<verticesGraph.size();i++){
			if(((Edge)verticesGraph.get(i)).v1.equals(start)){
				for(int j=0;j<vertices.size();j++){
					if(((Edge)verticesGraph.get(i)).v2.equals(vertices.get(j))){
						VertexDistance[j]=VertexDistance[0]+start.distanceToVertex(((Edge)verticesGraph.get(i)).v2);
						if(VertexDistance[j]<localShortestDistance){
							localShortestDistance=VertexDistance[j];
							localShortestVertex=((Edge)verticesGraph.get(i)).v2;
							//localShortestVertexIndex=j;
						}
					}
				}
			}else if(((Edge)verticesGraph.get(i)).v2.equals(start)){
				for(int j=0;j<vertices.size();j++){
					if(((Edge)verticesGraph.get(i)).v1.equals(vertices.get(j))){
						VertexDistance[j]=VertexDistance[0]+start.distanceToVertex(((Edge)verticesGraph.get(i)).v1);
						if(VertexDistance[j]<localShortestDistance){
							localShortestDistance=VertexDistance[j];
							localShortestVertex=((Edge)verticesGraph.get(i)).v1;
							//localShortestVertexIndex=j;
						}
					}
				}
			}
		}	
		
		for(int j=0;j<vertices.size();j++){
			System.out.println("VertexDistance["+j+"] = "+VertexDistance[j]);
		}
		//EdgeDistance[previousEdge[0]]=localShortestDistance;
		
		//previousEdge=indexShortestEdge;
		shortestPath.add(new Edge(start,localShortestVertex));
		isVertexValid[vertices.indexOf(start)]=false;
		previousVertex[vertices.indexOf(localShortestVertex)]=vertices.indexOf(start);
		System.out.println("# of edge in shortest Path : "+shortestPath.size());
		System.out.println("Last vertex : ["+vertices.indexOf(localShortestVertex)+"] = ("+localShortestVertex.x+","+localShortestVertex.y+")");
		System.out.println("Previous vertex : ["+vertices.indexOf(localShortestVertex)+"] = "+vertices.indexOf(start));
		
		while(!goal.equals(((Edge)shortestPath.get(shortestPath.size()-1)).v2)){
			indexShortestEdge=0;
			localShortestDistance=Integer.MAX_VALUE;
			boolean replaceFlag=false;
			
			//Find shortest path from the latest vertices to neighbors
			for(int i=0;i<verticesGraph.size();i++){
//				previousFlag=false;
				if(((Edge)verticesGraph.get(i)).v1.equals(((Edge)shortestPath.get(shortestPath.size()-1)).v2)){
					if(isVertexValid[vertices.indexOf(((Edge)verticesGraph.get(i)).v2)]==true){
						if(((Edge)verticesGraph.get(i)).v2.x>=((Edge)shortestPath.get(shortestPath.size()-1)).v2.x){
							tempDistance=VertexDistance[vertices.indexOf(((Edge)shortestPath.get(shortestPath.size()-1)).v2)]+((Edge)shortestPath.get(shortestPath.size()-1)).v2.distanceToVertex(((Edge)verticesGraph.get(i)).v2);
							if(tempDistance<VertexDistance[vertices.indexOf(((Edge)verticesGraph.get(i)).v2)]){
								VertexDistance[vertices.indexOf(((Edge)verticesGraph.get(i)).v2)]=tempDistance;	
								previousVertex[vertices.indexOf(((Edge)verticesGraph.get(i)).v2)]=vertices.indexOf(((Edge)verticesGraph.get(i)).v1);
								if(VertexDistance[vertices.indexOf(((Edge)verticesGraph.get(i)).v2)]<localShortestDistance){
									localShortestDistance=tempDistance;
									localShortestVertex=((Edge)verticesGraph.get(i)).v2;
								}
							}else{
								if(VertexDistance[vertices.indexOf(((Edge)verticesGraph.get(i)).v2)]<localShortestDistance){
									localShortestDistance=VertexDistance[vertices.indexOf(((Edge)verticesGraph.get(i)).v2)];
									localShortestVertex=((Edge)verticesGraph.get(i)).v2;
									replaceFlag=true;
								}
							}
						}
					}
				}else if(((Edge)verticesGraph.get(i)).v2.equals(((Edge)shortestPath.get(shortestPath.size()-1)).v2)){
					if(isVertexValid[vertices.indexOf(((Edge)verticesGraph.get(i)).v1)]==true){
						if(((Edge)verticesGraph.get(i)).v1.x>=((Edge)shortestPath.get(shortestPath.size()-1)).v2.x){
							tempDistance=VertexDistance[vertices.indexOf(((Edge)shortestPath.get(shortestPath.size()-1)).v2)]+((Edge)shortestPath.get(shortestPath.size()-1)).v2.distanceToVertex(((Edge)verticesGraph.get(i)).v1);
							if(tempDistance<VertexDistance[vertices.indexOf(((Edge)verticesGraph.get(i)).v1)]){
								VertexDistance[vertices.indexOf(((Edge)verticesGraph.get(i)).v1)]=tempDistance;
								previousVertex[vertices.indexOf(((Edge)verticesGraph.get(i)).v1)]=vertices.indexOf(((Edge)verticesGraph.get(i)).v2);
								if(VertexDistance[vertices.indexOf(((Edge)verticesGraph.get(i)).v1)]<localShortestDistance){
									localShortestDistance=tempDistance;
									localShortestVertex=((Edge)verticesGraph.get(i)).v1;
								}
							}else{
								if(VertexDistance[vertices.indexOf(((Edge)verticesGraph.get(i)).v1)]<localShortestDistance){
									localShortestDistance=VertexDistance[vertices.indexOf(((Edge)verticesGraph.get(i)).v1)];
									localShortestVertex=((Edge)verticesGraph.get(i)).v1;
									replaceFlag=true;
								}
							}
						}
					}
				}
			}
			
			//Update Edge distance from u
			//Debug here !!!
			int replaceIndex=0;
			boolean cutFlag=true;
			if(replaceFlag){
				isVertexValid[vertices.indexOf(((Edge)shortestPath.get(shortestPath.size()-1)).v2)]=false;
				/*for(int k=0;k<shortestPath.size();k++){
					if(shortestPath.get(k).v1.equals(vertices.get(previousVertex[vertices.indexOf(localShortestVertex)]))){
						replaceIndex=k;
						break;
					}
				}
				
				for(int j=shortestPath.size()-1;j>=replaceIndex;j--)
					shortestPath.remove(j);
				*/
				shortestPath.clear();
				
				int[] nextVertex=new int[vertices.size()];
				int m=0;
				nextVertex[m]=vertices.indexOf(localShortestVertex);
				if(previousVertex[nextVertex[m]]==0){
					shortestPath.add(new Edge(start,localShortestVertex));
					isVertexValid[0]=false;
				}else{
					while(true){
						if(previousVertex[nextVertex[m]]==0){
							break;
						}else{
							nextVertex[m+1]=previousVertex[nextVertex[m]];
						}
						m++;
					}
					for(int n=m;n>=0;n--){
						shortestPath.add(new Edge(vertices.get(previousVertex[nextVertex[n]]),vertices.get(nextVertex[n])));
						isVertexValid[nextVertex[n]]=false;
					}
				}
			}else{
				shortestPath.add(new Edge(((Edge)shortestPath.get(shortestPath.size()-1)).v2,localShortestVertex));
				isVertexValid[vertices.indexOf(((Edge)shortestPath.get(shortestPath.size()-1)).v1)]=false;
			}
				
			System.out.println("# of edge in shortest Path : "+shortestPath.size());
			System.out.println("Last vertex : ["+vertices.indexOf(localShortestVertex)+"] = ("+localShortestVertex.x+","+localShortestVertex.y+")");
			System.out.println("Previous vertex : ["+vertices.indexOf(localShortestVertex)+"] = "+previousVertex[vertices.indexOf(localShortestVertex)]);
			System.out.println(replaceFlag);
			System.out.println("----------------------------");
		}
		
		//Show shortest Path
		for(int i=0;i<shortestPath.size();i++){
			System.out.println(((Edge)shortestPath.get(i)).toString());
		}
		
		return shortestPath;
	}
	
	public static ArrayList calculateShortestPath(ArrayList verticesGraph, Vertex start, Vertex goal,ArrayList gObstacles) { 
		/**
		 * Insert your code here
		 */
		ArrayList<Edge> shortestPath = new ArrayList();
		double tempDistance=0;
		double localShortestDistance=Integer.MAX_VALUE;
		int indexShortestEdge=0;
		Vertex goalDirection=new Vertex(0,0);
		ArrayList<Vertex> vertices = new ArrayList();
		Vertex localShortestVertex =new Vertex(0,0);
		
		//Read all vertex from all grown objects
		vertices.add(start);
		vertices.add(goal);
		for(int i = 1; i < gObstacles.size(); i++){
			for(int j=0;j<((ArrayList)((Obstacle)gObstacles.get(i)).listVertices()).size();j++){
				vertices.add((Vertex) ((ArrayList)((Obstacle)gObstacles.get(i)).listVertices()).get(j));//make the list of vertices from all obstacles
			}
		}
		//for(int i=0;i<vertices.size();i++){
		//	System.out.println(vertices.get(i).x+","+vertices.get(i).y);
		//}
		
		boolean[] isVertexValid=new boolean[vertices.size()];
		double[] VertexDistance = new double[vertices.size()];
		int[] previousVertex=new int[vertices.size()];
		int currentVertex=0;
		
		for(int i=0;i<vertices.size();i++){
			VertexDistance[i]=Integer.MAX_VALUE;//((Edge)verticesGraph.get(i)).v1.distanceToVertex(((Edge)verticesGraph.get(i)).v2);
			isVertexValid[i]=true;
		}
		VertexDistance[0]=0;		//start point
		
		
		while(!goal.equals(vertices.get(currentVertex))){
			indexShortestEdge=0;
			localShortestDistance=Integer.MAX_VALUE;
			boolean replaceFlag=false;
			
			//Find smallest distance vertex
			for(int i=0;i<vertices.size();i++){
				if(isVertexValid[i]==true){
					if(VertexDistance[i]<localShortestDistance){
						localShortestDistance=VertexDistance[i];
						currentVertex=i;
					}
				}
			}
			
			//Find shortest path from the latest vertices to neighbors
			for(int i=0;i<verticesGraph.size();i++){
//				previousFlag=false;
				if(((Edge)verticesGraph.get(i)).v1.equals(vertices.get(currentVertex))){
					if(isVertexValid[vertices.indexOf(((Edge)verticesGraph.get(i)).v2)]==true){
						tempDistance=VertexDistance[currentVertex]+vertices.get(currentVertex).distanceToVertex(((Edge)verticesGraph.get(i)).v2);
						if(tempDistance<VertexDistance[vertices.indexOf(((Edge)verticesGraph.get(i)).v2)]){
							VertexDistance[vertices.indexOf(((Edge)verticesGraph.get(i)).v2)]=tempDistance;	
							previousVertex[vertices.indexOf(((Edge)verticesGraph.get(i)).v2)]=vertices.indexOf(((Edge)verticesGraph.get(i)).v1);
						}
					}
				}else if(((Edge)verticesGraph.get(i)).v2.equals(vertices.get(currentVertex))){
					if(isVertexValid[vertices.indexOf(((Edge)verticesGraph.get(i)).v1)]==true){
						tempDistance=VertexDistance[currentVertex]+vertices.get(currentVertex).distanceToVertex(((Edge)verticesGraph.get(i)).v1);
						if(tempDistance<VertexDistance[vertices.indexOf(((Edge)verticesGraph.get(i)).v1)]){
							VertexDistance[vertices.indexOf(((Edge)verticesGraph.get(i)).v1)]=tempDistance;	
							previousVertex[vertices.indexOf(((Edge)verticesGraph.get(i)).v1)]=vertices.indexOf(((Edge)verticesGraph.get(i)).v2);
						}
					}
				}
			}	
			isVertexValid[currentVertex]=false;
		}
		
		//make shortest Path
		shortestPath.add(0, new Edge(vertices.get(previousVertex[vertices.indexOf(goal)]),goal));
		while(!shortestPath.get(0).v1.equals(start)){
			shortestPath.add(0, new Edge(vertices.get(previousVertex[vertices.indexOf(shortestPath.get(0).v1)]),shortestPath.get(0).v1));
		}

		//Show shortest Path
		//for(int i=0;i<shortestPath.size();i++){
		//	System.out.println(((Edge)shortestPath.get(i)).toString());
		//}
		
		return shortestPath;
	}
}
