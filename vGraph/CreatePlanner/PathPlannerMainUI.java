package CreatePlanner;

import java.awt.BasicStroke;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.ComponentEvent;
import java.awt.event.ComponentListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;
import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.Map;
import javax.swing.JCheckBoxMenuItem;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JMenuItem;
import javax.swing.JPanel;
import javax.swing.Timer;
import javax.swing.event.MenuEvent;
import javax.swing.event.MenuListener;
import javax.swing.filechooser.FileFilter;


/**
 * Main GUI for Path Planner
 * @author David
 *
 */
public class PathPlannerMainUI extends JFrame implements MouseListener, MouseMotionListener, ActionListener, MenuListener, ComponentListener
{
	public final static String MENUITEM_FILE_EXIT = "Exit";
	//public final static String MENUITEM_LOAD_STARTGOAL = "Start/Goal Points";
	public final static String MENUITEM_LOAD_OBSTACLES = "Obstacles & Star/Goal";	
	public final static String MENUITEM_VIEW_STARTGOALOBS = "Start & Goal";
	public final static String MENUITEM_VIEW_OBSTACLE = "Obstacle Set";
	public final static String MENUITEM_VIEW_GROWNOBSTACLE = "Grown Obstacle Set";
	public final static String MENUITEM_VIEW_VISIBILITYGRAPH = "Visibility Graph";
	public final static String MENUITEM_VIEW_SAFEPATH = "Safe Path";
	public final static String MENUITEM_RUN_FOLLOWVIS = "Follow Visibility Path";
	public final static String MENUITEM_RRT_LOAD = "Open Obstacles & Start/Goal";
	public final static String MENUITEM_RRT_RUN = "RUN RRT";
	
	private boolean drawVisStartGoal;
	private boolean drawVisGrownObstacles;
	private boolean drawVisGraph;
	private boolean drawVisSafePath; 
	
	public final static String WINDOW_TITLE = "Create Path Planner";
	private final static int BORDER = 20; 
	private float xScale, yScale, xShift, yShift; 
	ArrayList lt = new ArrayList();
	ControlPanel cp = new ControlPanel(lt);
	VisibilityGraph vg;
	
	RapidlyExploringRandomTree rrt;
	String obstaclesFileRRT;
	String startGoalFileRRT; 
	private boolean drawRRTStartGoal=false;
	
	JPanel mainPanel;
	SceneMap scenemap;
	int pathDrawC; 
	
	/**
	 * @param args either zero or one arguements passed
	 * args[0] = IP of PER to connect to 
	 * if no arguements are passed to command line, then the PER is not connected to
	 */
	public static void main(String[] args) {
		PathPlannerMainUI ui = new PathPlannerMainUI(); 
		//ui.controller = new PERController(); 
	}

	/**
	 * Setup the GUI
	 *
	 */
	public PathPlannerMainUI() { 
		super(PathPlannerMainUI.WINDOW_TITLE);
		
		this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		this.pathDrawC = 0; 
		mainPanel = new JPanel(new BorderLayout());
		mainPanel.setMinimumSize(new Dimension(320,240));
		this.setContentPane(mainPanel);
		
		JPanel drawingPanel = new JPanel(new BorderLayout());
		drawingPanel.setMinimumSize(new Dimension(300, 220));
		drawingPanel.setMaximumSize(new Dimension(300, 220));
		drawingPanel.setBackground(Color.WHITE);
		
		this.drawVisGraph = this.drawVisGrownObstacles = this.drawVisSafePath = this.drawVisStartGoal = true; 
		mainPanel.setBackground(Color.WHITE);
		mainPanel.setMaximumSize(new Dimension(640,640));
		mainPanel.setOpaque(true);
		scenemap = new SceneMap(); 
		this.addComponentListener(this);
		this.generateMenu();
		
		this.setSize(new Dimension(640,640));
		
		/**
		 * Hard-coded for debugging, can also rely on GUI 
		 */
		//runVisibility("hw3_world_obstacles.txt", "hw3_start_goal.txt"); 
		
		this.show();
		repaint(); 
	}
	
	public void paint(Graphics g) 
	{
		super.paint(g);
		Graphics2D g2 = (Graphics2D) mainPanel.getGraphics();
		
		g2.setPaint(new Color(91,45,45));
		if (vg!=null) {
			paintVisibility(g2);
		}
		if(rrt!=null){
			paintRRT(g2);
		}
	}
	
	/**
	 * Utilizese the minimum and maximum obstacle vertices to determine
	 * the proper location and scale when drawing the environment. 
	 *
	 */
	private void calculateVisShiftScale() { 
		float worldWidth, worldHeight, screenWidth, screenHeight;
		float xM, yM; 
		
		worldWidth = (scenemap.maxX-scenemap.minX);
		worldHeight = (scenemap.maxY-scenemap.minY);
		xM = scenemap.maxX - worldWidth/2;
		yM = scenemap.maxY - worldHeight/2; 
		screenWidth = mainPanel.getWidth() - 2 * PathPlannerMainUI.BORDER;
		screenHeight = mainPanel.getHeight() - 2 * PathPlannerMainUI.BORDER;
		xScale = (float) (((mainPanel.getWidth() - 2*PathPlannerMainUI.BORDER)/(scenemap.maxX-scenemap.minX))* 1);
		yScale = (float) (((mainPanel.getHeight() - 2*PathPlannerMainUI.BORDER)/(scenemap.maxY-scenemap.minY)) * .75);
		xShift = PathPlannerMainUI.BORDER; 
		yShift = PathPlannerMainUI.BORDER; 
	}
	
	
	private void paintVisibility(Graphics2D g2) {
		this.calculateVisShiftScale();
		
		Iterator it = scenemap.obstacles.iterator();
		Obstacle o;
		g2.setPaint(new Color(91,45,45));
			
		//paint visibility graph 
		g2.setColor(Color.BLUE);
		if (this.drawVisGraph && vg.getVisibilityGraph()!=null) {
			Edge visGraph[] = (Edge[]) vg.getVisibilityGraph().toArray(new Edge[0]);
			this.drawEdges(visGraph, g2);
		}
		
		//paint visibility's grown obstacles 
		g2.setPaint(new Color(91,45,45));
		if (this.drawVisGrownObstacles && vg.getGrownObstacles()!=null) {
			it = vg.getGrownObstacles().iterator(); 
			while (it.hasNext()) {
				o = (Obstacle) it.next();
				Edge[] e = o.getEdgeArray();
				this.drawEdges(e, g2);
			
			}
		}
		
		if (this.drawVisStartGoal) {
			g2.setPaint(Color.cyan);
			it = vg.origObstacles.iterator();
			
			while (it.hasNext()) {
				o = (Obstacle) it.next();
				Edge[] e = o.getEdgeArray();
				this.drawEdges(e, g2);
			}
			
			//paint start/goal
			Vertex start, goal;
			start = vg.start;
			g2.setPaint(Color.red); 
			drawCircle(vg.start, g2);
			drawCircle(vg.goal, g2);
		}
		
		//draw safe path
		if (this.drawVisSafePath && vg.shortestPath!=null) {
			g2.setPaint(Color.red);
			//it = this.vg.shortestPath.iterator();
			Edge[] e = (Edge[]) vg.shortestPath.toArray(new Edge[0]);
			drawEdges(e, g2);
		}
	}
	private void paintRRT(Graphics2D g2) {
		this.calculateVisShiftScale();
		
		Iterator it = scenemap.obstacles.iterator();
		Obstacle o;
		//g2.setPaint(new Color(91,45,45));
		//Draw Tree from start point 
				
		if (this.drawRRTStartGoal) {
			g2.setPaint(Color.gray);
			it = rrt.origObstacles.iterator();
			//int i=0;
			while (it.hasNext()) {
				o = (Obstacle) it.next();
				Edge[] e = o.getEdgeArray();
				//if(i!=0)
					this.drawEdges(e, g2);
				//i++;
			}
			
			//paint start/goal
			Vertex start, goal;
			start = rrt.start;
			g2.setPaint(Color.red); 
			drawCircle(rrt.start, g2);
			drawCircle(rrt.goal, g2);
		}
		
		//Draw Tree from goal point 
		if (rrt.mergedEdges!=null) {
			g2.setPaint(Color.red);
			Edge goalTree[] = (Edge[]) rrt.mergedEdges.toArray(new Edge[0]);
			this.drawEdges(goalTree, g2);
		}
		
		if (rrt.startEdges!=null) {
			g2.setPaint(Color.blue);
			Edge startTree[] = (Edge[]) rrt.startEdges.toArray(new Edge[0]);
			this.drawEdges(startTree, g2);
		}
		
		//Draw Tree from goal point 
		if (rrt.goalEdges!=null) {
			g2.setPaint(Color.green);
			Edge goalTree[] = (Edge[]) rrt.goalEdges.toArray(new Edge[0]);
			this.drawEdges(goalTree, g2);
		}
		
		//Draw shortest path 
		if (rrt.shortestEdges!=null) {
			g2.setPaint(Color.red);
			g2.setStroke(new BasicStroke(2));
			Edge goalTree[] = (Edge[]) rrt.shortestEdges.toArray(new Edge[0]);
			this.drawEdges(goalTree, g2);
		}
	}
	
	private void drawCircle(Vertex v, Graphics2D g2) {
		int x1,x2,y1,y2; 
		
		x1 = (int)((v.x-scenemap.minX)*xScale+xShift);
		y1 = (int)((v.y-scenemap.minY)*yScale+yShift);
		g2.setPaint(Color.red); 
		g2.fillOval(x1-8, y1-8, 16, 16);
		g2.setPaint(Color.white);
		g2.fillOval(x1-5, y1-5, 10, 10);
		g2.setPaint(Color.red);
		g2.fillOval(x1-3,y1-3, 6, 6);
		g2.setPaint(Color.white);
		g2.fillOval(x1-1,y1-1,2,2);
	}
	
	private void drawEdges(Edge[] e, Graphics2D g2) {
		Vertex v1, v2; 
		int x1,x2,y1,y2; 
		for (int i=0; i<e.length; i++) {
			
			v1 = e[i].v1;
			v2 = e[i].v2; 
			
			x1 = (int)((v1.x-scenemap.minX)*xScale+xShift);
			y1 = (int)((v1.y-scenemap.minY)*yScale+yShift);
			x2 = (int)((v2.x-scenemap.minX)*xScale+xShift);
			y2 = (int)((v2.y-scenemap.minY)*yScale+yShift);
			g2.drawLine(x1,y1,x2,y2); 
		}
	}
	
	
		
	//helper method for generateMenu to create item, add listener, and add to menu
	private void addMenuItem(JMenu m, String s) { 
		JMenuItem i = new JMenuItem(s);
		m.add(i);
		i.addActionListener(this);
	}
	private void addCheckBoxMenuItem(JMenu m, String s) { 
		JMenuItem i = new JCheckBoxMenuItem(s, true); //default state is true
		m.add(i);
		i.addActionListener(this);
	}
	private void generateMenu() { 
		JMenuBar menuBar = new JMenuBar();
		JMenu fileMenu = new JMenu("File");
		fileMenu.addActionListener(this);
		addMenuItem(fileMenu, MENUITEM_FILE_EXIT);
		menuBar.add(fileMenu);
		
		JMenu loadMenu = new JMenu("Load");
		loadMenu.addActionListener(this);
		addMenuItem(loadMenu, MENUITEM_LOAD_OBSTACLES);
		//addMenuItem(loadMenu, MENUITEM_LOAD_STARTGOAL);
		
		menuBar.add(loadMenu);
		loadMenu.addMenuListener(this);
		
		JMenu viewMenu = new JMenu("View");
		viewMenu.addActionListener(this);
		addCheckBoxMenuItem(viewMenu, MENUITEM_VIEW_STARTGOALOBS);
		addCheckBoxMenuItem(viewMenu, MENUITEM_VIEW_GROWNOBSTACLE);
		addCheckBoxMenuItem(viewMenu, MENUITEM_VIEW_VISIBILITYGRAPH);
		addCheckBoxMenuItem(viewMenu, MENUITEM_VIEW_SAFEPATH);
		menuBar.add(viewMenu);
		viewMenu.addMenuListener(this);
		
		JMenu runMenu = new JMenu("Run");
		addMenuItem(runMenu, MENUITEM_RUN_FOLLOWVIS);
		menuBar.add(runMenu);
		runMenu.addActionListener(this);
		
		JMenu rrtMenu = new JMenu("RRT");
		addMenuItem(rrtMenu, MENUITEM_RRT_LOAD);
		addMenuItem(rrtMenu, MENUITEM_RRT_RUN);
		menuBar.add(rrtMenu);
		runMenu.addActionListener(this);
		
		this.setJMenuBar(menuBar);
	}
		
	private String getFile() { 
		JFileChooser chooser = new JFileChooser("."); //System.getProperty("java.class.path"));
	  
	    int returnVal = chooser.showOpenDialog(this);
	    if(returnVal == JFileChooser.APPROVE_OPTION) {
	       System.out.println("You chose to open this file: " +
	            chooser.getSelectedFile().getAbsolutePath());
	       return chooser.getSelectedFile().getAbsolutePath();
	    }
	    return null; 
	}
	
	/**
	 * Prompt user for obstacles and then start-goal file
	 * and then run the Visibility graph algorithm. 
	 *
	 */
	public void runVisibility() { 
		String obstaclesFile = getFile();
		String startGoalFile = getFile(); 
		runVisibility(obstaclesFile, startGoalFile); 
	}
	
	public void runVisibility(String obstaclesFile, String startGoalFile) {
		if (obstaclesFile==null || startGoalFile==null) {
			System.out.println("No file selected");
			return; 
		}
		this.vg = new VisibilityGraph();
		//compute the visibility graph (expand obstacles, apply dijkstra's to expanded edges to get shortest path) 
		vg.computeVisibilityGraph(obstaclesFile, startGoalFile); 
		
		//get the grown obstacles and add them to the scenemap (used when painting this frame's canvas)
		ArrayList edges;
		edges = vg.getGrownObstacles();
		//scenemap.visibilityPath = vg.getVisibilityGraph();
		if (edges!=null) { 
			Iterator it = edges.iterator();
			while (it.hasNext()) {
				scenemap.addObstacle((Obstacle) it.next());
			}
		}
	}
	
	//given an event presumed to be from a jcheckboxmenuitem, it will return whether the menu item is checked or unchecked
	private boolean getEventCheckBoxVal(ActionEvent a) {
		JCheckBoxMenuItem i = (JCheckBoxMenuItem) a.getSource();
		return i.getState();
	}
	
	/**
	 * RRT : Open Obstacles and Start/Goal
	 */  
	public void openObstaclesStartGoal(){
		this.obstaclesFileRRT = getFile();
		this.startGoalFileRRT = getFile();
		if (this.obstaclesFileRRT==null || this.startGoalFileRRT==null) {
			System.out.println("No file selected");
			return; 
		}
		this.drawRRTStartGoal=true;
		this.rrt=new RapidlyExploringRandomTree();
		rrt.drawEnvironments(this.obstaclesFileRRT,this.startGoalFileRRT);
		
		ArrayList edges;
		edges = rrt.getObstacles();
		//scenemap.visibilityPath = vg.getVisibilityGraph();
		if (edges!=null) { 
			Iterator it = edges.iterator();
			while (it.hasNext()) {
				scenemap.addObstacle((Obstacle) it.next());
			}
		}
	}
	/**
	 * RRT : Compute RRT
	 */
	public void runRRT(){
		if (this.obstaclesFileRRT==null || this.startGoalFileRRT==null) {
			System.out.println("No file selected");
			return; 
		}
		rrt.computeRRT();
	}
	
	/**
	 * Main action handler (all generated by the Menu items)
	 */
	public void actionPerformed(ActionEvent arg0) {
		String event = arg0.getActionCommand();
		if (event==this.MENUITEM_FILE_EXIT) {
			System.exit(0);
		}
		else if (event==this.MENUITEM_LOAD_OBSTACLES) { 
			runVisibility();
		}
		else if (event==this.MENUITEM_VIEW_STARTGOALOBS) {
			this.drawVisStartGoal = getEventCheckBoxVal(arg0);
		}
		else if (event==this.MENUITEM_VIEW_GROWNOBSTACLE) {
			this.drawVisGrownObstacles = getEventCheckBoxVal(arg0);
		}
		else if (event==this.MENUITEM_VIEW_VISIBILITYGRAPH) {
			this.drawVisGraph= getEventCheckBoxVal(arg0);
		}
		else if (event==this.MENUITEM_VIEW_SAFEPATH) {
			this.drawVisSafePath = getEventCheckBoxVal(arg0);
		}else if (event==this.MENUITEM_RUN_FOLLOWVIS) {
/*			Thread currThread; 
			currThread = new Thread(){
	            public void run(){
	                controller.followPath(vg.waypoints);
	            }
	        };
	        //run the command
	        try{
	            currThread.start();
	        }catch (java.lang.IllegalThreadStateException e){}
*/	        
//	        controller.followPath(vg.waypoints);
			cp.createAndShowGUI(vg.waypoints);
		}else if (event==this.MENUITEM_RRT_LOAD){
			//System.out.println("Load Obstacles and Start/Goal");
			openObstaclesStartGoal();
		}else if (event==this.MENUITEM_RRT_RUN){
			runRRT();
		}
		this.repaint();
	}

	public void mouseClicked(MouseEvent arg0) {
		// TODO Auto-generated method stub	
	}

	public void mousePressed(MouseEvent arg0) {
		// TODO Auto-generated method stub
	}

	public void mouseReleased(MouseEvent arg0) {
		// TODO Auto-generated method stub
	}

	public void mouseEntered(MouseEvent arg0) {
		// TODO Auto-generated method stub
	}

	public void mouseExited(MouseEvent arg0) {
		// TODO Auto-generated method stub
	}

	public void mouseDragged(MouseEvent arg0) {
		// TODO Auto-generated method stub
	}

	public void mouseMoved(MouseEvent arg0) {
		// TODO Auto-generated method stub
	}

	public void menuSelected(MenuEvent arg0) {
		// TODO Auto-generated method stub
	}

	public void menuDeselected(MenuEvent arg0) {
		// TODO Auto-generated method stub
	}

	public void menuCanceled(MenuEvent arg0) {
		// TODO Auto-generated method stub
	}

	public void componentResized(ComponentEvent arg0) {
		// TODO Auto-generated method stub
		this.repaint();
	}

	public void componentMoved(ComponentEvent arg0) {
		// TODO Auto-generated method stub
	}

	public void componentShown(ComponentEvent arg0) {
		// TODO Auto-generated method stub
	}

	public void componentHidden(ComponentEvent arg0) {
		// TODO Auto-generated method stub
	}
}
