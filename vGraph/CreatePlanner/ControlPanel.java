package CreatePlanner;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.*;


import java.awt.*;
import java.util.ArrayList;

/**
 * @author Joseph
 * @date 10/23/2008
 */
public class ControlPanel extends JPanel implements ActionListener,Runnable {

	protected JButton Up,Down,Left,Right,Stop, Open,Mode,GoDistance,TurnAngle,MakeSquare,GoVelocity,ReadSensors, WallFollow,Bug2,vGraph;
	protected JPanel p1,p2,p3,p4,p5;
	protected JTextField WheelDropCaster, WheelDropRight, WheelDropLeft, BumpLeft, BumpRight, CliffLeft, CliffRight, CliffFrontLeft, CliffFrontRight, Wall;
	protected final int Max2Bytes = 32768;	// 32767 = 0x8000
	protected final int Min2Bytes = -32767;	// -32767 = 0xFFFF
	//protected int RightVelocity;
	//protected int LeftVelocity;
	protected final int MaxVelocity = 500; // 500 mm/s
	protected final int MinVelocity = -500; // -500 mm/s
	protected final int DefaultVelocity = 200;
	boolean Wallfollowstop = false;
	protected final byte ON=(byte)1;
	protected final byte OFF=(byte)0;
	protected Point Create = new Point(0,0);
	protected double CreateOrientation = 0;
	protected final int DIAMETER=280;		/*Dimension : mm*/
	long startTime=0;
	protected boolean isTimerStarted=false;
	OpenComPort iRobotBAM = new OpenComPort();
	protected int algorithmIndex=0;
	boolean Bug2stop = false;
	boolean vGraphstop = false;
	byte previousDrive=0;
	int previousVr=0;
	int previousVl=0;
	float previousAngle=0;
	final float PI=(float)3.14;
	boolean isHitObject=false;
	final double ANGLE_CAL_RATIO=0.94; 	// 95% of nominal angle
	final int GOAL_DISTANCE=5000; //5000; 5M
	Point createLocal =new Point(0,0);
	public ArrayList<Point> localtarget=new ArrayList();
	
    public ControlPanel(ArrayList<Point> lt) {
    	for(int i=0; i<lt.size();i++)
    		this.localtarget.add(lt.get(i));
    	
    	Open = new JButton("Open");
       	Open.setActionCommand("Open");

    	Mode = new JButton("Mode");
    	Mode.setActionCommand("Mode");
    	Mode.setEnabled(false);
    	
    	Up = new JButton("Go");
    	Up.setActionCommand("Go");
    	Up.setEnabled(false);
    	
    	Down = new JButton("Back");
    	Down.setActionCommand("Back");
    	Down.setEnabled(false);
    	
    	Left = new JButton("Turn Left");
    	Left.setActionCommand("Turn Left");
    	Left.setEnabled(false);
    	
    	Right = new JButton("Turn Right");
    	Right.setActionCommand("Turn Right");
    	Right.setEnabled(false);
    	
    	Stop = new JButton("Stop");
    	Stop.setActionCommand("Stop");
    	Stop.setEnabled(false);
    	
    	GoDistance = new JButton("Go Distance");
    	GoDistance.setActionCommand("Go Distance");
    	GoDistance.setEnabled(false);
    	
    	TurnAngle = new JButton("Turn Angle");
    	TurnAngle.setActionCommand("Turn Angle");
    	TurnAngle.setEnabled(false);
    	
    	MakeSquare = new JButton("Make Square");
    	MakeSquare.setActionCommand("Make Square");
    	MakeSquare.setEnabled(false);
    	
    	GoVelocity = new JButton("Go Velocity");
    	GoVelocity.setActionCommand("Go Velocity");
    	GoVelocity.setEnabled(false);
    	
    	ReadSensors = new JButton("Read Sensors");
    	ReadSensors.setActionCommand("Read Sensors");
    	ReadSensors.setEnabled(false);
    	
    	WheelDropCaster =new JTextField(null,1);
    	WheelDropRight =new JTextField(null,1);
    	WheelDropLeft =new JTextField(null,1);
    	BumpLeft =new JTextField(null,1);
    	BumpRight =new JTextField(null,1);
    	CliffLeft =new JTextField(null,1);
    	CliffRight =new JTextField(null,1);
    	CliffFrontLeft =new JTextField(null,1);
    	CliffFrontRight =new JTextField(null,1);
    	Wall =new JTextField(null,4);
    	
    	WallFollow = new JButton("Wall Follow");
    	WallFollow.setActionCommand("Wall Follow");
    	WallFollow.setEnabled(false);
    	Bug2 = new JButton("Bug2");
    	Bug2.setActionCommand("Bug2");
    	Bug2.setEnabled(false);
    	vGraph = new JButton("Follow vGraph");
    	vGraph.setActionCommand("Follow vGraph");
    	vGraph.setEnabled(false);
    	
    	
        //Listen for actions on buttons 1 and 3.
    	Open.addActionListener(this);
    	Mode.addActionListener(this);
    	Up.addActionListener(this);
    	Down.addActionListener(this);
    	Left.addActionListener(this);
    	Right.addActionListener(this);
    	Stop.addActionListener(this);
    	GoDistance.addActionListener(this);
    	TurnAngle.addActionListener(this);
    	MakeSquare.addActionListener(this);
    	GoVelocity.addActionListener(this);
    	ReadSensors.addActionListener(this);
    	WallFollow.addActionListener(this);
    	Bug2.addActionListener(this);
    	vGraph.addActionListener(this);

       	p1=new JPanel(new BorderLayout());
    	p2=new JPanel(new BorderLayout());
    	p3=new JPanel(new BorderLayout());
    	SpringLayout slayout=new SpringLayout();
    	p4=new JPanel(slayout);
    	p5=new JPanel(new BorderLayout());
    	
    	//Add Components to this container, using the default FlowLayout.
        p1.add(Open,BorderLayout.WEST);
        p1.add(Mode,BorderLayout.EAST);
        p2.add(Up,BorderLayout.NORTH);
        p2.add(Down,BorderLayout.SOUTH);
        p2.add(Left,BorderLayout.WEST);
        p2.add(Right,BorderLayout.EAST);
        p2.add(Stop,BorderLayout.CENTER);
        p3.add(GoDistance,BorderLayout.NORTH);
        p3.add(TurnAngle,BorderLayout.EAST);
        p3.add(MakeSquare,BorderLayout.CENTER);
        p3.add(GoVelocity,BorderLayout.WEST);
        p3.add(ReadSensors,BorderLayout.SOUTH); 
        JLabel labelWheelDropCaster=new JLabel("Wheel Drop Caster : ");
        p4.add(labelWheelDropCaster);
        p4.add(WheelDropCaster);
        JLabel labelWheelDropRight=new JLabel("Wheel Drop Right : ");
        p4.add(labelWheelDropRight);
        p4.add(WheelDropRight);
        JLabel labelWheelDropLeft=new JLabel("Wheel Drop Left : ");
        p4.add(labelWheelDropLeft);
        p4.add(WheelDropLeft);
        JLabel labelBumpLeft=new JLabel("Bump Left : ");
        p4.add(labelBumpLeft);
        p4.add(BumpLeft);
        JLabel labelBumpRight=new JLabel("Bump Right : ");
        p4.add(labelBumpRight);
        p4.add(BumpRight);
        JLabel labelCliffLeft=new JLabel("Cliff Left : ");
        p4.add(labelCliffLeft);
        p4.add(CliffLeft);
        JLabel labelCliffRight=new JLabel("Cliff Right : ");
        p4.add(labelCliffRight);
        p4.add(CliffRight);
        JLabel labelCliffFrontLeft=new JLabel("Cliff Front Left : ");
        p4.add(labelCliffFrontLeft);
        p4.add(CliffFrontLeft);
        JLabel labelCliffFrontRight=new JLabel("Cliff Front Right : ");
        p4.add(labelCliffFrontRight);
        p4.add(CliffFrontRight);
        JLabel labelWall=new JLabel("Wall : ");
        p4.add(labelWall);
        p4.add(Wall);
        
        slayout.putConstraint(SpringLayout.WEST, labelWheelDropCaster, 5, SpringLayout.WEST, p4);
        slayout.putConstraint(SpringLayout.NORTH, labelWheelDropCaster, 5,SpringLayout.NORTH, p4);
        slayout.putConstraint(SpringLayout.WEST, WheelDropCaster, 5, SpringLayout.EAST, labelWheelDropCaster);
        slayout.putConstraint(SpringLayout.NORTH, WheelDropCaster, 5,SpringLayout.NORTH, p4);
        
        slayout.putConstraint(SpringLayout.WEST, labelWheelDropRight, 5, SpringLayout.WEST, p4);
        slayout.putConstraint(SpringLayout.NORTH, labelWheelDropRight, 9,SpringLayout.SOUTH, labelWheelDropCaster);
        slayout.putConstraint(SpringLayout.WEST, WheelDropRight, 5, SpringLayout.EAST, labelWheelDropRight);
        slayout.putConstraint(SpringLayout.NORTH, WheelDropRight, 5,SpringLayout.SOUTH, WheelDropCaster);
        
        slayout.putConstraint(SpringLayout.WEST, labelWheelDropLeft, 5, SpringLayout.WEST, p4);
        slayout.putConstraint(SpringLayout.NORTH, labelWheelDropLeft, 9,SpringLayout.SOUTH, labelWheelDropRight);
        slayout.putConstraint(SpringLayout.WEST, WheelDropLeft, 5, SpringLayout.EAST, labelWheelDropLeft);
        slayout.putConstraint(SpringLayout.NORTH, WheelDropLeft, 5,SpringLayout.SOUTH, WheelDropRight);
        
        slayout.putConstraint(SpringLayout.WEST, labelBumpLeft, 5, SpringLayout.WEST, p4);
        slayout.putConstraint(SpringLayout.NORTH, labelBumpLeft, 9,SpringLayout.SOUTH, labelWheelDropLeft);
        slayout.putConstraint(SpringLayout.WEST, BumpLeft, 5, SpringLayout.EAST, labelBumpLeft);
        slayout.putConstraint(SpringLayout.NORTH, BumpLeft, 5,SpringLayout.SOUTH, WheelDropLeft);
        
        slayout.putConstraint(SpringLayout.WEST, labelBumpRight, 5, SpringLayout.WEST, p4);
        slayout.putConstraint(SpringLayout.NORTH, labelBumpRight, 9,SpringLayout.SOUTH, labelBumpLeft);
        slayout.putConstraint(SpringLayout.WEST, BumpRight, 5, SpringLayout.EAST, labelBumpRight);
        slayout.putConstraint(SpringLayout.NORTH, BumpRight, 5,SpringLayout.SOUTH, BumpLeft);
        
        slayout.putConstraint(SpringLayout.WEST, labelCliffLeft, 5, SpringLayout.WEST, p4);
        slayout.putConstraint(SpringLayout.NORTH, labelCliffLeft, 9,SpringLayout.SOUTH, labelBumpRight);
        slayout.putConstraint(SpringLayout.WEST, CliffLeft, 5, SpringLayout.EAST, labelCliffLeft);
        slayout.putConstraint(SpringLayout.NORTH, CliffLeft, 5,SpringLayout.SOUTH, BumpRight);
        
        slayout.putConstraint(SpringLayout.WEST, labelCliffRight, 5, SpringLayout.WEST, p4);
        slayout.putConstraint(SpringLayout.NORTH, labelCliffRight, 9,SpringLayout.SOUTH, labelCliffLeft);
        slayout.putConstraint(SpringLayout.WEST, CliffRight, 5, SpringLayout.EAST, labelCliffRight);
        slayout.putConstraint(SpringLayout.NORTH, CliffRight, 5,SpringLayout.SOUTH, CliffLeft);
        
        slayout.putConstraint(SpringLayout.WEST, labelCliffFrontLeft, 5, SpringLayout.WEST, p4);
        slayout.putConstraint(SpringLayout.NORTH, labelCliffFrontLeft, 9,SpringLayout.SOUTH, labelCliffRight);
        slayout.putConstraint(SpringLayout.WEST, CliffFrontLeft, 5, SpringLayout.EAST, labelCliffFrontLeft);
        slayout.putConstraint(SpringLayout.NORTH, CliffFrontLeft, 5,SpringLayout.SOUTH, CliffRight);
        
        slayout.putConstraint(SpringLayout.WEST, labelCliffFrontRight, 5, SpringLayout.WEST, p4);
        slayout.putConstraint(SpringLayout.NORTH, labelCliffFrontRight, 9,SpringLayout.SOUTH, labelCliffFrontLeft);
        slayout.putConstraint(SpringLayout.WEST, CliffFrontRight, 5, SpringLayout.EAST, labelCliffFrontRight);
        slayout.putConstraint(SpringLayout.NORTH, CliffFrontRight, 5,SpringLayout.SOUTH, CliffFrontLeft);
        
        slayout.putConstraint(SpringLayout.WEST, labelWall, 5, SpringLayout.WEST, p4);
        slayout.putConstraint(SpringLayout.NORTH, labelWall, 9,SpringLayout.SOUTH, labelCliffFrontRight);
        slayout.putConstraint(SpringLayout.WEST, Wall, 5, SpringLayout.EAST, labelWall);
        slayout.putConstraint(SpringLayout.NORTH, Wall, 5,SpringLayout.SOUTH, CliffFrontRight);
        
        slayout.putConstraint(SpringLayout.SOUTH, p4, 5, SpringLayout.SOUTH, Wall);
        
        p5.add(WallFollow,BorderLayout.NORTH);
        p5.add(Bug2,BorderLayout.CENTER);
        p5.add(vGraph,BorderLayout.SOUTH);
        
        //setLayout(new BorderLayout());
        setLayout(new BoxLayout(this,BoxLayout.Y_AXIS));
        add(p1);
        add(p2);
        add(p3);
        add(p4);
        add(p5);
    }

    /**
     * Create the GUI and show it.  For thread safety, 
     * this method should be invoked from the 
     * event-dispatching thread.
     */
    public void createAndShowGUI(ArrayList<Point> lt) {
  	
        //Create and set up the window.
        JFrame frame = new JFrame("ControlPanel");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        //Create and set up the content pane.
        ControlPanel newContentPane = new ControlPanel(lt);
        newContentPane.setOpaque(true); //content panes must be opaque
        frame.setContentPane(newContentPane);

        //Display the window.
        frame.pack();
        frame.setVisible(true);
    }
	
	/* (non-Javadoc)
	 * @see java.awt.event.ActionListener#actionPerformed(java.awt.event.ActionEvent)
	 */
	@Override
	public void actionPerformed(ActionEvent e) {
		// TODO Auto-generated method stub
		if ("Open".equals(e.getActionCommand())) {
			Mode.setEnabled(true);
			if(iRobotBAM.SetupBAM()==0){
				Open.setEnabled(false);
			}
		}
		if ("Mode".equals(e.getActionCommand())) {
			
			SetMode();
			LED_On();
						
			Up.setEnabled(true);
			Down.setEnabled(true);
			Right.setEnabled(true);
			Left.setEnabled(true);
			Stop.setEnabled(true);
		    GoDistance.setEnabled(true);
		    TurnAngle.setEnabled(true);
		    MakeSquare.setEnabled(true);
		    GoVelocity.setEnabled(true);
		    ReadSensors.setEnabled(true);
		    WallFollow.setEnabled(true);
		    Bug2.setEnabled(true);
		    vGraph.setEnabled(true);
		}
		
		if ("Go".equals(e.getActionCommand())) {
			/*byte[] data = new byte[5];  

            data[0] = (byte)145;  //Direct Drive command
            data[1] = 0;    //[Right velocity high byte] 
            data[2] = (byte)200;  //[Right velocity low byte]
            data[3] = 0;    //[Left velocity high byte]
            data[4] = (byte)200;  //[Left velocity low byte]
            
            iRobotBAM.Write(data);*/
			CreatDirectDrive(200,200);
		}
		
		if ("Back".equals(e.getActionCommand())) {
			byte[] data = new byte[5];

            data[0] = (byte)145;  //Direct Drive command
            data[1] = (byte)((-200)>>8 & 0x00FF);  //[Right velocity high byte] 
            data[2] = (byte)((-200) & 0x00FF);    //[Right velocity low byte]
            data[3] = (byte)((-200) >> 8 & 0x00FF);  //[Left velocity high byte]
            data[4] = (byte)((-200) & 0x00FF);    //[Left velocity low byte]
            
            iRobotBAM.Write(data);
		}
		
		if ("Turn Right".equals(e.getActionCommand())) {
			byte[] data = new byte[5];

            data[0] = (byte)145;  //Direct Drive command
            data[1] = (byte)((-200) >> 8 & 0x00FF);  //[Right velocity high byte] 
            data[2] = (byte)((-200) & 0x00FF);    //[Right velocity low byte]
            data[3] = 0;  //[Left velocity high byte]
            data[4] = (byte)200;    //[Left velocity low byte]

            iRobotBAM.Write(data);
		}
		
		if ("Turn Left".equals(e.getActionCommand())) {
			byte[] data = new byte[5];

            data[0] = (byte)145;  //Direct Drive command
            data[1] = (byte)0;  //[Right velocity high byte] 
            data[2] = (byte)200;    //[Right velocity low byte]
            data[3] = (byte)((-200) >> 8 & 0x00FF);  //[Left velocity high byte]
            data[4] = (byte)((-200) & 0x00FF);    //[Left velocity low byte]
            
            iRobotBAM.Write(data);
		}
		
		if ("Stop".equals(e.getActionCommand())) {
			byte[] data = new byte[5];

            data[0] = (byte)145;  //Direct Drive command
            data[1] = 0;    //[Right velocity high byte] 
            data[2] = 0;  //[Right velocity low byte]
            data[3] = 0;    //[Left velocity high byte]
            data[4] = 0;  //[Left velocity low byte]
            
            iRobotBAM.Write(data);
            if(algorithmIndex==1)
            	Wallfollowstop = false;
            else  if(algorithmIndex==2)
            	Bug2stop = false;
            else  if(algorithmIndex==3)
            	vGraphstop = false;
            	
		}
		
		//Additional functions by Soonhac Hong from Here
		//prompts for a distance in mm's which it then traverses 
		if ("Go Distance".equals(e.getActionCommand())) {
			byte[] data = new byte[5];  
			int distance=0;
			boolean DataValid = false;
			
			//Get Distance from dialog box
			do{
				String s = (String)JOptionPane.showInputDialog("Distance(-32767 ~ 32768) [mm] : ",JOptionPane.QUESTION_MESSAGE);
				
				if ((s != null) && (s.length() > 0)) {
					distance=Integer.parseInt(s);
					if((distance<=Max2Bytes)&&(distance>=Min2Bytes)){ 		//check data validation
						DataValid=true;
					}else{
						JOptionPane.showMessageDialog(null,"Velocity is out of range","Message",JOptionPane.WARNING_MESSAGE);
					}
				}else{
					distance=0;
					DataValid=true;
				}
			}while(!DataValid);
			
			if(distance!=0){
				//Go distance
				if(distance>0){
					CreatDirectDrive(DefaultVelocity,DefaultVelocity);
				}else{
					CreatDirectDrive(-DefaultVelocity,-DefaultVelocity);
				}
				//CreatDirectDrive(RightVelocity,LeftVelocity);
					            
				//Wait distance
	            data[0] = (byte)156;  //Wait distance
	            data[1] = (byte)((distance >> 8) & 0x00FF);  //[Distance high byte] 
	            data[2] = (byte)(distance & 0x00FF);    //[Distance low byte]
	            iRobotBAM.Write(data);
	            
	            //Stop
	            CreatDirectDrive(0,0);
			}
		}

		//same as go but with a user specified input velocity 
		if ("Go Velocity".equals(e.getActionCommand())) {
			boolean DataValid = false;
			int inputdata=0;
			//Get Distance from dialog box
			do{
				String s = (String)JOptionPane.showInputDialog("Distance(-500 ~ 500) [mm/s] : ",JOptionPane.QUESTION_MESSAGE);
				
				if ((s != null) && (s.length() > 0)) {
					inputdata=Integer.parseInt(s);
					if((inputdata<=MaxVelocity)&&(inputdata>=MinVelocity)){ 		//check data validation
						DataValid=true;
					}else{
						JOptionPane.showMessageDialog(null,"Velocity is out of range","Message",JOptionPane.WARNING_MESSAGE);
					}
				}else{
					inputdata=0;
					DataValid=true;
				}
			}while(!DataValid);
			
			if(inputdata!=0){
				//Go Velocity
				CreatDirectDrive(inputdata,inputdata);
			}
		}
		
		//turns the robot that many degrees, positive counterclockwise, negative clockwise 
		if ("Turn Angle".equals(e.getActionCommand())) {
			byte[] data = new byte[5];  
			int Angle=0;
			boolean DataValid = false;
			
			//Get Distance from dialog box
			do{
				String s = (String)JOptionPane.showInputDialog("Angle(-32767 ~ 32768) [degree] : ",JOptionPane.QUESTION_MESSAGE);
				
				if ((s != null) && (s.length() > 0)) {
					Angle=Integer.parseInt(s);
					if((Angle<=Max2Bytes)&&(Angle>=Min2Bytes)){ 		//check data validation
						DataValid=true;
					}else{
						JOptionPane.showMessageDialog(null,"Velocity is out of range","Message",JOptionPane.WARNING_MESSAGE);
					}
				}else{
					Angle=0;
					DataValid=true;
				}
			}while(!DataValid);
			
			if(Angle!=0){
				//Go angle
				if(Angle>0){
					CreatDirectDrive(DefaultVelocity,-DefaultVelocity);
				}else{
					CreatDirectDrive(-DefaultVelocity,DefaultVelocity);
				}
				//Wait angle
	            data[0] = (byte)157;  //Wait Angle
	            data[1] = (byte)((Angle >> 8) & 0x00FF);  //[Angle high byte] 
	            data[2] = (byte)(Angle & 0x00FF);    //[Angle low byte]
	            iRobotBAM.Write(data);
	            
	            //Stop
	            CreatDirectDrive(0,0);
			}
		}
		
		//drive the robot in a square 400mm on a side 
		if ("Make Square".equals(e.getActionCommand())) {
			byte[] data = new byte[5];  
			int Angle=82;
			int distance = 400;
			boolean DataValid = false;
			
			for (int i=0; i<4;i++){
				//Go 400mm
				//Go distance
				CreatDirectDrive(DefaultVelocity,DefaultVelocity);            
				//Wait distance
	            data[0] = (byte)156;  //Wait distance
	            data[1] = (byte)((distance >> 8) & 0x00FF);  //[Distance high byte] 
	            data[2] = (byte)(distance & 0x00FF);    //[Distance low byte]
	            iRobotBAM.Write(data);
	            //Stop
	            CreatDirectDrive(0,0);
				
	            //Turn Right 90 degree
				CreatDirectDrive(DefaultVelocity,-DefaultVelocity);
				//Wait angle
	            data[0] = (byte)157;  //Wait Angle
	            data[1] = (byte)((Angle >> 8) & 0x00FF);  //[Angle high byte] 
	            data[2] = (byte)(Angle & 0x00FF);    //[Angle low byte]
	            iRobotBAM.Write(data);
	            //Stop
	            CreatDirectDrive(0,0);
			}
		}
				
		//Read and Display the state of the wheel, bumper, and cliff sensors (binary) and read and display the value of the wall sensor (0-4095). 
		if("Read Sensors".equals(e.getActionCommand())){
						
			WheelDropCaster.setText(String.valueOf(((ReadSensor((byte)7))>>4) & 0x0001));	//WheelDropCaster : BumpsandWheelDrops[4]
			WheelDropLeft.setText(String.valueOf(((ReadSensor((byte)7))>>3) & 0x0001));	//WheelDropLeft: BumpsandWheelDrops[3]
			WheelDropRight.setText(String.valueOf(((ReadSensor((byte)7))>>2) & 0x0001));	//WheelDropRight: BumpsandWheelDrops[2]
			BumpLeft.setText(String.valueOf(((ReadSensor((byte)7))>>1) & 0x0001));	//BumpLeft: BumpsandWheelDrops[1]
			BumpRight.setText(String.valueOf((ReadSensor((byte)7)) & 0x0001));	//BumpRight: BumpsandWheelDrops[0]
			CliffLeft.setText(String.valueOf(ReadSensor((byte)9)));	//CliffLeft
			CliffRight.setText(String.valueOf(ReadSensor((byte)12)));	//CliffRight
			CliffFrontLeft.setText(String.valueOf(ReadSensor((byte)10)));	//CliffFrontLeft
			CliffFrontRight.setText(String.valueOf(ReadSensor((byte)11)));	//CliffFrontRight
			Wall.setText(String.valueOf(ReadSensor((byte)27)));	//Wall signal (0~4095(0xFFF))
			//System.out.println(ReadSensor((byte)8));	//Wall state
		}
		
		if("Wall Follow".equals(e.getActionCommand())){
			System.out.println("Wall Follow starts !!!");
			algorithmIndex=1;
			Wallfollowstop = true; 
			Thread itself = new Thread(this);
		    itself.start();
		}
		
		if("Bug2".equals(e.getActionCommand())){
			System.out.println("Bug2 starts !!!");
			algorithmIndex=2;
			Bug2stop = true; 
			Thread itself = new Thread(this);
		    itself.start();
		}
		
		if("Follow vGraph".equals(e.getActionCommand())){
			System.out.println("vGraph starts !!!");
			algorithmIndex=3;
			vGraphstop = true; 
			Thread itself = new Thread(this);
		    itself.start();
		}
	}
	
	public void run(){
		if(algorithmIndex==1){
			wallFollowing();
		}else if(algorithmIndex==2){
			Bug2Algorithm();
			//Bug2AlgorithmCurve();
		}else if(algorithmIndex==3){
			//followPath2();
			followPath();
		}
	}
	
	public void followPath()  { 
		int localTargetDistance=0;
		int localTargetAngle=0;
		
		Create.x=0;
		Create.y=0;
		for(int i=0;i<localtarget.size();i++){
			localTargetDistance=(int)Math.sqrt((Create.x-localtarget.get(i).x)*(Create.x-localtarget.get(i).x)+(Create.y-localtarget.get(i).y)*(Create.y-localtarget.get(i).y));
			localTargetAngle=(int)(Math.atan2(Create.y-localtarget.get(i).y,localtarget.get(i).x-Create.x)*180/PI)+(int)CreateOrientation*(-1);
			System.out.println("Robot Position = ["+Create.x+","+Create.y+"]");
			System.out.println("Local Target Position = ["+localtarget.get(i).x+","+localtarget.get(i).y+"]");
			System.out.println("Local Target Distacne : "+localTargetDistance);
			System.out.println("Local Target Orientation : "+localTargetAngle);
			
			//Turn to the local Target position
			if(localTargetAngle<0){
				CreatDirectDrive(-200,200);	//turn right
			}else if(localTargetAngle>0){
				CreatDirectDrive(200,-200);	//turn left
			}
			WaitAngle((int)(localTargetAngle*ANGLE_CAL_RATIO));		//0.69 radian
			CreatDirectDrive(0,0);
			CreateOrientation=CreateOrientation+localTargetAngle;
			
			//Go to the local Target position
			CreatDirectDrive(300,300);
			WaitDistance(localTargetDistance);
			CreatDirectDrive(0,0);
			Create.x=localtarget.get(i).x;
			Create.y=localtarget.get(i).y;
			if(!vGraphstop){
				break;
			}
		}
	}
	
	public void followPath2()  { 
		int localTargetDistance=0;
		int localTargetAngle=0;
		
		Create.x=0;
		Create.y=0;
		for(int i=0;i<localtarget.size();i++){
			localTargetDistance=(int)Math.sqrt((Create.x-localtarget.get(i).x)*(Create.x-localtarget.get(i).x)+(Create.y-localtarget.get(i).y)*(Create.y-localtarget.get(i).y));
			localTargetAngle=(int)(Math.atan2(Create.y-localtarget.get(i).y,localtarget.get(i).x-Create.x)*180/PI)+(int)CreateOrientation*(-1);
			System.out.println("Robot Position = ["+Create.x+","+Create.y+"]");
			System.out.println("Local Target Position = ["+localtarget.get(i).x+","+localtarget.get(i).y+"]");
			System.out.println("Local Target Distacne : "+localTargetDistance);
			System.out.println("Local Target Orientation : "+localTargetAngle);
			
			//Turn to the local Target position
			if(localTargetAngle<0){
				CreatDirectDrive(-200,200);	//turn right
			}else if(localTargetAngle>0){
				CreatDirectDrive(200,-200);	//turn left
			}
			WaitAngle((int)(localTargetAngle*ANGLE_CAL_RATIO));		//0.69 radian
			CreatDirectDrive(0,0);
			CreateOrientation=CreateOrientation+localTargetAngle;
			
			//Go to the local Target position
			//Acceleration
//			CreatDirectDrive(300,300);
//			WaitDistance(100);
//			CreatDirectDrive(0,0);
			CreatDirectDrive(400,400);
			WaitDistance(localTargetDistance-100);
			CreatDirectDrive(0,0);
			CreatDirectDrive(300,300);
			WaitDistance(100);
			CreatDirectDrive(0,0);
			Create.x=localtarget.get(i).x;
			Create.y=localtarget.get(i).y;
			if(!vGraphstop){
				break;
			}
		}
	}
	
	public void Bug2AlgorithmCurve(){
		int localDis=0;
		
		isTimerStarted=false;
		CreateOrientation=0;
		Create.x=0;
		Create.y=0;
		isHitObject=false;
		previousAngle=0;
		createLocal.x=createLocal.y=0;
		
		CreatDirectDrive(200,200);//Go straight
		previousDrive=1;
		previousVr=200;
		previousVl=200;
		//startTime=System.currentTimeMillis();
		while (Bug2stop){
			if(Create.x>GOAL_DISTANCE){		//Check out goal position.
				CreatDirectDrive(0,0);
				Bug2stop=false;
				break;
			}
			//if Object is detected and bumper is pressed, turn left
			if(isHitObject==false){
				if(((ReadSensor((byte)7)) & 0x0001)==1 ||((ReadSensor((byte)7))>>1 & 0x0001)==1 ){
					CreatDirectDrive(0,0);
					UpdateCreatePosition();
					
					CreatDirectDrive(200,-200);	//turn left
					previousDrive=2;
					previousAngle=30*PI/180;
					WaitAngle((int)(30*ANGLE_CAL_RATIO));		//0.69 radian
					CreatDirectDrive(0,0);
					UpdateCreatePosition();
					
					CreatDrive(200,-200);//Drive Curve
					previousDrive=3;
	
					isHitObject=true;
				}else{
					localDis=getDistance();
					//System.out.println("Distance="+localDis);
					Create.x+=(int)((long)localDis*Math.cos(CreateOrientation));
					Create.y+=(int)((long)localDis*Math.sin(CreateOrientation));
					//System.out.println("Robot local = ["+createLocal.x+","+createLocal.y+"]");
				}
			}else{	//Wall Following
				if(((ReadSensor((byte)7)) & 0x0001)==1 ||((ReadSensor((byte)7))>>1 & 0x0001)==1 ){
					if(Create.y<-10 && Create.x>0){		//check intersection m-line
						isHitObject=false;
						CreatDirectDrive(0,0);
						UpdateCreatePosition();
						isHitObject=true;
						
						CreatDirectDrive(200,-200);	//turn left
						WaitAngle((int)(((double)(-1)*(int)(CreateOrientation*180/PI)+90)*ANGLE_CAL_RATIO));
						previousDrive=2;
						previousAngle=(-1)*(float)CreateOrientation+PI/2;
						CreatDirectDrive(0,0);
						UpdateCreatePosition();
						
						CreatDirectDrive(200,200);//Go back to the m-line
						previousDrive=4;
						previousVr=(-1)*Create.y;
						WaitDistance((-1)*Create.y);		
						CreatDirectDrive(0,0);
						UpdateCreatePosition();
						
						CreatDirectDrive(-200,200);	//turn right
						WaitAngle(-85);
						previousDrive=2;
						previousAngle=-90*PI/180;
						CreatDirectDrive(0,0);
						UpdateCreatePosition();
						
						CreatDirectDrive(200,200);//Go straight
						previousDrive=1;
						previousVr=200;
						previousVl=200;
						isHitObject=false;	
					}else{
						CreatDirectDrive(0,0);
						UpdateCreatePosition();
						
						CreatDirectDrive(200,-200);	//turn left
						previousDrive=2;
						previousAngle=30*PI/180;
						WaitAngle((int)(30*ANGLE_CAL_RATIO));		//0.69 radian
						CreatDirectDrive(0,0);
						UpdateCreatePosition();
						
						CreatDrive(200,-200);//Drive Curve
						previousDrive=3;
					}
				}
			}
		}
	}
	
	//Bug2 Algorithm using Square pattern
	public void Bug2Algorithm(){
		int goCounter=0;
		int localDis=0;
		
		isTimerStarted=false;
		CreateOrientation=0;
		Create.x=0;
		Create.y=0;
		isHitObject=false;
		previousAngle=0;
		createLocal.x=createLocal.y=0;
		
		CreatDirectDrive(200,200);//Go straight
		previousDrive=1;
		previousVr=200;
		previousVl=200;
		//startTime=System.currentTimeMillis();
		while (Bug2stop){
			if(Create.x>GOAL_DISTANCE){		//Check out goal position.
				CreatDirectDrive(0,0);
				System.out.println("Arrive goal position successfully !!");
				Bug2stop=false;
				break;
			}
			//if Object is detected and bumper is pressed, turn left
			if(isHitObject==false){
				if(((ReadSensor((byte)7)) & 0x0001)==1 ||((ReadSensor((byte)7))>>1 & 0x0001)==1 ){
					CreatDirectDrive(0,0);
					UpdateCreatePosition();
					
					CreatDirectDrive(200,-200);	//turn left
					previousDrive=2;
					previousAngle=90*PI/180;
					WaitAngle(85);		//0.69 radian
					CreatDirectDrive(0,0);
					UpdateCreatePosition();
					
					//CreatDrive(200,-100);//Drive Curve
					//previousDrive=3;
					//previousVr=200;
					//startTime=System.currentTimeMillis();
					
					CreatDirectDrive(200,200);//Go straight
					previousDrive=4;
					previousVr=100;
					WaitDistance(100);		
					CreatDirectDrive(0,0);
					UpdateCreatePosition();
					
					CreatDirectDrive(-200,200);	//turn right
					previousDrive=2;
					previousAngle=-90*PI/180;
					WaitAngle(-85);		//0.69 radian
					CreatDirectDrive(0,0);
					UpdateCreatePosition();
					
					CreatDirectDrive(200,200);//Go straight
					previousDrive=1;
					previousVr=200;
					previousVl=200;
					//startTime=System.currentTimeMillis();
	
					isHitObject=true;
				}else{
					localDis=getDistance();
					//System.out.println("Distance="+localDis);
					Create.x+=(int)((long)localDis*Math.cos(CreateOrientation));
					Create.y+=(int)((long)localDis*Math.sin(CreateOrientation));
					//System.out.println("Robot local = ["+createLocal.x+","+createLocal.y+"]");
				}
			}else{	//Wall Following
				if(((ReadSensor((byte)7)) & 0x0001)==1 ||((ReadSensor((byte)7))>>1 & 0x0001)==1 ){
					if(Create.y<-10 && Create.x>0){		//check intersection m-line
						isHitObject=false;
						CreatDirectDrive(0,0);
						UpdateCreatePosition();
						isHitObject=true;
						
						CreatDirectDrive(200,-200);	//turn left
						WaitAngle((int)(((double)(-1)*(int)(CreateOrientation*180/PI)+90)*ANGLE_CAL_RATIO));
						previousDrive=2;
						previousAngle=(-1)*(float)CreateOrientation+PI/2;
						CreatDirectDrive(0,0);
						UpdateCreatePosition();
						
						CreatDirectDrive(200,200);//Go back to the m-line
						previousDrive=4;
						previousVr=(-1)*Create.y;
						WaitDistance((-1)*Create.y);		
						CreatDirectDrive(0,0);
						UpdateCreatePosition();
						
						CreatDirectDrive(-200,200);	//turn right
						WaitAngle(-85);
						previousDrive=2;
						previousAngle=-90*PI/180;
						CreatDirectDrive(0,0);
						UpdateCreatePosition();
						
						CreatDirectDrive(200,200);//Go straight
						previousDrive=1;
						previousVr=200;
						previousVl=200;
						isHitObject=false;	
					}else{
						//Advnaced_LED(OFF);
						CreatDirectDrive(0,0);
						UpdateCreatePosition();
						
						CreatDirectDrive(200,-200);	//turn left
						WaitAngle(85);
						previousDrive=2;
						previousAngle=90*PI/180;
						CreatDirectDrive(0,0);
						UpdateCreatePosition();
						
						//CreatDrive(200,-100);//Drive Curve
						//previousDrive=3;
						//previousVr=200;
						//startTime=System.currentTimeMillis();
						CreatDirectDrive(200,200);//Go straight
						previousDrive=4;
						previousVr=100;
						WaitDistance(100);		
						CreatDirectDrive(0,0);
						UpdateCreatePosition();
						
						CreatDirectDrive(-200,200);	//turn left
						previousDrive=2;
						previousAngle=-90*PI/180;
						WaitAngle(-85);		//0.69 radian
						CreatDirectDrive(0,0);
						UpdateCreatePosition();
						
						CreatDirectDrive(200,200);//Go straight
						previousDrive=1;
						previousVr=200;
						previousVl=200;
						//startTime=System.currentTimeMillis();
						goCounter=0;
					}
				}else{
					//Calculate the current position on the line.
					if(goCounter>1){
						if(Create.y<-10 && Create.x>0){
							isHitObject=false;
							CreatDirectDrive(0,0);
							UpdateCreatePosition();
							isHitObject=true;
							
							CreatDirectDrive(200,-200);	//turn left
							WaitAngle((int)(((double)(-1)*(int)(CreateOrientation*180/PI)+90)*ANGLE_CAL_RATIO));
							previousDrive=2;
							previousAngle=(-1)*(float)CreateOrientation+PI/2;
							CreatDirectDrive(0,0);
							UpdateCreatePosition();
							
							CreatDirectDrive(200,200);//Go back to the m-line
							previousDrive=4;
							previousVr=(-1)*Create.y;
							WaitDistance((-1)*Create.y);		
							CreatDirectDrive(0,0);
							UpdateCreatePosition();
							
							CreatDirectDrive(-200,200);	//turn right
							WaitAngle(-85);
							previousDrive=2;
							previousAngle=-90*PI/180;
							CreatDirectDrive(0,0);
							UpdateCreatePosition();
							
							CreatDirectDrive(200,200);//Go straight
							previousDrive=1;
							previousVr=200;
							previousVl=200;
							isHitObject=false;
							
						}else{
							isHitObject=false;
							CreatDirectDrive(0,0);
							UpdateCreatePosition();
							isHitObject=true;
							
							CreatDirectDrive(-200,200);	//turn right
							WaitAngle(-85);
							previousDrive=2;
							previousAngle=(-90)*PI/180;
							CreatDirectDrive(0,0);
							UpdateCreatePosition();
							
							CreatDirectDrive(200,200);//Go straight
							previousDrive=1;
							previousVr=200;
							previousVl=200;
							//startTime=System.currentTimeMillis();
							goCounter=0;
						}
					}else{
						goCounter++;
					}
				}
			}
		}
	}
	
	private void wallFollowing()
	{
		byte drivingState=0;		//0 : STOP, 1 : Forwarding 
		boolean isHitWall=false;
		int NoWallCounter=0;
		boolean isWall=false;
		int[] wallValue=new int[10];
		int wallFollowCounter=0;
		int wallAdjacence=0;
		
		CreatDirectDrive(200,200);//Go straight
		while (Wallfollowstop){
			//if wall is detected and bumper is pressed, turn left
			if(isHitWall==false){
				if(((ReadSensor((byte)7)) & 0x0001)==1 ||((ReadSensor((byte)7))>>1 & 0x0001)==1 ){
					CreatDirectDrive(200,-200);	//turn left
					WaitAngle(40);
					CreatDirectDrive(0,0);
					CreatDrive(200,-200);//Drive Curve
					isHitWall=true;
				}
			}else{
				if(((ReadSensor((byte)7)) & 0x0001)==1 ||((ReadSensor((byte)7))>>1 & 0x0001)==1 ){
					Advnaced_LED(OFF);
					CreatDirectDrive(200,-200);	//turn left
					WaitAngle(10);
					CreatDirectDrive(0,0);
					CreatDirectDrive(200,200);
					//CreatDrive(150,-600);//Drive Curve right
					//isWall=false;
					NoWallCounter=0;
					
				}else{
					if(ReadSensor((byte)8)==0){		//No Wall
						Advnaced_LED(OFF);
						CreatDirectDrive(-200,200);	//turn right
						if(NoWallCounter<10){
							WaitAngle(-5);
						}else{
							WaitAngle(-30);
							NoWallCounter=0;
						}
						CreatDirectDrive(200,200);
						/*if(NoWallCounter>=5){	
							CreatDirectDrive(0,200);	//turn right
							WaitAngle(-5);
							CreatDirectDrive(200,200);
							NoWallCounter=0;
						}*/
						isWall=false;
						NoWallCounter++;
					}else if((ReadSensor((byte)8)==1) || (ReadSensor((byte)27)>= 100)){		//Wall seen
						if(isWall==false){
							Advnaced_LED(ON);
							CreatDirectDrive(200,-200);	//turn left
							WaitAngle(5);
							CreatDirectDrive(200,200);
							NoWallCounter=0;
							isWall=true;
							wallFollowCounter=0;
							wallAdjacence=0;
						}else{
							wallValue[wallFollowCounter]=ReadSensor((byte)27);
							if(wallFollowCounter!=0){
								if(wallValue[wallFollowCounter-1]<wallValue[wallFollowCounter])
									wallAdjacence++;
								else if(wallValue[wallFollowCounter-1]>wallValue[wallFollowCounter])
									wallAdjacence--;
							}
							if(wallAdjacence>=4){
								CreatDirectDrive(200,-200);	//turn left
								WaitAngle(5);
								CreatDirectDrive(200,200);
								wallFollowCounter=0;
								wallAdjacence=0;
							}
							if(wallFollowCounter<10)
								wallFollowCounter++;
							else
								wallFollowCounter=0;
							//System.out.println(wallFollowCounter+","+wallAdjacence);
						}	
					}
				}
			}
		}
	}
	
	public int getDistance(){
		return ReadSensor((byte)19);
	}
	
	public int getAngle(){
		return ReadSensor((byte)20);
	}
	
	public void UpdateCreatePosition(){
		long elapsedTime=0;
		int localDistance=0;
		int localAngle=0;
		
		if(previousDrive==1){  /* Direct Drive */
			localDistance=getDistance();
			if(isHitObject==true)
				localDistance=localDistance-185;
			//System.out.println("Distance="+localDistance);
			//elapsedTime=(System.currentTimeMillis()-startTime);
			//System.out.println(elapsedTime);
			//CreateOrientation=CreateOrientation+((previousVr-previousVl)/DIAMETER)*elapsedTime/1000;
			//Create.x=Create.x+(int)((long)((double)(previousVr+previousVl)*Math.cos(CreateOrientation)/2)*elapsedTime/1000);
			//Create.y=Create.y+(int)((long)((double)(previousVr+previousVl)*Math.sin(CreateOrientation)/2)*elapsedTime/1000);
			Create.x=Create.x+(int)((long)localDistance*Math.cos(CreateOrientation));
			Create.y=Create.y+(int)((long)localDistance*Math.sin(CreateOrientation));
		}else if(previousDrive==2){	/* Curve Drive */
			CreateOrientation=CreateOrientation+previousAngle;
		}else if(previousDrive==3){/* Curve Drive */
			localDistance=getDistance();
			//if(isHitObject==true)
			//	localDistance=localDistance-185;
			//System.out.println("Distance="+localDistance);
			localAngle=getAngle();
			//System.out.println("Local Angle="+localAngle);
			int radius=200;
			int theta=localDistance/radius;
			CreateOrientation=CreateOrientation+localAngle*PI/180;
			Create.x=Create.x+(int)((double)(radius-(int)((long)radius*Math.cos(theta)))*Math.cos(CreateOrientation));
			Create.y=Create.y+(int)(((double)((long)radius*Math.sin(theta)))*Math.sin(CreateOrientation));
		}else if(previousDrive==4){
			Create.x=Create.x+(int)((long)((double)(previousVr)*Math.cos(CreateOrientation)));
			Create.y=Create.y+(int)((long)((double)(previousVr)*Math.sin(CreateOrientation)));
		}
		
		//Check intersect the m-line
		//if(Create.y==0)
		//System.out.println("Previoius Drive : " + previousDrive);
		//System.out.println("Robot= ("+Create.x+","+Create.y+","+(CreateOrientation*180/PI)+")");
	}
	
	private void WaitDistance(int d){
		//Wait distance
		byte[] data = new byte[3];
		
	    data[0] = (byte)156;  //Wait distance
	    data[1] = (byte)((d >> 8) & 0x00FF);  //[Distance high byte] 
	    data[2] = (byte)(d & 0x00FF);    //[Distance low byte]
	    iRobotBAM.Write(data);
	}
	private void WaitAngle(int Angle)
	{
		byte[] data = new byte[3];
		
		data[0] = (byte)157;  //Wait Angle
		data[1] = (byte)((Angle >> 8) & 0x00FF);  //[Angle high byte] 
		data[2] = (byte)(Angle & 0x00FF);    //[Angle low byte]
		iRobotBAM.Write(data);
	}
	
	private void CreatDrive(int Velocity, int Radius)
	{
		byte[] data = new byte[5]; 
		
		data[0] = (byte)137;  //Direct Drive command
		data[1] = (byte)((Velocity >> 8) & 0x00FF);  //[velocity high byte] 
        data[2] = (byte)(Velocity & 0x00FF);    //[velocity low byte]
        data[3] = (byte)((Radius >> 8) & 0x00FF); // [Radius high byte]
        data[4] = (byte)(Radius & 0x00FF);    //[Radius low byte]
        
        iRobotBAM.Write(data);
        
        //UpdateCreatPosition();
	}
	
	public void CreatDirectDrive(int rightVelocity, int leftVelocity)
	{
		byte[] data = new byte[5]; 

		data[0] = (byte)145;  //Direct Drive command
		data[1] = (byte)((rightVelocity >> 8) & 0x00FF);  //[Right velocity high byte] 
        data[2] = (byte)(rightVelocity & 0x00FF);    //[Right velocity low byte]
        data[3] = (byte)((leftVelocity >> 8) & 0x00FF);  //[Left velocity high byte]
        data[4] = (byte)(leftVelocity & 0x00FF);    //[Left velocity low byte]
        
        iRobotBAM.Write(data);
        //UpdateCreatePosition(rightVelocity,leftVelocity);
	}
	
	public int ReadSensor(byte packetID)
    {
        byte[] data = new byte[2];
        data[0] = (byte)142;  //Read Sensors
        data[1] = packetID;    //Packet ID
        iRobotBAM.Write(data);

        byte [] sensor =new byte[2];
        sensor[0]=0;
        sensor[1]=0;
        iRobotBAM.Read(sensor);
		if(packetID==27 || packetID==19){
			//System.out.println(sensor[0]);		//High byte
			//System.out.println(sensor[1]);		//Low byte
			int sensorValue=(sensor[0]&0xff);
			sensorValue=(sensorValue<<8)+(sensor[1]&0xff);	//
			//System.out.println(wallValue);
			return sensorValue;		//High byte first
		}else{
			//System.out.println(sensor[0]);
			return (int)sensor[0];
    	}
    }
	
	private void SetMode()
    {
		byte[] data = new byte[2];
		
		data[0] = (byte)128;  //Direct Drive command
        data[1] = (byte)131;  //Safe Mode
		
		iRobotBAM.Write(data);
    }

    private void LED_On()
    {
        byte[] data = new byte[4];
        int numBytes = 4;

        data[0] = (byte)139;  //LED command
        data[1] = (byte)8;    //Select LED (Play : 8, power : 2) 
        data[2] = (byte)0;    //Color 0 = green, 255 = red
        data[3] = (byte)128;  //Intensity

        iRobotBAM.Write(data);

        data[0] = (byte)139;  //LED command
        data[1] = (byte)2;    //Select LED (Play : 8, power : 2) 
        data[2] = (byte)255;    //Color 0 = green, 255 = red
        data[3] = (byte)128;  //Intensity

        iRobotBAM.Write(data);
    }
    
    private void Advnaced_LED(byte onoff)
    {
        byte[] data = new byte[4];

        data[0] = (byte)139;  //LED command
        data[1] = (byte)(onoff<<3);    //Select LED (Advanced : 3, Play : 1) 
        data[2] = (byte)255;    //Color 0 = green, 255 = red
        data[3] = (byte)128;  //Intensity

        iRobotBAM.Write(data);
    }

}
