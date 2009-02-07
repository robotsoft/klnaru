import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.*;
import java.awt.*;

/**
 * @author Joseph
 * @date 10/23/2008
 */
public class ControlPanel extends JPanel implements ActionListener {

	protected JButton Up,Down,Left,Right,Stop, Open,Mode;
	protected JPanel p1,p2;
	OpenComPort iRobotBAM = new OpenComPort();
	
    public ControlPanel() {
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
    	
        //Listen for actions on buttons 1 and 3.
    	Open.addActionListener(this);
    	Mode.addActionListener(this);
    	Up.addActionListener(this);
    	Down.addActionListener(this);
    	Left.addActionListener(this);
    	Right.addActionListener(this);
    	Stop.addActionListener(this);

       	p1=new JPanel(new BorderLayout());
    	p2=new JPanel(new BorderLayout());
        
    	//Add Components to this container, using the default FlowLayout.
        p1.add(Open,BorderLayout.WEST);
        p1.add(Mode,BorderLayout.EAST);
        p2.add(Up,BorderLayout.NORTH);
        p2.add(Down,BorderLayout.SOUTH);
        p2.add(Left,BorderLayout.WEST);
        p2.add(Right,BorderLayout.EAST);
        p2.add(Stop,BorderLayout.CENTER);
        
        setLayout(new BorderLayout());
        add(p1,BorderLayout.NORTH);
        add(p2,BorderLayout.SOUTH);
    }

    /**
     * Create the GUI and show it.  For thread safety, 
     * this method should be invoked from the 
     * event-dispatching thread.
     */
    public void createAndShowGUI() {

        //Create and set up the window.
        JFrame frame = new JFrame("ControlPanel");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        //Create and set up the content pane.
        ControlPanel newContentPane = new ControlPanel();
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
		}
		
		if ("Go".equals(e.getActionCommand())) {
			byte[] data = new byte[5];  

            data[0] = (byte)145;  //Direct Drive command
            data[1] = 0;    //[Right velocity high byte] 
            data[2] = (byte)200;  //[Right velocity low byte]
            data[3] = 0;    //[Left velocity high byte]
            data[4] = (byte)200;  //[Left velocity low byte]
            
            iRobotBAM.Write(data);
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

}
