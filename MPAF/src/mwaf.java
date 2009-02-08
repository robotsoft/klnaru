/**
 * @file 	mwaf.java
 * @brief 	This file is main class(mwaf.class).
 *
 * @author  joseph_hong
 * @version	1.0
 * @see     Development Environment : Eclipse Java Development Tools 2.1.3 & JDK1.4.2
 *          Run : <APPLET CODE=mwaf.class WIDTH=640 HEIGHT=480></APPLET> in *.html
 * @since	5 Jan. 2005
 *          18 Aug. 2005 Modify layout
 *          19 Aug. 2005 Release ver.1.0
 */

import java.awt.*;
import java.awt.event.*;
import javax.swing.*;



public class mwaf extends JApplet{
	
	//Declare class member variables
	JComboBox cbNumberOfWindow,cbFVValid, cbDOF;
	ViewCanvas viewcanvas;
	JButton bSetMaxFocusPos, bRun;
	//AFWindow afw = new AFWindow();
	IntelligentAlgorithm ia = new IntelligentAlgorithm();
	
	/**
	 * @brief	initialize this class for Applet
	 * @fn		void init()
	 * @return 	None
	 *
	 * @author	joseph_hong
	 * @see	
	 */
	public void 
	init()
	{
		configFrame();
	}
  
	/**
	 * @brief	Config Main Frame
	 * @fn		void configFrame()
	 * @return 	None
	 *
	 * @author	joseph_hong
	 * @see	
	 */
	public void 
	configFrame()
	{
		String cbNumberOfWindowData[]=new String[5];
		String cbFVValidData[]={"True","Random"};
		String cbDOFData[]=new String[10];
		
		//Initialize Variables 
		for(int i=0;i<5;i++)
		{
			cbNumberOfWindowData[i]=""+(i*2+1);
		}
		for(int i=0; i<10; i++)
		{
			cbDOFData[i]=""+(i+1)*2;
		}
		ia.SetNumberOfWindow(1);
		
		//Set Layout
		//getContentPane().setLayout(new GridLayout(4,1));
		
		//Grid Layout (1,1)  
	    //Font f=new Font("TimesRoman",Font.BOLD,20);
		JLabel lTitle=new JLabel("Intelligent Algorithm for multi-points Auto Focus   v1.0",SwingConstants.CENTER);
		//lTitle.setFont(f);	
		
		//Grid Layout (2,1)
		viewcanvas = new ViewCanvas();
		//Add Listener
		viewcanvas.addMouseListener(new MouseAdapter(){
			public void mouseClicked(MouseEvent e){
				chooseWindows(e.getX(), e.getY());
			}
		});
		
	    //Grid Layout (3,1) 
		JPanel pParameters=new JPanel();
		JLabel lnow=new JLabel("Number of Windows :",SwingConstants.LEFT);
		//Make comboBox for number of window
		cbNumberOfWindow=new JComboBox(cbNumberOfWindowData);
		cbNumberOfWindow.setMaximumRowCount(4);
		cbNumberOfWindow.setSelectedIndex(ia.GetNumberOfWindow()-1);
		//Make Button for random maximum Focus position of choosed windows respectively
		bSetMaxFocusPos = new JButton("Set MFLP");
		bSetMaxFocusPos.setEnabled(false);
	  	//Add components to pParameter Panel
		pParameters.add(lnow);
		pParameters.add(cbNumberOfWindow);
		pParameters.add(bSetMaxFocusPos);
	  	//Add Listener
		cbNumberOfWindow.addItemListener(new ItemListener(){
			public void itemStateChanged(ItemEvent ie){
				if(ie.getStateChange()==ItemEvent.SELECTED){
					UpdateNumberOfWindow();
				}
			}
		 }); /*End of cbNumberOfWindow.addItemListener*/
		bSetMaxFocusPos.addActionListener(new ActionListener(){
			public void actionPerformed(ActionEvent ae){
				SetMaxFocusPos();
			}
		});/*End of bSetMaxFocusPos.addActionListener*/
	  
	  	//Gird Layout (4,1)
	  	JPanel pAlgorithm = new JPanel();
	  	
	  	JPanel pFVValid = new JPanel();
	  	JLabel lFVValid = new JLabel("FV Valid :");
		cbFVValid =new JComboBox(cbFVValidData);
		cbFVValid.setSelectedIndex(-1);
		cbFVValid.setEnabled(false);
		pFVValid.add(lFVValid);
	  	pFVValid.add(cbFVValid);
	  	
		JPanel pDOF = new JPanel();
		JLabel lDOF = new JLabel("DOF");
		cbDOF = new JComboBox(cbDOFData);
		cbDOF.setSelectedIndex(-1);
		cbDOF.setEnabled(false);
		pDOF.add(lDOF);
		pDOF.add(cbDOF);
		
	  	bRun = new JButton("RUN");
		bRun.setEnabled(false);
		
	  	//Add components to Panel
	  	pAlgorithm.add(pFVValid);
	  	pAlgorithm.add(pDOF);
	  	pAlgorithm.add(bRun);
	  	//Add Listener
		cbFVValid.addItemListener(new ItemListener(){
			public void itemStateChanged(ItemEvent ie){
				if(ie.getStateChange()==ItemEvent.SELECTED){
					cbDOF.setEnabled(true);
					//bRun.setEnabled(true);	
					ia.SetFVValid(cbFVValid.getSelectedIndex());
				}
			}
		 }); /*End of cbFVValid.addItemListener*/
		cbDOF.addItemListener(new ItemListener(){
			public void itemStateChanged(ItemEvent ie){
				if(ie.getStateChange()==ItemEvent.SELECTED){
					bRun.setEnabled(true);	
					ia.SetDOF(cbDOF.getSelectedIndex());
				}
			}
		 }); /*End of cbFVValid.addItemListener*/
	  	bRun.addActionListener(new ActionListener(){
			public void actionPerformed(ActionEvent ae){
				// call intelligent algorithm
				RunAlgorithm();
			}
	  	});/*End of bRun.addActionListener*/
	  	
	  	
	  	//Set entier layout
	  	GridBagLayout gridbag = new GridBagLayout();
        GridBagConstraints c = new GridBagConstraints();
        getContentPane().setLayout(gridbag);
                
        //c.weightx = 0.0;
        c.gridx = 0;
        c.gridy = 0;
        //c.gridwidth = GridBagConstraints.BOTH;
        //c.gridheight = 10;
        c.weighty = 10;		// % weight vertically
        gridbag.setConstraints(lTitle, c);
        
        //c.weightx = 0.0;
        c.gridx = 0;
        c.gridy = 1;
        //c.gridwidth = GridBagConstraints.BOTH;
        //c.gridheight = 300;
        c.weighty = 70;		// % weight vertically
        c.fill = GridBagConstraints.BOTH;		// It should be set in order to show the canvas
        gridbag.setConstraints(viewcanvas,c);
        
        //c.weightx = 0.0;
        c.gridx = 0;
        c.gridy = 2;
        //c.gridwidth = GridBagConstraints.BOTH;
        //c.gridheight = 10;
        c.weighty = 10;		// % weight vertically
        gridbag.setConstraints(pParameters,c);
        
        //c.weightx = 0.0;
        c.gridx = 0;
        c.gridy = 3;
        //c.gridwidth = GridBagConstraints.BOTH;
        //c.gridheight = GridBagConstraints.BOTH;
        c.weighty = 10;		// % weight vertically
        gridbag.setConstraints(pAlgorithm,c);
        
        //setSize(640, 640);
        
	  	//Add components to mainfram
		getContentPane().add(lTitle);
		getContentPane().add(viewcanvas);
		getContentPane().add(pParameters);
		getContentPane().add(pAlgorithm);
		
		

	    
  	}/*configFrame()*/

	/**
	 * @brief	Choose AF window by mouse
	 * @fn		chooseWindows()
	 * @return 	None
	 *
	 * @author	joseph_hong
	 * @see	
	 */
	public void
	chooseWindows(int mouseX, int mouseY)
	{
		viewcanvas.redrawChoosedWindow(viewcanvas.SetWindowProperties(mouseX, mouseY));
	}
	
	/**
	 * @brief	Update number of window
	 * @fn		void UpdateNumberOfWindow()
	 * @return 	None
	 *
	 * @author	joseph_hong
	 * @see	
	 */
  	public void 
	UpdateNumberOfWindow()
  	{
		//numberOfWindow=cbNumberOfWindow.getSelectedIndex()*2+1;
		ia.SetNumberOfWindow(cbNumberOfWindow.getSelectedIndex()*2+1);
		viewcanvas.redrawWindow();
		bSetMaxFocusPos.setEnabled(true);
	}
    
	/**
	 * @brief	Set Maximum Focus Position using random function
	 * @fn		void SetMaxFocusPos()
	 * @return 	None
	 *
	 * @author	joseph_hong
	 * @see	
	 */
    public void
	SetMaxFocusPos()
	{
		ia.InitMaxFocusPos();
		viewcanvas.showMaxFocusPos();
		cbFVValid.setEnabled(true);
	}
	
    /**
	 * @brief	Run algorithm and redraw sorted window
	 * @fn		void RunAlgorithm()
	 * @return 	None
	 *
	 * @author	joseph_hong
	 * @see	
	 */
	public void
	RunAlgorithm()
	{
		ia.RunAlgorithm();
		viewcanvas.redrawSortedWindow();
	}
	
	/**
	 * @brief	This is main function in entier classes
	 * @fn		void main(String args[])
	 * @param	String args[]
	 * @return 	None
	 *
	 * @author	joseph_hong
	 * @see	
	 */
	public static void main(String args[]) {
		JFrame f = new JFrame("Intelligent Algorithm for multi-windows Auto Focus");
		mwaf mainframe = new mwaf();
		mainframe.init();
			
		f.getContentPane().add("Center", mainframe);
		f.setSize(640, 480);
		f.setVisible(true);			// display mainframe
	 }

}