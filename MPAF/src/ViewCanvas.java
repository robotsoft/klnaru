/**
 * @file 	ViewCanvas.java
 * @brief 	This file is canvas to draw af window. 
 *          It is dependent of mwaf.java
 * @author  joseph_hong
 * @version	1.0
 * @see     Development Environment : Eclipse Java Development Tools 2.1.3 & JDK1.4.2
 * @since	6 Jan. 2005
 */

import java.awt.*;

class ViewCanvas extends Canvas{
	private boolean isChoosedWinDraw=false, isMaxFocusPos=false, isSortedWindow=false;
//	private boolean isChoosedWindow[][];
	private Point gmax;
//	private int now;
	private int hgaps, vgaps;
//	private int MaxFocusPos[][];
	IntelligentAlgorithm ia = new IntelligentAlgorithm();
	
	public ViewCanvas(){
		setBackground(Color.white);
//		max=new Point(30,30);
//		min=new Point(-30,-30);
//		one=new Point(0,0);
//		two=new Point(0,0);
//		gone=new Point(0,0);
//		gtwo=new Point(0,0);
//		bSlope=false;
//		this.now=now;
	}
	
	public void paint(Graphics g){
		Rectangle r=getBounds();
		
		System.out.println(ia.GetNumberOfWindow());
		
		vgaps=r.height/ia.GetNumberOfWindow();
		hgaps=r.width/ia.GetNumberOfWindow();
				
		gmax=new Point(r.width,r.height);
				
		g.setColor(Color.gray);
		for(int i=0;i<=gmax.y;i+=vgaps)		//draw horizontal line
			g.drawLine(0,i,gmax.x,i);

		for(int i=0;i<=gmax.x;i+=hgaps)		//draw vertical line
			g.drawLine(i,0,i,gmax.y);	
		
		if(isChoosedWinDraw)
		{
			for(int i=0; i<ia.GetNumberOfWindow() ; i++){
				for (int j=0; j<ia.GetNumberOfWindow(); j++){
					if(ia.GetIsChoosedWindow(i,j)){
						g.setColor(Color.yellow);
						g.fillRect(i*hgaps,j*vgaps,hgaps,vgaps);
						if(isMaxFocusPos){
							g.setColor(Color.black);
							String s=new String();
							s=""+ia.GetMaxFocusPos(i,j);
							g.drawString(s,i*hgaps+hgaps/2,j*vgaps+vgaps/2);	
						}
					}
				}
				
			}
			//System.out.println(gmax+","+hgaps+","+vgaps+","+(choosedWin.x+1)*hgaps+","+(choosedWin.y+1)*vgaps);
		}
		
		if(isSortedWindow)
		{
			for(int i=0; i<ia.GetNumberOfWindow() ; i++){
				for (int j=0; j<ia.GetNumberOfWindow(); j++){
					System.out.println("WindowSet : "+ia.GetWindowSet(i,j));
					if(ia.GetWindowSet(i,j)>0){
						if(ia.GetWindowSet(i,j)==1)		//show maximum set
						{
							g.setColor(Color.lightGray);
							g.fillRect(i*hgaps,j*vgaps,hgaps,vgaps);
							
							g.setColor(Color.black);
							String s=new String();
							s=""+ia.GetMaxFocusPos(i,j);
							g.drawString(s,i*hgaps+hgaps/2,j*vgaps+vgaps/2);	
						}		
						if(isMaxFocusPos){
							g.setColor(Color.red);
							String s1=new String();
							s1=""+ia.GetWindowSet(i,j);
							g.drawString(s1,i*hgaps+2,j*vgaps+vgaps/2);
						}
					}
				}
	
			}
		}
	}
	
	/**
	 * @brief	Redraw windows
	 * @fn		void redrawWindow()
	 * @return 	None
	 *
	 * @author	joseph_hong
	 * @see	
	 */
	public void 
	redrawWindow(){
		
//		MaxFocusPos = new int[ia.GetNumberOfWindow()][ia.GetNumberOfWindow()];
		isMaxFocusPos = false;
//		ia.InitMaxFocusPos();
		ia.InitIsChoosedWindow();	
		repaint();
	}
	
	/**
	 * @brief	Find out array of window which is selected by mouse.
	 * @fn		Point SetWindowProperties(int mouseX, int mouseY)
	 * @param	mouseX, mouseY [in] the positon of mouse click
	 * @return 	win [out] the arrary of window
	 *
	 * @author	joseph_hong
	 * @see	
	 */
	public Point 
	SetWindowProperties(int mouseX, int mouseY){
		Point win = new Point(0,0);
		
		for(int i=0; i<ia.GetNumberOfWindow(); i++){
			if((i*hgaps<mouseX)&&(((i+1)*hgaps)>mouseX)){
				win.x=i;
				break;
			}
		}
		
		for(int j=0; j<ia.GetNumberOfWindow(); j++){
			if((j*vgaps<mouseY)&&(((j+1)*vgaps)>mouseY)){
				win.y=j;
				break;
			}
		}
		
		if(ia.GetIsChoosedWindow(win.x,win.y))
			ia.SetIsChoosedWindow(win.x,win.y,false);
		else
			ia.SetIsChoosedWindow(win.x,win.y,true);	
		System.out.println("Selected Window is ("+win.x+","+win.y+")");
		
		return win;
	}
	
	/**
	 * @brief	Redraw choosed windows
	 * @fn		void redrawChoosedWindow(Point win)
	 * @return 	None
	 *
	 * @author	joseph_hong
	 * @see	
	 */
	public void 
	redrawChoosedWindow(Point win)
	{
		isChoosedWinDraw=true;
		repaint();
	}
	
	/**
	 * @brief	Show max focus position at each window
	 * @fn		void showMaxFocusPos()
	 * @return 	None
	 *
	 * @author	joseph_hong
	 * @see	
	 */
	public void
	showMaxFocusPos()
	{
		isMaxFocusPos = true;
		repaint();
	}
	
	/**
	 * @brief	Redraw sorted windows
	 * @fn		void redrawSortedWindow()
	 * @return 	None
	 *
	 * @author	joseph_hong
	 * @see	
	 */
	public void
	redrawSortedWindow()
	{
		isSortedWindow = true;
		repaint();
	}
}