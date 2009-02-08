/**
 * @file 	IntelligentAlgorithm.java
 * @brief 	This file contain Intelligent Algorithm for multi window Auto Focus.
 *			It is dependent of mwaf.java
 * @author  joseph_hong
 * @version	1.0
 * @see     Development Environment : Eclipse Java Development Tools 2.1.3 & JDK1.4.2
 * @since	6 Jan. 2005
 *          18 Aug. 2005 Debug sorting the sets
 *          19 Aug. 2005 Add grouping w/ considering neighbor
 */

import java.util.Random;		/*for Random()*/
import java.awt.*;				/* for Point class */


class IntelligentAlgorithm{
	final static int MAX_FOCUS_POS=500;
	private static int numberOfWindow;
	private static boolean isChoosedWindow[][];
	private static int MaxFocusPos[][];
	private static boolean FVValid[][];
	private static int depthOfFocus;
	private static int windowSet[][];
	
	/**
	 * @brief	Set Number of Window
	 * @fn		SetNumberOfWindow(int now)
	 * @param	now	[in] number of window
	 * @return 	None
	 *
	 * @author	joseph_hong
	 * @see	
	 */
	public void
	SetNumberOfWindow(int now)
	{
		numberOfWindow=now;
//		System.out.println("Number Of Window is "+numberOfWindow);
	}
	
	/**
	 * @brief	Get Number of Window
	 * @fn		SetNumberOfWindow(int now)
	 * @return 	numberOfWindow	[out] number Of Window
	 *
	 * @author	joseph_hong
	 * @see	
	 */
	public int
	GetNumberOfWindow()
	{
		return numberOfWindow;
	}

	/**
	 * @brief	Initialize Maximum Focus Position
	 * @fn		void InitMaxFocusPos()
	 * @return 	None
	 *
	 * @author	joseph_hong
	 * @see	
	 */
	public void
	InitMaxFocusPos(){
		MaxFocusPos = new int[numberOfWindow][numberOfWindow];
		FVValid = new boolean[numberOfWindow][numberOfWindow];
		windowSet = new int[numberOfWindow][numberOfWindow];
		
		Random r=new Random();
		
		for(int i=0; i<numberOfWindow; i++)
			for(int j=0; j<numberOfWindow; j++)
				if(isChoosedWindow[i][j]){
					MaxFocusPos[i][j]=(r.nextInt(MAX_FOCUS_POS));
					System.out.println(MaxFocusPos[i][j]);
				}
	}
	
	/**
	 * @brief	Set IsChoosedWindow
	 * @fn		void SetIsChoosedWindow(int i, int j, boolean tf)
	 * @param	i	[in] arrary
	 * @param	j	[in] arrary
	 * @param	tf	[in] true or false
	 * @return 	None
	 *
	 * @author	joseph_hong
	 * @see	
	 */
	public void
	SetIsChoosedWindow(int i, int j, boolean tf)
	{
		isChoosedWindow[i][j]=tf;
	}
	
	/**
	 * @brief	Get IsChoosedWindow
	 * @fn		boolean GetIsChoosedWindow(int i, int j)
	 * @param	i	[in]	array
	 * @param	j	[in]	array
	 * @return 	isChoosedWindow[i][j]	[out]	true or false
	 *
	 * @author	joseph_hong
	 * @see	
	 */
	public boolean
	GetIsChoosedWindow(int i, int j)
	{
		return isChoosedWindow[i][j];
	}
	
	/**
	 * @brief	Initialize IsChoosedWindow
	 * @fn		void InitIsChoosedWindow()
	 * @return 	None
	 *
	 * @author	joseph_hong
	 * @see	
	 */
	public void
	InitIsChoosedWindow()
	{
		isChoosedWindow = new boolean[numberOfWindow][numberOfWindow];
		for(int i=0;i<numberOfWindow;i++)
			for(int j=0;j<numberOfWindow;j++)
				isChoosedWindow[i][j]=false;	/*Initialize*/
	}
	
	/**
	 * @brief	Get MaxFocusPos
	 * @fn		int GetMaxFocusPos(int i, int j)
	 * @param	i	[in]	array
	 * @param	j	[in]	array
	 * @return 	isChoosedWindow[i][j]	[out]	true or false
	 *
	 * @author	joseph_hong
	 * @see	
	 */
	public int
	GetMaxFocusPos(int i, int j)
	{
		return MaxFocusPos[i][j];
	}
	
	/**
	 * @brief	Set MaxFocusPos
	 * @fn		void SetFVValid(int validOption)
	 * @param	validOption	[in]	0 : all true, 1 : random true or false
	 * @return 	None
	 *
	 * @author	joseph_hong
	 * @see	
	 */
	public void
	SetFVValid(int validOption){
		if(validOption==0){
			for(int i=0; i<numberOfWindow; i++)
				for(int j=0; j<numberOfWindow; j++){
					FVValid[i][j]=true;
					System.out.println(FVValid[i][j]);
				}
					
		}else if(validOption==1){
			Random r=new Random();
			
			for(int i=0; i<numberOfWindow; i++)
				for(int j=0; j<numberOfWindow; j++){
					if(r.nextInt(2)==1)
							FVValid[i][j]=true;
					else
							FVValid[i][j]=false;
					System.out.println(FVValid[i][j]);
				}					
		}else{
			/*Exception Handler*/
		}
		
	}
	
	/**
	 * @brief	Set DOF
	 * @fn		void SetDOF(int dof)
	 * @param	dof	[in]	depth of focus
	 * @return 	None
	 *
	 * @author	joseph_hong
	 * @see	
	 */
	public void
	SetDOF(int dof)
	{
		depthOfFocus = (dof+1)*2;
		System.out.println("DOF : "+depthOfFocus);
	}
	
	/**
	 * @brief	Run intelligent Algorithm
	 * @fn		void RunAlgorithm()
	 * @return 	None
	 *
	 * @author	joseph_hong
	 * @see	
	 */
	public void 
	RunAlgorithm()
	{
		MakeSet();
		CalculateIOS();
		System.out.println("Algorithm done.");
	}
	
	/**
	 * @brief	Make Set according to maximum focus position of each window
	 * @fn		void MakeSet()
	 * @return 	None
	 *
	 * @author	joseph_hong
	 * @see	    
	 */
	public void
	MakeSet()
	{
//		int IndexOfMaxFocusPos[][] = new int[numberOfWindow][numberOfWindow];
		int tempMaxFocusPos=0;
		int constant=1;
//		int tempMaxSetNumber=1;
		int tempWindowSet[] = new int[numberOfWindow*numberOfWindow];
		int tempWindow = 0;
		//Point tempSet[]=new Point[numberOfWindow*numberOfWindow];
		//Point temp =new Point();
		int tempSetx[]=new int[numberOfWindow*numberOfWindow];
		int tempSety[]=new int[numberOfWindow*numberOfWindow];
		int tempx=0; 
		int tempy=0; 
		int n=0;
		int order=1;
		int tempDistance=0;
		int DistanceLimit=15;	// limit distance for setting group
		
		// Find Maximum Focus Position
		for(int i=0; i<numberOfWindow; i++)
			for(int j=0; j<numberOfWindow; j++)
			{
				if((isChoosedWindow[i][j]==true)&&(FVValid[i][j]==true))
				{
					if(MaxFocusPos[i][j]>tempMaxFocusPos)
						tempMaxFocusPos=MaxFocusPos[i][j];
				}
			}
		System.out.println("MaxPos : "+tempMaxFocusPos);
		
		//Make set SETm = {AFWn, ет1 < FPn,max< ет2}
		for(int i=0; i<numberOfWindow; i++)
			for(int j=0; j<numberOfWindow; j++)
			{
				if((isChoosedWindow[i][j]==true)&&(FVValid[i][j]==true))
				{
					tempWindowSet[n]= (tempMaxFocusPos-MaxFocusPos[i][j])/(constant*depthOfFocus)+1;
					tempSetx[n]=i;
					tempSety[n]=j;
					System.out.println("1st Sorted Set : "+ MaxFocusPos[i][j]+":"+tempWindowSet[n]);
					n++;
				}
			}
		
		//Sort the sets
		for(int p=0; p<n; p++)
			for(int q=0; q<(n-1); q++)
			{
				if((tempWindowSet[q]>tempWindowSet[q+1])||(MaxFocusPos[tempSetx[q]][tempSety[q]]<MaxFocusPos[tempSetx[q+1]][tempSety[q+1]]))
				{
					tempWindow=tempWindowSet[q];
					tempWindowSet[q]=tempWindowSet[q+1];
					tempWindowSet[q+1]=tempWindow;
					
					tempx=tempSetx[q];
					tempSetx[q]=tempSetx[q+1];
					tempSetx[q+1]=tempx;
					
					tempy=tempSety[q];
					tempSety[q]=tempSety[q+1];
					tempSety[q+1]=tempy;
					
				}
				System.out.println("2nd Sorted Set : "+ MaxFocusPos[tempSetx[q]][tempSety[q]]+":"+tempWindowSet[q]);
			}
		//Save temp variable into global variable
		for(int p=0; p<n; p++)
		{	
			if((p!=0)&&(tempWindowSet[p]!=tempWindowSet[p-1]))
				order++;
			windowSet[tempSetx[p]][tempSety[p]]=order;
			System.out.println("3rd Sorted Set : "+ MaxFocusPos[tempSetx[p]][tempSety[p]]+":"+windowSet[tempSetx[p]][tempSety[p]]);
		}
		
		//Check adjacent point of same set
		for(int p=0; p<n; p++)
		{	
			int z=p;
			if(windowSet[tempSetx[p]][tempSety[p]]==windowSet[tempSetx[p+1]][tempSety[p+1]])
			{
				tempDistance = (int)((double)10*(Math.sqrt(Math.pow(Math.abs(tempSetx[p]-tempSetx[p+1]),2)+Math.pow(Math.abs(tempSety[p]-tempSety[p+1]),2))));
				if(tempDistance>DistanceLimit)
				{
					do
					{
						windowSet[tempSetx[z+1]][tempSety[z+1]]++;
						z++;
					}while(z<(n-1));
				}
				//System.out.println("4th Sorted Set : "+ MaxFocusPos[tempSetx[p]][tempSety[p]]+":"+windowSet[tempSetx[p]][tempSety[p]]);
			}
		}
		
		for(int p=0; p<n; p++)
		{	
			System.out.println("4th Sorted Set : "+ MaxFocusPos[tempSetx[p]][tempSety[p]]+":"+windowSet[tempSetx[p]][tempSety[p]]);
		}
	}
	
	/**
	 * @brief	Get windowSet after making SETs
	 * @fn		GetWindowSet(int i, int j)
	 * @param	i	[in]	window array
	 * @param	j	[in]	window array
	 * @return 	windowSet	[out]
	 *
	 * @author	joseph_hong
	 * @see	    
	 */	
	public int
	GetWindowSet(int i, int j)
	{
		return windowSet[i][j];
	}
	
	
	/**
	 * @brief	Calculate IOS (Index Of Subject) among windowSet
	 * @fn		void CalculateIOS()
	 * @return 	None
	 *
	 * @author	joseph_hong
	 * @see	    
	 */
	public void
	CalculateIOS()
	{
		int tempWindowSet=0;
		int tempMaxFocusPos=0;
		int IOS[] = new int [numberOfWindow*numberOfWindow];
		boolean endflag=false;
		int doneWindowSet[] = new int[numberOfWindow*numberOfWindow];
		int nows=0;
		
		do{
			for(int i=0; i<numberOfWindow; i++)
				for(int j=0; j<numberOfWindow; j++)
				{
					if((isChoosedWindow[i][j]==true)&&(FVValid[i][j]==true))
					{
						if(windowSet[i][j]>0){
							if((tempWindowSet==windowSet[i][j])||(tempWindowSet==0)){
								tempWindowSet=windowSet[i][j];
								if(MaxFocusPos[i][j]>tempMaxFocusPos)
									tempMaxFocusPos=MaxFocusPos[i][j];
							}
						}
							
					}
				}
			IOS[tempWindowSet]=tempWindowSet*CalculateAOC(tempWindowSet);
			System.out.println("IOS["+IOS[tempWindowSet]+"] = "+tempWindowSet+"x"+CalculateAOC(tempWindowSet));
			
			//debug here !!!!!
			doneWindowSet[nows]=tempWindowSet;
			nows++;
			tempWindowSet=0;
		}while(endflag);
	}
	
	/**
	 * @brief	Calculate distance of centers
	 * @fn		void CalculateAOC(int ws)
	 * @param   ws [in]	number of windowset calculated
	 * @return 	None
	 *
	 * @author	joseph_hong
	 * @see	    
	 */
	public int 
	CalculateAOC(int ws)
	{
		Point centerOfWindow = new Point();
		Point topLeft = new Point(numberOfWindow,numberOfWindow);
		Point bottomRight = new Point(0,0);
		Point shortedWindow = new Point();
		int tempMaxFocusPos = 0;
		int aoc = 0;
		
		//Find center of selected window
		for(int i=0; i<numberOfWindow; i++)
			for(int j=0; j<numberOfWindow; j++)
			{
				if((isChoosedWindow[i][j]==true)&&(FVValid[i][j]==true))
				{
					if(topLeft.x<i)
						topLeft.x=i;
					if(topLeft.y<j)
						topLeft.y=j;
					if(bottomRight.x>i)
						bottomRight.x=i;
					if(bottomRight.y>j)
						bottomRight.y=j;				
				}
			}
		centerOfWindow.x=Math.abs(bottomRight.x-topLeft.x)/2;
		centerOfWindow.y=Math.abs(bottomRight.y-topLeft.y)/2;
		
		//Find shortest window within specified window set (sw)
		for(int i=0; i<numberOfWindow; i++)
			for(int j=0; j<numberOfWindow; j++)
			{
				if((isChoosedWindow[i][j]==true)&&(FVValid[i][j]==true))
				{
					if(windowSet[i][j]==ws)
						if(MaxFocusPos[i][j]>tempMaxFocusPos){
							shortedWindow.x=i;
							shortedWindow.y=j;
							tempMaxFocusPos=MaxFocusPos[i][j];	
						}		
				}
			}
		
		//Calculate AOC	
		aoc=(int)((double)10*(Math.sqrt(Math.pow(Math.abs(shortedWindow.x-centerOfWindow.x),2)+Math.pow(Math.abs(shortedWindow.y-centerOfWindow.y),2))));
		//System.out.println(Math.pow(Math.abs(shortedWindow.x-centerOfWindow.x),2));
		System.out.println("shortedWindow ="+shortedWindow+","+"centeredOfWindow = "+centerOfWindow);
		return aoc;
	}
}