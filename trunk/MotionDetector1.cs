// Motion Detector
//
// Copyright © Andrew Kirillov, 2005
// andrew.kirillov@gmail.com
//


//iRobot Todo List
//1. Mouse detection : done @ 11/17/08 by joseph
//2. Get color : done @ 11/19/08 by joseph
//3. Robust to light : Converting RGB to HSV : done @ 12/3/08 by joseph
//4. Add manual control panel : done @ 12/9/08 by joseph
//5. Dynamic Search Area : done @ 12/10/08 by joseph ---> 12/12 : update when target object disappear
//6. Limit for Adjacency : done @ 12/10/08 by joseph
//7. Robustness for noise : 

namespace motion
{
    using System;
    using System.Drawing;

    using AForge.Imaging;
    using System.Collections;
    using System.Runtime.InteropServices;
    using System.Reflection;
    using System.Windows.Forms;
    using System.Threading;

    public struct ColorImage{
            public byte red;
            public byte green;
            public byte blue;
     }
    public struct HSVColorSpace
    {
        public double h;        /* Hue degree between 0.0 and 360.0 */
        public double s;        /* Saturation between 0.0 (gray) and 1.0 */
        public double v;        /* Value between 0.0 (black) and 1.0 */
    }
    /// <summary>
    /// MotionDetector1
    /// </summary>
    public class MotionDetector1 : IMotionDetector
    {

        private MainForm _mForm;
        private int step = 1;
        private bool wait = false;
        private int frameCount = 0;
        public CreateDeviceDriver f1;
        //add for iRobot
        int TurnLeftCounter = 0;
        int TurnRightCounter = 0;
        int GoCounter = 0;
        int StopCounter = 0;
        int BumperStopCounter = 0;
        int LostCounter = 0;
        int Stablecount = 2;        // 30ms * 6 = 180ms delay for control
        int stateCreateMobility = 0;    // 0: Stop, 1: Go, 2 :Turn Left, 3 : Trun Right
        int centralRange = 30;
        private const int STOP = 0;
        private const int GO = 1;
        private const int LEFT = 2;
        private const int RIGHT = 3;
        private const int BACK = 4;
        ColorImage realImage = new ColorImage();
        ColorImage targetImage = new ColorImage();
        public bool targetFlag = false;
        private double ColorTolerance = 8;
        private double SaturationTolerance=0.03;
        private HSVColorSpace realImageHSV = new HSVColorSpace();
        private HSVColorSpace targetImageHSV = new HSVColorSpace();
        private Point targetPoint = new Point();
        private const int FOV = 61;         // Field of View of the camera
        private const int IMAGE_WIDTH = 320;
        private int targetAngle = 0;
        private Point targetSum = new Point();
        private int targetPixel = 0;
        private Point targetCentroid = new Point();
        private Point maxTargetPoint = new Point();
        private Point minTargetPoint = new Point();
        private Point SearchStartPoint = new Point();
        private Point SearchEndPoint = new Point();
        private const int SEARCH_MARGIN = 20;
        private const int MIN_TARGET_PIXEL = 2;

        public MainForm mForm
        {
            get { return _mForm; }
            set { _mForm = value; }
        }

        // Constructor
        public MotionDetector1()
        {
            //Add for iRobot
            f1 = new CreateDeviceDriver();
            f1.OpenPort_Click();
            Thread.Sleep(2000);      //2000ms
            f1.SafeMode_Click();
        }

        // Reset detector to initial state
        public void Reset()
        {
        }        


        private class PortAccess
        {
            [DllImport("inpout32.dll", EntryPoint = "Out32")]
            public static extern void Output(int adress, int value);
        }

        //added for iRobot @ 11/17/08 by joseph
        //Get Target Color
        public void getTargetColor(ref Bitmap image)
        {
            //Point mousePoint = new Point();
   
            //Read MouseClick
            if(mForm.MouseClickOnImage==true){
                targetPoint.X = mForm.MouseLocationOnImage.X;
                targetPoint.Y = mForm.MouseLocationOnImage.Y;
                Console.WriteLine("Mouse(x,y) = (" + targetPoint.X + "," + targetPoint.Y + ")");

                //Read target color
                Utility.UnsafeBitmap uBitmap = new Utility.UnsafeBitmap(image);
                uBitmap.LockBitmap();

                targetImage.red = uBitmap.GetPixel(targetPoint.X, targetPoint.Y).red;
                targetImage.green = uBitmap.GetPixel(targetPoint.X, targetPoint.Y).green;
                targetImage.blue = uBitmap.GetPixel(targetPoint.X, targetPoint.Y).blue;
                targetImageHSV = ConvertRGB2HSV(targetImage.red, targetImage.green, targetImage.blue);
                Console.WriteLine("Traget(r,g,b) = (" + targetImage.red + "," + targetImage.green+","+targetImage.blue+ ")");
                Console.WriteLine("Target(H,S,V) = (" + targetImageHSV.h + "," + targetImageHSV.s + "," + targetImageHSV.v + ")");
                targetFlag = true;
                mForm.MouseClickOnImage = false;
            }
        }

        // Process new frame
        public void ProcessFrame(ref Bitmap image)
        {
            
            //Manual Control
            if (mForm.ColorTrackingOnFlag == false)     
            {
                switch (mForm.ManualControlFlag)
                {
                    case GO: if (stateCreateMobility != GO)
                                {
                                    f1.Go_Click();
                                    ResetCounters();
                                    stateCreateMobility = GO;
                                }
                                break;
                    case STOP: if (stateCreateMobility != STOP) 
                                {
                                    f1.Stop_Click();
                                    ResetCounters();
                                    stateCreateMobility = STOP;
                                }; 
                                break;
                    case RIGHT: if (stateCreateMobility != RIGHT)
                                {
                                    f1.TurnRight_Click(50);
                                    ResetCounters();
                                    stateCreateMobility = RIGHT;
                                }
                                break;
                    case LEFT : if (stateCreateMobility != LEFT)
                                {
                                    f1.TurnLeft_Click(50);
                                    ResetCounters();
                                    stateCreateMobility = LEFT;
                                } break;
                    case BACK : if (stateCreateMobility != BACK)
                                {
                                    f1.Back_Click();
                                    ResetCounters();
                                    stateCreateMobility = BACK;
                                } break;
                    default: break;
                }
                targetFlag = false;
                return; 
            }

            Stablecount = mForm.StableCounter;
            ColorTolerance = mForm.ColorTolerance;

            if ((targetFlag == false) || (mForm.MouseClickOnImage == true))
            {
                getTargetColor(ref image);
                //TurnToTarget();
                SearchStartPoint.X = 0;
                SearchStartPoint.Y = 0;
                SearchEndPoint.X = image.Width;
                SearchEndPoint.Y = image.Height;
                return;
            }
            

            //Find out target Color
            Utility.UnsafeBitmap uBitmap = new Utility.UnsafeBitmap(image);
            bool tragetColorFound = false;            
            int xPos = 0, yPos = 0;
            targetSum.X = 0;
            targetSum.Y = 0;
            targetPixel = 0;
            targetCentroid.X=targetCentroid.Y=0;
            minTargetPoint.X = image.Width;
            minTargetPoint.Y = image.Height;
            maxTargetPoint.X = maxTargetPoint.Y = 0;

            uBitmap.LockBitmap();
            //Console.WriteLine("Search Window :("+SearchStartPoint.X+","+SearchStartPoint.Y+"),("+SearchEndPoint.X+","+SearchEndPoint.Y+")");
            for (int y = SearchStartPoint.Y; y < SearchEndPoint.Y; y += 2)
            {
                for (int x = SearchStartPoint.X ; x < SearchEndPoint.X; x += 2)
                {
                    //byte red, green, blue;
                    realImage.red = uBitmap.GetPixel(x, y).red;
                    realImage.green = uBitmap.GetPixel(x, y).green;
                    realImage.blue = uBitmap.GetPixel(x, y).blue;

                    //float brightness = (299 * red + 587 * green + 114 * blue) / 1000;
                    //float brightness = realImage.red;
                    
                    //Change the Color Space from RGB to HSV
                    realImageHSV=ConvertRGB2HSV(realImage.red, realImage.green, realImage.blue);
                    //Console.WriteLine("Real Image (R,G,B) = ("+realImage.red+","+realImage.green+","+realImage.blue+")");

                    //Use sin(hue) to compensate the period of hue is pi.
                    if (((realImageHSV.h >= (targetImageHSV.h - ColorTolerance)) && (realImageHSV.h <= (targetImageHSV.h + ColorTolerance)))
                        && ((realImageHSV.s >= (targetImageHSV.s - SaturationTolerance)) && (realImageHSV.s <= (targetImageHSV.s + SaturationTolerance)))
                        )
                    {
                    //if ((realImageHSV.h >= (targetImageHSV.h - ColorTolerance)) && (realImageHSV.h <= (targetImageHSV.h + ColorTolerance)))
                    //{
                        /*if (((realImage.red >= (targetImage.red-ColorTolerance))&&(realImage.red <= (targetImage.red+ColorTolerance))) 
                            && ((realImage.green >= (targetImage.green-ColorTolerance))&&(realImage.green <= (targetImage.green+ColorTolerance)))
                            && ((realImage.blue >= (targetImage.blue-ColorTolerance)) && (realImage.blue <= (targetImage.blue+ColorTolerance))))
                        {*/
                        //brightest = brightness;
                        xPos = x;
                        yPos = y;
                        targetSum.X += xPos;
                        targetSum.Y += yPos;
                        targetPixel ++;
                        if (xPos > maxTargetPoint.X) maxTargetPoint.X = xPos;
                        if (yPos > maxTargetPoint.Y) maxTargetPoint.Y = yPos;
                        if (xPos < minTargetPoint.X) minTargetPoint.X = xPos;
                        if (yPos < minTargetPoint.Y) minTargetPoint.Y = yPos;
                        //tragetColorFound = true;
                        //realImageHSV = ConvertRGB2HSV(realImage.red, realImage.green, realImage.blue);
                        //Console.WriteLine("Position : ("+xPos+","+yPos+")");
                        //Console.WriteLine("Real(r,g,b) = (" + realImage.red + "," + realImage.green + "," + realImage.blue + ")");
                        //Console.WriteLine("Real(H,S,V) = (" + realImageHSV.h + "," + realImageHSV.s + "," + realImageHSV.v + ")");
                    }
                } // x loop
            } // y loop

            //Calculate the centroid of target object
            if (targetPixel > MIN_TARGET_PIXEL)
            {
                targetCentroid.X = targetSum.X / targetPixel;
                targetCentroid.Y = targetSum.Y / targetPixel;
                tragetColorFound = true;
                //Console.WriteLine("target Centroid : ("+targetCentroid.X+","+targetCentroid.Y+")");
                //Console.WriteLine("target pixel : " + targetPixel);

                //Change Search Range for the dynamic search
                if (stateCreateMobility==GO)
                {
                    if (minTargetPoint.X > SEARCH_MARGIN)
                    {
                        SearchStartPoint.X = minTargetPoint.X - SEARCH_MARGIN;
                    }
                    else
                    {
                        SearchStartPoint.X = 0;
                    }
                    if (minTargetPoint.Y > SEARCH_MARGIN)
                    {
                        SearchStartPoint.Y = minTargetPoint.Y - SEARCH_MARGIN;
                    }
                    else
                    {
                        SearchStartPoint.Y = 0;
                    }

                    if ((maxTargetPoint.X + SEARCH_MARGIN) < image.Width)
                    {
                        SearchEndPoint.X = maxTargetPoint.X + SEARCH_MARGIN;
                    }
                    else
                    {
                        SearchEndPoint.X = image.Width;
                    }
                    if ((maxTargetPoint.Y + SEARCH_MARGIN) < image.Height)
                    {
                        SearchEndPoint.Y = maxTargetPoint.Y + SEARCH_MARGIN;
                    }
                    else
                    {
                        SearchEndPoint.Y = image.Height;
                    }
                    //Console.WriteLine("Search Window :(" + SearchStartPoint.X + "," + SearchStartPoint.Y + "),(" + SearchEndPoint.X + "," + SearchEndPoint.Y + ")");
                }
                else
                {
                    SearchStartPoint.X = 0;
                    SearchStartPoint.Y = 0;
                    SearchEndPoint.X = image.Width;
                    SearchEndPoint.Y = image.Height;
                    //Console.WriteLine("Search Window :(" + SearchStartPoint.X + "," + SearchStartPoint.Y + "),(" + SearchEndPoint.X + "," + SearchEndPoint.Y + ")");
                }
            }
            else
            {
                if (StopCounter >= Stablecount)
                {
                    if (stateCreateMobility != STOP)
                    {
                        f1.Stop_Click();
                        ResetCounters();
                        stateCreateMobility = STOP;
                        Console.WriteLine("Stop! Target is gone !!");
                    }
                    //Expand Search window
                    SearchStartPoint.X = 0;
                    SearchStartPoint.Y = 0;
                    SearchEndPoint.X = image.Width;
                    SearchEndPoint.Y = image.Height;
                    //Console.WriteLine("Search Window :(" + SearchStartPoint.X + "," + SearchStartPoint.Y + "),(" + SearchEndPoint.X + "," + SearchEndPoint.Y + ")");
                }
                else
                {
                    TurnRightCounter = 0;
                    TurnLeftCounter = 0;
                    GoCounter = 0;
                    BumperStopCounter = 0;
                    StopCounter = 0;
                    LostCounter++;
                }
                return;
            }

            //Control iRobot if target color is found
            if (tragetColorFound == true)
            {
                //Draw cross line on target color
                uBitmap.UnlockBitmap();
                Graphics dc = Graphics.FromImage(image);
                Pen p = new Pen(Color.Yellow, 2);
                //dc.DrawLine(p, 0, targetCentroid.Y, image.Width, targetCentroid.Y); // Draw yellow horizontal line
                //dc.DrawLine(p, targetCentroid.X, 0, targetCentroid.X, image.Height); // Draw yellow vertical line
                dc.DrawLine(p, minTargetPoint.X, minTargetPoint.Y, maxTargetPoint.X, minTargetPoint.Y); // Draw rectanlge of target objects
                dc.DrawLine(p, minTargetPoint.X, minTargetPoint.Y, minTargetPoint.X, maxTargetPoint.Y);
                dc.DrawLine(p, maxTargetPoint.X, minTargetPoint.Y, maxTargetPoint.X, maxTargetPoint.Y);
                dc.DrawLine(p, minTargetPoint.X, maxTargetPoint.Y, maxTargetPoint.X, maxTargetPoint.Y);
                dc.Dispose();
                uBitmap.LockBitmap();


                //Restric the distance from object
                if (targetPixel >=(image.Width*image.Height/32))
                {
                    if (stateCreateMobility != STOP)
                    {
                        f1.Stop_Click();
                        ResetCounters();
                        stateCreateMobility = STOP;
                        Console.WriteLine("Stop by Adjacency");
                    }
                    return;
                }
                // Control iRobot
                if (targetCentroid.X < ((image.Width / 2) - centralRange))
                {
                    //Service for iRobot @ Oct 30, 2008
                    //move turn left
                    if (TurnLeftCounter >= Stablecount)
                    {
                        if (stateCreateMobility != LEFT)
                        {
                            f1.TurnLeft_Click(50);
                            ResetCounters();
                            stateCreateMobility = LEFT; 
                            Console.WriteLine("Trun Left");
                        }
                    }
                    else
                    {
                        TurnLeftCounter++;
                        TurnRightCounter = 0;
                        GoCounter = 0;
                        StopCounter = 0;
                        BumperStopCounter = 0;
                        LostCounter = 0;
                    }
                }
                else if (targetCentroid.X > ((image.Width / 2) + centralRange))
                {
                    //Service for iRobot @ Oct 30, 2008
                    //move turn right
                    if (TurnRightCounter >= Stablecount)
                    {
                        if (stateCreateMobility != RIGHT)
                        {
                            f1.TurnRight_Click(50);
                            ResetCounters();
                            stateCreateMobility = RIGHT;
                            Console.WriteLine("Trun Right");
                        }
                    }
                    else
                    {
                        TurnRightCounter++;
                        TurnLeftCounter = 0;
                        GoCounter = 0;
                        StopCounter = 0;
                        BumperStopCounter = 0;
                        LostCounter = 0;
                    }
                }
                else   // In case of find out the red stuff within central range
                {
                    if ((f1.ReadSensor_Click(7) & 0x03) != 0)    //bumper is pressed
                    {
                        if (BumperStopCounter >= Stablecount)
                        {
                            if (stateCreateMobility != STOP)
                            {
                                f1.Stop_Click();
                                ResetCounters();
                                stateCreateMobility = STOP;
                                Console.WriteLine("Stop by bumper");
                            }
                        }
                        else
                        {
                            TurnRightCounter = 0;
                            TurnLeftCounter = 0;
                            GoCounter = 0;
                            StopCounter = 0;
                            LostCounter = 0;
                            BumperStopCounter++;
                        }
                    }
                    else
                    {
                        if (GoCounter >= Stablecount/2)
                        {
                            if (stateCreateMobility != GO)
                            {
                                f1.Go_Click();
                                ResetCounters();
                                stateCreateMobility = GO;
                                centralRange = 50;
                                Console.WriteLine("Go");
                            }
                        }
                        else
                        {
                            TurnRightCounter = 0;
                            TurnLeftCounter = 0;
                            GoCounter ++;
                            StopCounter = 0;
                            LostCounter = 0;
                            BumperStopCounter = 0;
                        } 
                    }
                }
            }
            else   //In case of fail to find out target color stuffs.
            {
                if (StopCounter >= Stablecount)
                {
                    if (stateCreateMobility != STOP)
                    {
                        f1.Stop_Click();
                        ResetCounters();
                        stateCreateMobility = STOP;
                        Console.WriteLine("Stop! Target is gone !!");
                    }
                    //Expand Search window
                    SearchStartPoint.X = 0;
                    SearchStartPoint.Y = 0;
                    SearchEndPoint.X = image.Width;
                    SearchEndPoint.Y = image.Height;
                    //Console.WriteLine("Search Window :(" + SearchStartPoint.X + "," + SearchStartPoint.Y + "),(" + SearchEndPoint.X + "," + SearchEndPoint.Y + ")");
                }
                else 
                {
                    TurnRightCounter = 0;
                    TurnLeftCounter = 0;
                    GoCounter = 0;
                    BumperStopCounter = 0;
                    StopCounter = 0;
                    LostCounter++;
                }
            }
            uBitmap.UnlockBitmap();
            uBitmap.Bitmap.Dispose();
        }

        //Turn the iRobot to Target
        private void TurnToTarget()
        {
            targetAngle = ((targetPoint.X-(IMAGE_WIDTH/2)) * (FOV/2)) / (IMAGE_WIDTH/2);
            Console.WriteLine("Target angle : " + targetAngle);

            if (targetAngle > 0)
            {
                f1.TurnRight_Click(20);
            }
            else if (targetAngle < 0)
            {
                f1.TurnLeft_Click(20);
            }
            f1.WaitForAngle(targetAngle);
            f1.Stop_Click();
        }


        private void ResetCounters()
        {
            TurnLeftCounter = 0;
            TurnRightCounter = 0;
            GoCounter = 0;
            StopCounter = 0;
            LostCounter = 0;
        }

        /// <summary>
        /// Convert RGB to HSV
        /// </summary>
        /// <param name="r">range : 0~255</param>
        /// <param name="g">range : 0~255</param>
        /// <param name="b">range : 0~255</param>
        /// <returns>HSVColorSpace:h(0~360),s(0~1),v(0~1)</returns>
        private HSVColorSpace ConvertRGB2HSV(byte r, byte g, byte b){
            HSVColorSpace result = new HSVColorSpace();
            double red=0, green=0, blue=0;

            double rgbMin = 0, rgbMax = 0;

            //Scale 0~255 to 0~1
            red = r / (double)255;
            green = g / (double)255;
            blue = b / (double)255;

            //Find out max and min
            rgbMin = ((green) <= (blue) ? ((red) <= (green) ? (red) : (green)) : ((red) <= (blue) ? (red) : (blue)));
            rgbMax = ((green) >= (blue) ? ((red) >= (green) ? (red) : (green)) : ((red) >= (blue) ? (red) : (blue)));

            result.v = rgbMax;
            if (result.v == 0)
            {
                result.h = result.s = 0;
                return result;
            }

            //Normalize to 1
            //red /=result.v;
            //green /=result.v;
            //blue /=result.v;
            //rgbMin = ((green) <= (blue) ? ((red) <= (green) ? (red) : (green)) : ((red) <= (blue) ? (red) : (blue)));
            //rgbMax = ((green) >= (blue) ? ((red) >= (green) ? (red) : (green)) : ((red) >= (blue) ? (red) : (blue)));


            //Compute Saturation
            result.s = (rgbMax - rgbMin)/rgbMax;
            if (result.s == 0)
            {
                result.h = 0;
                return result;
            }

            //Normalize saturatin to 1
            //red = (red - rgbMin) / (rgbMax - rgbMin);
            //green = (green - rgbMin) / (rgbMax - rgbMin);
            //blue = (blue - rgbMin) / (rgbMax - rgbMin);
            //rgbMin = ((green) <= (blue) ? ((red) <= (green) ? (red) : (green)) : ((red) <= (blue) ? (red) : (blue)));
            //rgbMax = ((green) >= (blue) ? ((red) >= (green) ? (red) : (green)) : ((red) >= (blue) ? (red) : (blue)));

            //Calculate Hue
            if (rgbMin == rgbMax)
            {
                result.h = 0;
            }
            else
            {
                if (rgbMax == red)
                {
                    result.h = 60 * (green - blue) / (rgbMax - rgbMin);
                    if (result.h < 0.0)
                    {
                        result.h += 360.0;
                    }
                }
                else if (rgbMax == green)
                {
                    result.h = 120.0 + 60.0 * (blue - red) / (rgbMax - rgbMin);
                }
                else if (rgbMax == blue)
                {
                    result.h = 240.0 + 60.0 * (red - green) / (rgbMax - rgbMin);
                }

            }
            return result;
        }


        private void MoveStepper(int direction, int numberOfSteps, bool halfStepping)
        {
            if (direction == 0) // Turn motor left
            {
                if (halfStepping == false)
                {
                    for (int i = 0; i < numberOfSteps; i++)
                    {
                        if (step == 1) step = 2;
                        else if (step == 2) step = 4;
                        else if (step == 4) step = 8;
                        else if (step == 8) step = 1;
                        else step = 1;

                        PortAccess.Output(888, step);

                        System.Threading.Thread.Sleep(100);
                    }
                }
                else
                {
                    for (int i = 0; i < numberOfSteps; i++)
                    {

                        if (step == 1) step = 3;
                        else if (step == 3) step = 2;
                        else if (step == 2) step = 6;
                        else if (step == 6) step = 4;
                        else if (step == 4) step = 12;
                        else if (step == 12) step = 8;
                        else if (step == 8) step = 9;
                        else if (step == 9) step = 1;
                        else step = 1;

                        PortAccess.Output(888, step);

                        System.Threading.Thread.Sleep(100);
                    }

                }

            }
            else // Turn motor right
            {

                if (halfStepping == false)
                {
                    for (int i = 0; i < numberOfSteps; i++)
                    {

                        if (step == 1) step = 8;
                        else if (step == 2) step = 1;
                        else if (step == 4) step = 2;
                        else if (step == 8) step = 4;
                        else step = 1;

                        PortAccess.Output(888, step);

                        System.Threading.Thread.Sleep(100);
                    }
                }
                else
                {
                    for (int i = 0; i < numberOfSteps; i++)
                    {
                        if (step == 3) step = 1;
                        else if (step == 2) step = 3;
                        else if (step == 6) step = 2;
                        else if (step == 4) step = 6;
                        else if (step == 12) step = 4;
                        else if (step == 8) step = 12;
                        else if (step == 9) step = 8;
                        else if (step == 1) step = 9;
                        else step = 1;

                        PortAccess.Output(888, step);

                        System.Threading.Thread.Sleep(100);
                    }
                }

            }

            PortAccess.Output(888, 0); // Releases motor
            wait = true; // Wait for stepper to complete its movements

        }
    }
}