// Motion Detector
//
// Copyright © Andrew Kirillov, 2005
// andrew.kirillov@gmail.com
//

using System;
using System.Drawing;
using System.Collections;
using System.ComponentModel;
using System.Windows.Forms;
using System.Data;
using System.Threading;

using videosource;
//using iRobotDeviceDriver;

namespace motion
{
	/// <summary>
	/// Summary description for MainForm
	/// </summary>
	public class MainForm : System.Windows.Forms.Form
	{
		// statistics
		private const int	statLength = 15;
		private int			statIndex = 0, statReady = 0;
		private int[]		statCount = new int[statLength];

        private MotionDetector1 detector = new MotionDetector1();        

		private System.Windows.Forms.MenuItem fileItem;
		private System.Windows.Forms.MenuItem openFileItem;
		private System.Windows.Forms.MenuItem menuItem1;
		private System.Windows.Forms.MenuItem exitFileItem;
		private System.Windows.Forms.OpenFileDialog ofd;
		private System.Windows.Forms.MainMenu mainMenu;
		private System.Timers.Timer timer;
		private System.Windows.Forms.StatusBar statusBar;
		private System.Windows.Forms.StatusBarPanel fpsPanel;
		private System.Windows.Forms.Panel panel;
        private motion.CameraWindow cameraWindow;
		private System.Windows.Forms.MenuItem openURLFileItem;
		private System.Windows.Forms.MenuItem openMMSFileItem;
		private System.Windows.Forms.MenuItem openLocalFileItem;
        private System.Windows.Forms.MenuItem openMJEPGFileItem;
		private System.Windows.Forms.MenuItem helpItem;
        private System.Windows.Forms.MenuItem aboutHelpItem;
        private TrackBar thresholdTrackBar;
        private Label label1;
        private Label thresholdLabel;
        private IContainer components;

        private TrackBar pixelsPerStepTrackBar;
        private Label pixelsPerStepLabel;
        private Label label2;

        public int StableCounter = 2;
        public int ColorTolerance = 8;
        public bool halfStepping = false;
        public bool MouseClickOnImage = false;
        private Button ColorTrackingOn;
        public Point MouseLocationOnImage = new Point();
        private Button Left;
        private Button Right;
        private Button Stop;
        private Button Back;
        private Button Go;
        public bool ColorTrackingOnFlag = false;
        private const int STOP = 0;
        private const int GO = 1;
        private const int LEFT = 2;
        private const int RIGHT = 3;
        private const int BACK = 4;
        public int ManualControlFlag = STOP;

		public MainForm()
		{
			//
			// Required for Windows Form Designer support
			//
			InitializeComponent();
			//
			// TODO: Add any constructor code after InitializeComponent call
			//
            //dd = new CreateDeviceDriver();
            cameraWindow.MouseDown += new MouseEventHandler(cameraWindow_MouseDown);
            
		}

		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		protected override void Dispose( bool disposing )
		{
			if( disposing )
			{
				if (components != null) 
				{
					components.Dispose();
				}
			}
			base.Dispose( disposing );
		}

		#region Windows Form Designer generated code
		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		private void InitializeComponent()
		{
            this.components = new System.ComponentModel.Container();
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(MainForm));
            this.mainMenu = new System.Windows.Forms.MainMenu(this.components);
            this.fileItem = new System.Windows.Forms.MenuItem();
            this.openFileItem = new System.Windows.Forms.MenuItem();
            this.openURLFileItem = new System.Windows.Forms.MenuItem();
            this.openMJEPGFileItem = new System.Windows.Forms.MenuItem();
            this.openMMSFileItem = new System.Windows.Forms.MenuItem();
            this.openLocalFileItem = new System.Windows.Forms.MenuItem();
            this.menuItem1 = new System.Windows.Forms.MenuItem();
            this.exitFileItem = new System.Windows.Forms.MenuItem();
            this.helpItem = new System.Windows.Forms.MenuItem();
            this.aboutHelpItem = new System.Windows.Forms.MenuItem();
            this.ofd = new System.Windows.Forms.OpenFileDialog();
            this.timer = new System.Timers.Timer();
            this.statusBar = new System.Windows.Forms.StatusBar();
            this.fpsPanel = new System.Windows.Forms.StatusBarPanel();
            this.panel = new System.Windows.Forms.Panel();
            this.Left = new System.Windows.Forms.Button();
            this.Right = new System.Windows.Forms.Button();
            this.Stop = new System.Windows.Forms.Button();
            this.Back = new System.Windows.Forms.Button();
            this.Go = new System.Windows.Forms.Button();
            this.ColorTrackingOn = new System.Windows.Forms.Button();
            this.pixelsPerStepLabel = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.pixelsPerStepTrackBar = new System.Windows.Forms.TrackBar();
            this.thresholdLabel = new System.Windows.Forms.Label();
            this.label1 = new System.Windows.Forms.Label();
            this.thresholdTrackBar = new System.Windows.Forms.TrackBar();
            this.cameraWindow = new motion.CameraWindow();
            ((System.ComponentModel.ISupportInitialize)(this.timer)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.fpsPanel)).BeginInit();
            this.panel.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.pixelsPerStepTrackBar)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.thresholdTrackBar)).BeginInit();
            this.SuspendLayout();
            // 
            // mainMenu
            // 
            this.mainMenu.MenuItems.AddRange(new System.Windows.Forms.MenuItem[] {
            this.fileItem,
            this.helpItem});
            // 
            // fileItem
            // 
            this.fileItem.Index = 0;
            this.fileItem.MenuItems.AddRange(new System.Windows.Forms.MenuItem[] {
            this.openFileItem,
            this.openURLFileItem,
            this.openMJEPGFileItem,
            this.openMMSFileItem,
            this.openLocalFileItem,
            this.menuItem1,
            this.exitFileItem});
            this.fileItem.Text = "&File";
            // 
            // openFileItem
            // 
            this.openFileItem.Index = 0;
            this.openFileItem.Shortcut = System.Windows.Forms.Shortcut.CtrlO;
            this.openFileItem.Text = "&Open";
            this.openFileItem.Click += new System.EventHandler(this.openFileItem_Click);
            // 
            // openURLFileItem
            // 
            this.openURLFileItem.Index = 1;
            this.openURLFileItem.Text = "Open JPEG &URL";
            this.openURLFileItem.Click += new System.EventHandler(this.openURLFileItem_Click);
            // 
            // openMJEPGFileItem
            // 
            this.openMJEPGFileItem.Index = 2;
            this.openMJEPGFileItem.Text = "Open M&JPEG URL";
            this.openMJEPGFileItem.Click += new System.EventHandler(this.openMJEPGFileItem_Click);
            // 
            // openMMSFileItem
            // 
            this.openMMSFileItem.Index = 3;
            this.openMMSFileItem.Text = "Open &MMS Stream";
            this.openMMSFileItem.Click += new System.EventHandler(this.openMMSFileItem_Click);
            // 
            // openLocalFileItem
            // 
            this.openLocalFileItem.Index = 4;
            this.openLocalFileItem.Text = "Open &Local Device";
            this.openLocalFileItem.Click += new System.EventHandler(this.openLocalFileItem_Click);
            // 
            // menuItem1
            // 
            this.menuItem1.Index = 5;
            this.menuItem1.Text = "-";
            // 
            // exitFileItem
            // 
            this.exitFileItem.Index = 6;
            this.exitFileItem.Text = "E&xit";
            this.exitFileItem.Click += new System.EventHandler(this.exitFileItem_Click);
            // 
            // helpItem
            // 
            this.helpItem.Index = 1;
            this.helpItem.MenuItems.AddRange(new System.Windows.Forms.MenuItem[] {
            this.aboutHelpItem});
            this.helpItem.Text = "&Help";
            // 
            // aboutHelpItem
            // 
            this.aboutHelpItem.Index = 0;
            this.aboutHelpItem.Text = "&About";
            this.aboutHelpItem.Click += new System.EventHandler(this.aboutHelpItem_Click);
            // 
            // ofd
            // 
            this.ofd.Filter = "AVI files (*.avi)|*.avi";
            this.ofd.Title = "Open movie";
            // 
            // timer
            // 
            this.timer.Interval = 1000;
            this.timer.SynchronizingObject = this;
            this.timer.Elapsed += new System.Timers.ElapsedEventHandler(this.timer_Elapsed);
            // 
            // statusBar
            // 
            this.statusBar.Location = new System.Drawing.Point(0, 606);
            this.statusBar.Name = "statusBar";
            this.statusBar.Panels.AddRange(new System.Windows.Forms.StatusBarPanel[] {
            this.fpsPanel});
            this.statusBar.ShowPanels = true;
            this.statusBar.Size = new System.Drawing.Size(325, 22);
            this.statusBar.TabIndex = 1;
            // 
            // fpsPanel
            // 
            this.fpsPanel.AutoSize = System.Windows.Forms.StatusBarPanelAutoSize.Spring;
            this.fpsPanel.Name = "fpsPanel";
            this.fpsPanel.Width = 308;
            // 
            // panel
            // 
            this.panel.BorderStyle = System.Windows.Forms.BorderStyle.Fixed3D;
            this.panel.Controls.Add(this.Left);
            this.panel.Controls.Add(this.Right);
            this.panel.Controls.Add(this.Stop);
            this.panel.Controls.Add(this.Back);
            this.panel.Controls.Add(this.Go);
            this.panel.Controls.Add(this.ColorTrackingOn);
            this.panel.Controls.Add(this.pixelsPerStepLabel);
            this.panel.Controls.Add(this.label2);
            this.panel.Controls.Add(this.pixelsPerStepTrackBar);
            this.panel.Controls.Add(this.thresholdLabel);
            this.panel.Controls.Add(this.label1);
            this.panel.Controls.Add(this.thresholdTrackBar);
            this.panel.Controls.Add(this.cameraWindow);
            this.panel.Dock = System.Windows.Forms.DockStyle.Fill;
            this.panel.Location = new System.Drawing.Point(0, 0);
            this.panel.Name = "panel";
            this.panel.Size = new System.Drawing.Size(325, 606);
            this.panel.TabIndex = 2;
            // 
            // Left
            // 
            this.Left.Location = new System.Drawing.Point(42, 507);
            this.Left.Name = "Left";
            this.Left.Size = new System.Drawing.Size(75, 23);
            this.Left.TabIndex = 13;
            this.Left.Text = "Left";
            this.Left.UseVisualStyleBackColor = true;
            this.Left.Click += new System.EventHandler(this.Left_Click);
            // 
            // Right
            // 
            this.Right.Location = new System.Drawing.Point(205, 507);
            this.Right.Name = "Right";
            this.Right.Size = new System.Drawing.Size(75, 23);
            this.Right.TabIndex = 12;
            this.Right.Text = "Right";
            this.Right.UseVisualStyleBackColor = true;
            this.Right.Click += new System.EventHandler(this.Right_Click);
            // 
            // Stop
            // 
            this.Stop.Location = new System.Drawing.Point(123, 507);
            this.Stop.Name = "Stop";
            this.Stop.Size = new System.Drawing.Size(75, 23);
            this.Stop.TabIndex = 11;
            this.Stop.Text = "Stop";
            this.Stop.UseVisualStyleBackColor = true;
            this.Stop.Click += new System.EventHandler(this.Stop_Click);
            // 
            // Back
            // 
            this.Back.Location = new System.Drawing.Point(123, 555);
            this.Back.Name = "Back";
            this.Back.Size = new System.Drawing.Size(75, 23);
            this.Back.TabIndex = 10;
            this.Back.Text = "Back";
            this.Back.UseVisualStyleBackColor = true;
            this.Back.Click += new System.EventHandler(this.Back_Click);
            // 
            // Go
            // 
            this.Go.Location = new System.Drawing.Point(123, 457);
            this.Go.Name = "Go";
            this.Go.Size = new System.Drawing.Size(75, 23);
            this.Go.TabIndex = 9;
            this.Go.Text = "Go";
            this.Go.UseVisualStyleBackColor = true;
            this.Go.Click += new System.EventHandler(this.Go_Click);
            // 
            // ColorTrackingOn
            // 
            this.ColorTrackingOn.Location = new System.Drawing.Point(123, 362);
            this.ColorTrackingOn.Name = "ColorTrackingOn";
            this.ColorTrackingOn.Size = new System.Drawing.Size(75, 57);
            this.ColorTrackingOn.TabIndex = 8;
            this.ColorTrackingOn.Text = "Color Tracking On";
            this.ColorTrackingOn.UseVisualStyleBackColor = true;
            this.ColorTrackingOn.Click += new System.EventHandler(this.ColorTrackingOn_Click);
            // 
            // pixelsPerStepLabel
            // 
            this.pixelsPerStepLabel.AutoSize = true;
            this.pixelsPerStepLabel.Location = new System.Drawing.Point(99, 301);
            this.pixelsPerStepLabel.Name = "pixelsPerStepLabel";
            this.pixelsPerStepLabel.Size = new System.Drawing.Size(13, 13);
            this.pixelsPerStepLabel.TabIndex = 7;
            this.pixelsPerStepLabel.Text = "8";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(10, 301);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(88, 13);
            this.label2.TabIndex = 6;
            this.label2.Text = "Color Tolerance :";
            // 
            // pixelsPerStepTrackBar
            // 
            this.pixelsPerStepTrackBar.Location = new System.Drawing.Point(13, 315);
            this.pixelsPerStepTrackBar.Maximum = 30;
            this.pixelsPerStepTrackBar.Name = "pixelsPerStepTrackBar";
            this.pixelsPerStepTrackBar.Size = new System.Drawing.Size(297, 45);
            this.pixelsPerStepTrackBar.TabIndex = 5;
            this.pixelsPerStepTrackBar.TickStyle = System.Windows.Forms.TickStyle.TopLeft;
            this.pixelsPerStepTrackBar.Value = 8;
            this.pixelsPerStepTrackBar.Scroll += new System.EventHandler(this.pixelsStepTrackBar_Scroll);
            // 
            // thresholdLabel
            // 
            this.thresholdLabel.AutoSize = true;
            this.thresholdLabel.Location = new System.Drawing.Point(99, 245);
            this.thresholdLabel.Name = "thresholdLabel";
            this.thresholdLabel.Size = new System.Drawing.Size(13, 13);
            this.thresholdLabel.TabIndex = 4;
            this.thresholdLabel.Text = "2";
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.BackColor = System.Drawing.Color.Transparent;
            this.label1.Location = new System.Drawing.Point(10, 245);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(83, 13);
            this.label1.TabIndex = 3;
            this.label1.Text = "Stable Counter :";
            // 
            // thresholdTrackBar
            // 
            this.thresholdTrackBar.Location = new System.Drawing.Point(13, 261);
            this.thresholdTrackBar.Maximum = 30;
            this.thresholdTrackBar.Name = "thresholdTrackBar";
            this.thresholdTrackBar.Size = new System.Drawing.Size(297, 45);
            this.thresholdTrackBar.TabIndex = 2;
            this.thresholdTrackBar.TickStyle = System.Windows.Forms.TickStyle.TopLeft;
            this.thresholdTrackBar.Value = 2;
            this.thresholdTrackBar.Scroll += new System.EventHandler(this.thresholdTrackBar_Scroll);
            // 
            // cameraWindow
            // 
            this.cameraWindow.BackColor = System.Drawing.SystemColors.AppWorkspace;
            this.cameraWindow.Camera = null;
            this.cameraWindow.Location = new System.Drawing.Point(0, 0);
            this.cameraWindow.Name = "cameraWindow";
            this.cameraWindow.Size = new System.Drawing.Size(320, 240);
            this.cameraWindow.TabIndex = 1;
            this.cameraWindow.Click += new System.EventHandler(this.cameraWindow_Click_1);
            // 
            // MainForm
            // 
            this.AutoScaleBaseSize = new System.Drawing.Size(5, 13);
            this.ClientSize = new System.Drawing.Size(325, 628);
            this.Controls.Add(this.panel);
            this.Controls.Add(this.statusBar);
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.FixedSingle;
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.MaximizeBox = false;
            this.Menu = this.mainMenu;
            this.Name = "MainForm";
            this.StartPosition = System.Windows.Forms.FormStartPosition.CenterScreen;
            this.Text = "Laser Following Camera";
            this.Load += new System.EventHandler(this.MainForm_Load);
            this.Closing += new System.ComponentModel.CancelEventHandler(this.MainForm_Closing);
            ((System.ComponentModel.ISupportInitialize)(this.timer)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.fpsPanel)).EndInit();
            this.panel.ResumeLayout(false);
            this.panel.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.pixelsPerStepTrackBar)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.thresholdTrackBar)).EndInit();
            this.ResumeLayout(false);

		}
		#endregion

		/// <summary>
		/// The main entry point for the application.
		/// </summary>
		[STAThread]
		static void Main() 
		{
			Application.Run(new MainForm());
		}

		// On form closing
		private void MainForm_Closing(object sender, System.ComponentModel.CancelEventArgs e)
		{
			CloseFile();		
		}

		// Close the main form
		private void exitFileItem_Click(object sender, System.EventArgs e)
		{
			this.Close();
		}

		// On "Help->About"
		private void aboutHelpItem_Click(object sender, System.EventArgs e)
		{
			AboutForm form = new AboutForm();
			form.ShowDialog();
		}

		// Open file
		private void openFileItem_Click(object sender, System.EventArgs e)
		{
			if (ofd.ShowDialog() == DialogResult.OK)
			{
				// create video source
				VideoFileSource fileSource = new VideoFileSource();
				fileSource.VideoSource = ofd.FileName;

				// open it
				OpenVideoSource(fileSource);
			}		
		}

		// Open URL
		private void openURLFileItem_Click(object sender, System.EventArgs e)
		{
			URLForm	form = new URLForm();

			form.Description = "Enter URL of an updating JPEG from a web camera:";
			form.URLs = new string[]
				{
					"http://aleksandriacamk1.it.helsinki.fi/axis-cgi/jpg/image.cgi?resolution=320x240",
					"http://stareat.it.helsinki.fi/axis-cgi/jpg/image.cgi?resolution=320x240",
					"http://194.18.89.220/axis-cgi/jpg/image.cgi?resolution=320x240",
					"http://212.247.228.34/axis-cgi/jpg/image.cgi?resolution=352x240"
				};

			if (form.ShowDialog(this) == DialogResult.OK)
			{
				// create video source
				JPEGSource jpegSource = new JPEGSource();
				jpegSource.VideoSource = form.URL;
                jpegSource.Password = form.Password;
                jpegSource.Login = form.Name;
                jpegSource.PreAuthenticate = form.PreAuthenticate;

				// open it
				OpenVideoSource(jpegSource);
			}
		}

		// Open MJPEG URL
		private void openMJEPGFileItem_Click(object sender, System.EventArgs e)
		{
			URLForm	form = new URLForm();

			form.Description = "Enter URL of an MJPEG video stream:";
			form.URLs = new string[]
				{
                    "http://192.168.1.3/img/video.mjpeg",
					"http://hanselman.dyndns.org:81/mjpeg.cgi",
                    "http://sun.jerseyinsight.com/trafficbeaumont/nph-update.cgi",
					"http://peeper.axisinc.com/nph-manupdate.cgi",
					"http://marc15ter.vac.hu/nphMotionJpeg?Resolution=320x240&Quality=Standard",
					"http://213.200.232.69:8080/axis-cgi/mjpg/video.cgi?resolution=320x240"
                 
				};

			if (form.ShowDialog(this) == DialogResult.OK)
			{
				// create video source
				MJPEGSource mjpegSource = new MJPEGSource();
				mjpegSource.VideoSource = form.URL;
                mjpegSource.Login = form.Login;
                mjpegSource.Password = form.Password;
                mjpegSource.PreAuthenticate = form.PreAuthenticate;
                mjpegSource.AuthWithHomePage = form.AuthWithHomePage;

				// open it
				OpenVideoSource(mjpegSource);
			}
		}

		// Open MMS
		private void openMMSFileItem_Click(object sender, System.EventArgs e)
		{
			MMSForm	form = new MMSForm();

			if (form.ShowDialog(this) == DialogResult.OK)
			{
				// create video source
				VideoStream mmsSource = new VideoStream();
				mmsSource.VideoSource = form.URL;

				// open it
				OpenVideoSource(mmsSource);
			}
		}

		// Open local capture device
		private void openLocalFileItem_Click(object sender, System.EventArgs e)
		{
			CaptureDeviceForm form = new CaptureDeviceForm();

			if (form.ShowDialog(this) == DialogResult.OK)
			{
				// create video source
				CaptureDevice localSource = new CaptureDevice();
				localSource.VideoSource = form.Device;

				// open it
				OpenVideoSource(localSource);
			}
		}

		// Open video source
		private void OpenVideoSource(IVideoSource source)
		{
			// set busy cursor
			this.Cursor = Cursors.WaitCursor;

			// close previous file
			CloseFile();

			// create camera
			Camera camera = new Camera(source, detector);
			// start camera
			camera.Start();

			// attach camera to camera window
			cameraWindow.Camera = camera;

			// reset statistics
			statIndex = statReady = 0;

			// start timer
			timer.Start();

			this.Cursor = Cursors.Default;
		}

		// Close current file
		private void CloseFile()
		{
			Camera	camera = cameraWindow.Camera;

			if (camera != null)
			{
				// detach camera from camera window
				cameraWindow.Camera = null;

				// signal camera to stop
				camera.SignalToStop();
				// wait for the camera
				camera.WaitForStop();

				camera = null;

				if (detector != null)
					detector.Reset();
			}
		}

		// On timer event - gather statistic
		private void timer_Elapsed(object sender, System.Timers.ElapsedEventArgs e)
		{
			Camera	camera = cameraWindow.Camera;
		
			if (camera != null)
			{
				// get number of frames for the last second
				statCount[statIndex] = camera.FramesReceived;

				// increment indexes
				if (++statIndex >= statLength)
					statIndex = 0;
				if (statReady < statLength)
					statReady++;

				float	fps = 0;

				// calculate average value
				for (int i = 0; i < statReady; i++)
				{
					fps += statCount[i];
				}
				fps /= statReady;

				statCount[statIndex] = 0;

				fpsPanel.Text = fps.ToString("F2") + " fps";
			}
		}


		// Update motion detector
		private void SetMotionDetector()
		{
			Camera	camera = cameraWindow.Camera;
		
			if (camera != null)
			{
				camera.Lock();
				camera.MotionDetector = detector;

				// reset statistics
				statIndex = statReady = 0;
				camera.Unlock();
			}
		}

        private void thresholdTrackBar_Scroll(object sender, EventArgs e)
        {
            // Update value on the label
            thresholdLabel.Text = thresholdTrackBar.Value.ToString();
            StableCounter = thresholdTrackBar.Value;
        }

        private void pixelsStepTrackBar_Scroll(object sender, EventArgs e)
        {
            //Update text on label
            pixelsPerStepLabel.Text = pixelsPerStepTrackBar.Value.ToString();
            ColorTolerance = pixelsPerStepTrackBar.Value;
        }
        
        private void MainForm_Load(object sender, EventArgs e)
        {
            detector.mForm = this;
        }


        private void cameraWindow_Click_1(object sender, EventArgs e)
        {
            //MouseLocationOnImage.X = System.Windows.Forms.Control.MousePosition.X - this.Location.X - 5;
            //MouseLocationOnImage.Y = System.Windows.Forms.Control.MousePosition.Y - this.Location.Y - 51;
            MouseLocationOnImage.X = mousePosition.X;
            MouseLocationOnImage.Y = mousePosition.Y;
        
            MouseClickOnImage = true;
            Console.WriteLine(MouseLocationOnImage.X + "," + MouseLocationOnImage.Y);
            //Console.WriteLine(mousePosition.X + "," + mousePosition.Y);
        }

        Point mousePosition;

        void cameraWindow_MouseDown(object sender, System.Windows.Forms.MouseEventArgs e)
        {
            if (e.Button == MouseButtons.Left)
            {
                mousePosition = e.Location;
            }
        }

        private void ColorTrackingOn_Click(object sender, EventArgs e)
        {
            if (ColorTrackingOn.Text.Equals("Color Tracking On"))
            {
                ColorTrackingOn.Text = "Color Tracking Off";
                ColorTrackingOnFlag = true;
                this.Go.Enabled = false;
                this.Stop.Enabled = false;
                this.Back.Enabled = false;
                this.Right.Enabled = false;
                this.Left.Enabled = false;
            }
            else
            {
                ColorTrackingOn.Text = "Color Tracking On";
                ColorTrackingOnFlag = false;
                this.Go.Enabled = true;
                this.Stop.Enabled = true;
                this.Back.Enabled = true;
                this.Right.Enabled = true;
                this.Left.Enabled = true;
            }
        }

        private void Go_Click(object sender, EventArgs e)
        {
            ManualControlFlag = GO;
        }

        private void Stop_Click(object sender, EventArgs e)
        {
            ManualControlFlag = STOP;
        }

        private void Back_Click(object sender, EventArgs e)
        {
            ManualControlFlag = BACK;
        }

        private void Right_Click(object sender, EventArgs e)
        {
            ManualControlFlag = RIGHT;
        }

        private void Left_Click(object sender, EventArgs e)
        {
            ManualControlFlag = LEFT;
        }

	}
}
