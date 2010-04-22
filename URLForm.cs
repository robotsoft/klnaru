using System;
using System.Drawing;
using System.Collections;
using System.ComponentModel;
using System.Windows.Forms;

namespace motion
{
	/// <summary>
	/// Summary description for URLForm.
	/// </summary>
	public class URLForm : System.Windows.Forms.Form
	{
		private System.Windows.Forms.Label label1;
		private System.Windows.Forms.ComboBox urlCombo;
		private System.Windows.Forms.Button okButton;
		private System.Windows.Forms.Button cancelButton;

		private string url;
        private TextBox txtLogin;
        private TextBox txtPassword;
        private CheckBox chkPreAuth;
        private Label lblName;
        private Label lblPassword;
        private CheckBox chkHomePageAuth;

		/// <summary>
		/// Required designer variable.
		/// </summary>
		private System.ComponentModel.Container components = null;

		// URL property
		public string URL
		{
			get { return url; }
		}

        public string Login
        {
            get
            {
                return txtLogin.Text;
            }
        }

        public string Password
        {
            get
            {
                return txtPassword.Text;
            }
        }

        public bool AuthWithHomePage
        {
            get
            {
                return chkHomePageAuth.Checked;
            }
        }

        public bool PreAuthenticate
        {
            get
            {
                return chkPreAuth.Checked;
            }
        }


		// Description property
		public string Description
		{
			set
			{
				label1.Text = value;
			}
		}

		// URLs property
		public string[] URLs
		{
			set
			{
				urlCombo.Items.AddRange(value);
			}
		}


		// Constructor
		public URLForm()
		{
			//
			// Required for Windows Form Designer support
			//
			InitializeComponent();

			//
			// TODO: Add any constructor code after InitializeComponent call
			//
		}

		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		protected override void Dispose( bool disposing )
		{
			if( disposing )
			{
				if(components != null)
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
            this.label1 = new System.Windows.Forms.Label();
            this.urlCombo = new System.Windows.Forms.ComboBox();
            this.okButton = new System.Windows.Forms.Button();
            this.cancelButton = new System.Windows.Forms.Button();
            this.txtLogin = new System.Windows.Forms.TextBox();
            this.txtPassword = new System.Windows.Forms.TextBox();
            this.chkPreAuth = new System.Windows.Forms.CheckBox();
            this.lblName = new System.Windows.Forms.Label();
            this.lblPassword = new System.Windows.Forms.Label();
            this.chkHomePageAuth = new System.Windows.Forms.CheckBox();
            this.SuspendLayout();
            // 
            // label1
            // 
            this.label1.Location = new System.Drawing.Point(10, 10);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(332, 19);
            this.label1.TabIndex = 0;
            // 
            // urlCombo
            // 
            this.urlCombo.Location = new System.Drawing.Point(10, 30);
            this.urlCombo.Name = "urlCombo";
            this.urlCombo.Size = new System.Drawing.Size(325, 21);
            this.urlCombo.TabIndex = 1;
            // 
            // okButton
            // 
            this.okButton.DialogResult = System.Windows.Forms.DialogResult.OK;
            this.okButton.Location = new System.Drawing.Point(86, 119);
            this.okButton.Name = "okButton";
            this.okButton.Size = new System.Drawing.Size(75, 23);
            this.okButton.TabIndex = 8;
            this.okButton.Text = "OK";
            this.okButton.Click += new System.EventHandler(this.okButton_Click);
            // 
            // cancelButton
            // 
            this.cancelButton.DialogResult = System.Windows.Forms.DialogResult.Cancel;
            this.cancelButton.Location = new System.Drawing.Point(176, 119);
            this.cancelButton.Name = "cancelButton";
            this.cancelButton.Size = new System.Drawing.Size(75, 23);
            this.cancelButton.TabIndex = 9;
            this.cancelButton.Text = "Cancel";
            // 
            // txtLogin
            // 
            this.txtLogin.Location = new System.Drawing.Point(51, 60);
            this.txtLogin.Name = "txtLogin";
            this.txtLogin.Size = new System.Drawing.Size(100, 20);
            this.txtLogin.TabIndex = 3;
            // 
            // txtPassword
            // 
            this.txtPassword.Location = new System.Drawing.Point(232, 60);
            this.txtPassword.Name = "txtPassword";
            this.txtPassword.Size = new System.Drawing.Size(100, 20);
            this.txtPassword.TabIndex = 5;
            this.txtPassword.UseSystemPasswordChar = true;
            // 
            // chkPreAuth
            // 
            this.chkPreAuth.AutoSize = true;
            this.chkPreAuth.Checked = true;
            this.chkPreAuth.CheckState = System.Windows.Forms.CheckState.Checked;
            this.chkPreAuth.Location = new System.Drawing.Point(10, 86);
            this.chkPreAuth.Name = "chkPreAuth";
            this.chkPreAuth.Size = new System.Drawing.Size(104, 17);
            this.chkPreAuth.TabIndex = 6;
            this.chkPreAuth.Text = "Pre-authenticate";
            this.chkPreAuth.UseVisualStyleBackColor = true;
            // 
            // lblName
            // 
            this.lblName.AutoSize = true;
            this.lblName.Location = new System.Drawing.Point(7, 60);
            this.lblName.Name = "lblName";
            this.lblName.Size = new System.Drawing.Size(38, 13);
            this.lblName.TabIndex = 2;
            this.lblName.Text = "Name:";
            // 
            // lblPassword
            // 
            this.lblPassword.AutoSize = true;
            this.lblPassword.Location = new System.Drawing.Point(173, 60);
            this.lblPassword.Name = "lblPassword";
            this.lblPassword.Size = new System.Drawing.Size(56, 13);
            this.lblPassword.TabIndex = 4;
            this.lblPassword.Text = "Password:";
            // 
            // chkHomePageAuth
            // 
            this.chkHomePageAuth.AutoSize = true;
            this.chkHomePageAuth.Checked = true;
            this.chkHomePageAuth.CheckState = System.Windows.Forms.CheckState.Checked;
            this.chkHomePageAuth.Location = new System.Drawing.Point(149, 86);
            this.chkHomePageAuth.Name = "chkHomePageAuth";
            this.chkHomePageAuth.Size = new System.Drawing.Size(183, 17);
            this.chkHomePageAuth.TabIndex = 7;
            this.chkHomePageAuth.Text = "Authenticate with home page first";
            this.chkHomePageAuth.UseVisualStyleBackColor = true;
            // 
            // URLForm
            // 
            this.AcceptButton = this.okButton;
            this.AutoScaleBaseSize = new System.Drawing.Size(5, 13);
            this.CancelButton = this.cancelButton;
            this.ClientSize = new System.Drawing.Size(344, 154);
            this.Controls.Add(this.chkHomePageAuth);
            this.Controls.Add(this.lblPassword);
            this.Controls.Add(this.lblName);
            this.Controls.Add(this.chkPreAuth);
            this.Controls.Add(this.txtPassword);
            this.Controls.Add(this.txtLogin);
            this.Controls.Add(this.cancelButton);
            this.Controls.Add(this.okButton);
            this.Controls.Add(this.urlCombo);
            this.Controls.Add(this.label1);
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.FixedToolWindow;
            this.MaximizeBox = false;
            this.MinimizeBox = false;
            this.Name = "URLForm";
            this.ShowInTaskbar = false;
            this.StartPosition = System.Windows.Forms.FormStartPosition.CenterParent;
            this.Text = "Open URL";
            this.ResumeLayout(false);
            this.PerformLayout();

		}
		#endregion

		// On "Ok" button
		private void okButton_Click(object sender, System.EventArgs e)
		{
			url = urlCombo.Text;
		}
	}
}
