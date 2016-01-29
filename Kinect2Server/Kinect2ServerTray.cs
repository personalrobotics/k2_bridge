/*******************************************************************************************************
Copyright (c) 2013, Carnegie Mellon University
All rights reserved.
Authors: Anurag Jakhotia<ajakhoti@andrew.cmu.edu>, Prasanna Velagapudi<pkv@cs.cmu.edu>

Redistribution and use in source and binary forms, with or without modification, are permitted provided 
that the following conditions are met:

 -	Redistributions of source code must retain the above copyright notice, this list of conditions and 
    the following disclaimer.
 -	Redistributions in binary form must reproduce the above copyright notice, this list of conditions 
    and the following disclaimer in the documentation and/or other materials provided with the 
    distribution.
 -	Neither the name of Carnegie Mellon University nor the names of its contributors may be used to 
    endorse or promote products derived from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED 
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************************************/
using System;
using System.Drawing;
using System.Reflection;
using System.Windows.Forms;

namespace PersonalRobotics.Kinect2Server
{
    public class Kinect2ServerTray : Form
    { 
        private NotifyIcon trayIcon = new NotifyIcon();
        private ContextMenu trayMenu = new ContextMenu();
        private Kinect2ServerService service = new Kinect2ServerService();

        public Kinect2ServerTray()
        {
            // Register for connection events.
            service.IsConnectedChanged += OnConnectionChanged;

            // Create a simple tray menu with only one item.
            trayMenu.MenuItems.Add("Exit", OnExit);
 
            // Change the icon and text of the tray icon.
            trayIcon.Text = "Kinect 2 Server";
            trayIcon.Icon = (service.IsConnected)
                ? Kinect2Server.Properties.Resources.KinectGreen
                : Kinect2Server.Properties.Resources.KinectRed;
 
            // Add menu to tray icon and show it.
            trayIcon.ContextMenu = trayMenu;
            trayIcon.Visible = true;
        }

        private void OnConnectionChanged(object sender, IsConnectedChangedEventArgs e)
        {
            trayIcon.Icon = (e.IsConnected)
                ? Kinect2Server.Properties.Resources.KinectGreen
                : Kinect2Server.Properties.Resources.KinectRed;
        }

        protected override void OnLoad(EventArgs e)
        {
            Visible = false; // Hide form window.
            ShowInTaskbar = false; // Remove from taskbar.
 
            // Start the Kinect server.
            typeof(Kinect2ServerService)
                .GetMethod("OnStart", BindingFlags.Instance | BindingFlags.NonPublic)
                .Invoke(service, new object[] { null });

            // Load the placeholder form.
            base.OnLoad(e);
        }
 
        private void OnExit(object sender, EventArgs e)
        {
            
            // Shutdown the entire application.
            Application.Exit();
        }
 
        protected override void Dispose(bool isDisposing)
        {
            if (isDisposing)
            {
                // Release the tray resources.
                trayIcon.Dispose();
                trayMenu.Dispose();
            }

            // Stop the Kinect server.
            typeof(Kinect2ServerService)
                .GetMethod("OnStop", BindingFlags.Instance | BindingFlags.NonPublic)
                .Invoke(service, null);

            // Remove the placeholder form.
            base.Dispose(isDisposing);
        }
    }
}
