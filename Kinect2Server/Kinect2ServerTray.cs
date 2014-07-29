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

        private void OnConnectionChanged(object sender, EventArgs e)
        {
            trayIcon.Icon = (service.IsConnected)
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
