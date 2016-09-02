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
using System.ComponentModel;
using System.Configuration.Install;
using System.ServiceProcess;

[RunInstaller(true)]
public class Kinect2ServerInstaller : Installer
{
    private ServiceProcessInstaller processInstaller;
    private ServiceInstaller serviceInstaller;

    /// <summary>
    /// The installer hook for installing this service.  
    /// 
    /// To install and uninstall this service into Windows, use:
    /// (http://msdn.microsoft.com/en-us/library/aa984379%28v=vs.71%29.aspx)
    /// 
    /// InstallUtil /LogToConsole=true Kinect2Server.exe
    /// InstallUtil /u Kinect2Server.exe
    /// 
    /// </summary>
    public Kinect2ServerInstaller()
    {
        processInstaller = new ServiceProcessInstaller();
        serviceInstaller = new ServiceInstaller();

        processInstaller.Account = ServiceAccount.LocalSystem;
        serviceInstaller.StartType = ServiceStartMode.Manual;
        serviceInstaller.ServiceName = "Kinect2Server"; //must match Kinect2ServerService.ServiceName

        Installers.Add(serviceInstaller);
        Installers.Add(processInstaller);
    }
}  
