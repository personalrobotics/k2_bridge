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
using System.Collections;
using System.Net;
using System.Net.Sockets;
using System.Threading;

namespace PersonalRobotics.Kinect2Server
{
    /// <summary>
    /// Contains bookkeeping information for a TCP socket client.
    /// </summary>
    public sealed class Client : IDisposable
    {
        public readonly Socket socket;
        public readonly Semaphore sends;

        public Client(Socket socket, int concurrentSends)
        {
            this.socket = socket;
            this.sends = new Semaphore(concurrentSends, concurrentSends);
        }

        public void Dispose()
        {
            socket.Dispose();
            sends.Dispose();
        }
    }

    /// <summary>
    /// Defines a network connector that allows multiple subscribers to a binary data stream.
    /// </summary>
    public sealed class AsyncNetworkConnector : IDisposable
    {
        ArrayList connectedClients = ArrayList.Synchronized(new ArrayList());

        public readonly int port;
        readonly int concurrentSends;
        readonly Socket listenerSocket;

        public bool HasClients { get { return connectedClients.Count > 0; } }

        public AsyncNetworkConnector(int port, int concurrentSends = 3)
        {
            this.listenerSocket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
            this.port = port;
            this.concurrentSends = concurrentSends;
        }

        public void Listen()
        {
            IPEndPoint endpoint = new IPEndPoint(IPAddress.Any, this.port);
            this.listenerSocket.Bind(endpoint);
            this.listenerSocket.Listen(100);
            this.listenerSocket.BeginAccept(new AsyncCallback(OnAccept), this);
        }

        public void OnAccept(IAsyncResult ar)
        {
            try
            {
                AsyncNetworkConnector connector = (AsyncNetworkConnector)ar.AsyncState;
                Socket socket = connector.listenerSocket.EndAccept(ar);
                connector.connectedClients.Add(new Client(socket, this.concurrentSends));
                connector.listenerSocket.BeginAccept(new AsyncCallback(OnAccept), connector);
                Console.WriteLine("Connected to: " + socket.RemoteEndPoint); // TODO: put this in the event log instead.
            }
            catch (SocketException e)
            {
                Console.WriteLine("Error accepting client: " + e);
            }
        }

        public void OnSend(IAsyncResult ar)
        {
            Client client = (Client)ar.AsyncState;
            try
            {
                if (client.socket.Connected)
                {
                    client.socket.EndSend(ar);
                    client.sends.Release();
                }
            }
            catch (SocketException)
            {
                Console.WriteLine("Disconnected from: " + client.socket.RemoteEndPoint);
                client.socket.Close();
                this.connectedClients.Remove(client);
            }
        }

        public void Send(Client client, byte[] data)
        {
            try
            {
                if (client.sends.WaitOne(0))
                {
                    client.socket.BeginSend(data, 0, data.Length, SocketFlags.None, new AsyncCallback(this.OnSend), client);
                }
                else
                {
                    Console.WriteLine("Skipping send to client: " + client.socket.RemoteEndPoint);
                }
            }
            catch (SocketException)
            {
                Console.WriteLine("Disconnected from: " + client.socket.RemoteEndPoint);
                client.socket.Close();
                this.connectedClients.Remove(client);
            }
        }

        public void Broadcast(byte[] data)
        {
            // Send the data to every connected client.
            lock (this.connectedClients.SyncRoot)
            {
                // Copy the array here, since some clients might get dropped during iteration.
                foreach (Client client in this.connectedClients.ToArray())
                {
                    this.Send(client, data);
                }
            }
        }

        public void Close()
        {
            // Close the main listener socket.
            this.listenerSocket.Close();

            // Close each client socket.
            lock (this.connectedClients.SyncRoot)
            {
                foreach (Client client in this.connectedClients)
                {
                    try
                    {
                        client.socket.Shutdown(SocketShutdown.Both);
                        client.socket.Close();
                    }
                    catch (SocketException e)
                    {
                        Console.WriteLine("Error closing socket: " + e);
                    }
                }
            }
        }

        public void Dispose()
        {
            // Dispose of the main listener socket.
            this.listenerSocket.Dispose();

            // Dispose of each client socket.
            lock (this.connectedClients.SyncRoot)
            {
                foreach (Client client in this.connectedClients)
                {
                    client.Dispose();
                }
            }
        }
    }
}
