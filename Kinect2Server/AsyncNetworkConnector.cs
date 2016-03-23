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
using System.Text;
using System.Threading;

namespace PersonalRobotics.Kinect2Server
{
    /// <summary>
    /// Contains bookkeeping information for a TCP socket client.
    /// </summary>
    public sealed class Client : IDisposable
    {
        public const int BufferSize = 32;
        public readonly byte[] buffer = new byte[BufferSize];
        public readonly EndPoint endpoint;
        public readonly Socket socket;
        public readonly Semaphore sends;
        public readonly StringBuilder stringBuffer = new StringBuilder();

        public Client(Socket socket, int concurrentSends)
        {
            this.socket = socket;
            this.endpoint = socket.RemoteEndPoint;
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
                var client = new Client(socket, this.concurrentSends);

                socket.BeginReceive(client.buffer, 0, Client.BufferSize, 0,
                    new AsyncCallback(OnReceive), client);
                connector.connectedClients.Add(client);
                connector.listenerSocket.BeginAccept(new AsyncCallback(OnAccept), connector);

                // TODO: put this in the event log instead.
                Console.WriteLine("Connected to: " + client.endpoint);
            }
            catch (SocketException e)
            {
                Console.WriteLine("Error accepting client: " + e);
            }
        }

        public static void OnReceive(IAsyncResult ar)
        {
            String content = String.Empty;
            Client client = (Client)ar.AsyncState;

            // Read data from the client socket. 
            if (!client.socket.Connected)
                return;

            // Receive new data from the socket.
            int bytesRead = client.socket.EndReceive(ar);
            if (bytesRead > 0)
            {
                // Store the data received so far.
                client.stringBuffer.Append(Encoding.ASCII.GetString(client.buffer, 0, bytesRead));

                // Check for a newline.
                content = client.stringBuffer.ToString();
                var newlineIdx = content.IndexOf("\n");
                if (newlineIdx >= 0)
                {
                    // Put the remaining buffer back in the StringBuilder.
                    client.stringBuffer.Clear();
                    client.stringBuffer.Append(content.Substring(newlineIdx + 1));

                    // If we receive the word "OK", reset one of the send tokens.
                    if (content.Substring(0, newlineIdx).IndexOf("OK") >= 0)
                        client.sends.Release();
                }
            }

            // Queue the next line.
            try
            {
                client.socket.BeginReceive(client.buffer, 0, Client.BufferSize, 0,
                    new AsyncCallback(OnReceive), client);
            }
            catch (SocketException)
            {
                // Do nothing.
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
                }
            }
            catch (SocketException)
            {
                Console.WriteLine("Disconnected from: " + client.endpoint);
                client.socket.Close();
                this.connectedClients.Remove(client);
            }
        }

        public void Send(Client client, byte[] data)
        {
            try
            {
                if (client.sends.WaitOne(0))
                    client.socket.BeginSend(data, 0, data.Length, SocketFlags.None,
                        new AsyncCallback(this.OnSend), client);
            }
            catch (SocketException)
            {
                Console.WriteLine("Disconnected from: " + client.endpoint);
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
