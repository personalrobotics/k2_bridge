using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Threading;

namespace PersonalRobotics.Kinect2Server
{
    /// <summary>
    /// Contains bookkeeping information for a TCP socket client.
    /// </summary>
    public class Client : IDisposable
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
    public class AsyncNetworkConnector : IDisposable
    {
        public SynchronizedCollection<Client> connectedClients =
            new SynchronizedCollection<Client>();
        
        public readonly int port;
        protected readonly Socket listenerSocket;

        public AsyncNetworkConnector(int port, int concurrentSends = 34)
        {
            this.listenerSocket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
            this.port = port;
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
                connector.connectedClients.Add(new Client(socket, 3));
                Console.WriteLine("Added socket to client list"); // TODO: put this in the event log instead.
                connector.listenerSocket.BeginAccept(new AsyncCallback(OnAccept), connector);
            }
            catch (Exception e)
            {
                Console.WriteLine("Error in startListeningCallBack: " + e.ToString());
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
            catch (Exception e)
            {
                Console.WriteLine("Error in sendCallBack: " + e.ToString());
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
                    client.socket.BeginSend(data, 0, data.Length, 0, new AsyncCallback(this.OnSend), client);
                }
                else
                {
                    Console.WriteLine("Skipping send.");
                }

            }
            catch (Exception e)
            {
                Console.WriteLine("Error in send: " + e.ToString());
                client.socket.Close();
                this.connectedClients.Remove(client);
            }
        }

        public void Broadcast(byte[] data)
        {
            try
            {
                foreach (Client client in this.connectedClients)
                {
                    this.Send(client, data);
                }
            }
            catch
            {

            }
        }

        public void Close()
        {
            // Close the main listener socket.
            this.listenerSocket.Close();

            // Close each client socket.
            foreach (Client client in this.connectedClients)
            {
                try
                {
                    client.socket.Shutdown(SocketShutdown.Both);
                    client.socket.Close();
                }
                catch (Exception e)
                {
                    Console.WriteLine("Error in closing: " + e.ToString());
                }
            }
        }

        public void Dispose()
        {
            this.listenerSocket.Dispose();
            foreach (Client client in this.connectedClients)
            {
                client.Dispose();
            }
        }
    }
}
