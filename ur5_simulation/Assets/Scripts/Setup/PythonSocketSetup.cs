using UnityEngine;
using System;
using System.Net.Sockets;
using System.Text;
using System.Net;
using System.IO;

[DefaultExecutionOrder(100)]
public class PythonSocketSetup : MonoBehaviour
{
    private TcpListener listener;
    private TcpClient client;
    private NetworkStream stream;
    private byte[] receiveBuffer = new byte[1024];

    //for socket connection
    private static string unityAssetsFilePath = "ur5_simulation/Assets";
    private string envFilePath = Application.dataPath.Replace(unityAssetsFilePath, "");
    private string hostAddress;
    private int portNumber;

    void Start()
    {
        ReadEnv();
        try
        {
            // Start listening for connections
            listener = new TcpListener(IPAddress.Parse(hostAddress), portNumber);
            listener.Start();
            Debug.Log("Unity server started, waiting for Python connection...");

            // Accept connection from Python
            client = listener.AcceptTcpClient();
            stream = client.GetStream();
            Debug.Log("Python connected!");

            // Start receiving data
            stream.BeginRead(receiveBuffer, 0, receiveBuffer.Length, ReceiveCallback, null);
        }
        catch (Exception e)
        {
            Debug.LogError("Socket error: " + e.Message);
        }
    }

    private void ReadEnv()
    {
        Debug.Log(Application.dataPath);
        string envFile = Directory.GetFiles(envFilePath, "*.env")[0];
        foreach (var line in File.ReadAllLines(envFile))
        {
            // Skip empty lines and comments
            if (string.IsNullOrWhiteSpace(line) || line.StartsWith("#"))
            {
                continue;
            }

            var parts = line.Split('=', 2); // Split only on the first '='
            if (parts.Length == 2)
            {
                var key = parts[0].Trim();
                var value = parts[1].Trim().Trim('"'); // Remove potential quotes

                // Set as environment variable
                Environment.SetEnvironmentVariable(key, value);
            }
        }

        hostAddress = Environment.GetEnvironmentVariable("HOST");
        portNumber = int.Parse(Environment.GetEnvironmentVariable("PORT"));

        Debug.Log($"{hostAddress} | {portNumber}");
    }

    private void ReceiveCallback(IAsyncResult AR)
    {
        try
        {
            int bytesRead = stream.EndRead(AR);
            if (bytesRead > 0)
            {
                string receivedData = Encoding.ASCII.GetString(receiveBuffer, 0, bytesRead);
                Debug.Log("Received from Python: " + receivedData);

                // Send response back to Python
                string response = "Hello back from Unity!";
                byte[] responseData = Encoding.ASCII.GetBytes(response);
                stream.Write(responseData, 0, responseData.Length);
                Debug.Log("Sent to Python: " + response);

                // Continue listening for more data
                stream.BeginRead(receiveBuffer, 0, receiveBuffer.Length, ReceiveCallback, null);
            }
        }
        catch (Exception e)
        {
            Debug.LogError("Receive callback error: " + e.Message);
        }
    }

    void OnApplicationQuit()
    {
        if (stream != null) stream.Close();
        if (client != null) client.Close();
        if (listener != null) listener.Stop();
    }
}
