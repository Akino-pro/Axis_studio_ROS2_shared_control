using System;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

public class communicate_socket : MonoBehaviour
{
    private TcpClient client;
    private NetworkStream stream;
    private float timeSinceLastMessage = 0f; 
    public GameObject object1;
    public GameObject object2;
    public GameObject object3;
    public GameObject object4;
    public GameObject object5;
    public GameObject object6;

    public Vector3 pos1;
    public Vector3 pos2;
    public Vector3 pos3;
    public Vector3 pos4;
    public Vector3 pos5;
    public Vector3 pos6;

    void Start()
    {
        try
        {
            client = new TcpClient("192.168.30.134", 12345); // Server IP and port
            stream = client.GetStream();
            Debug.Log("Connected to server");
        }
        catch (Exception e)
        {
            Debug.LogError($"Socket error: {e.Message}");
        }
    }

    void Update()
    {
        if (client.Connected)
        {
            // Update the timer
            timeSinceLastMessage += Time.deltaTime;

            // Check if 0.1 seconds have passed
            if (timeSinceLastMessage >= 0.1f)
            {
                string message = GetPositions();
                SendMessageToServer(message);
                timeSinceLastMessage = 0f;  // Reset the timer
            }
        }
    }

    string GetPositions()
    {
        pos1 = object1.transform.position;
        pos2 = object2.transform.position;
        pos3 = object3.transform.position;
        pos4 = object4.transform.position;
        pos5 = object5.transform.position;
        pos6 = object6.transform.position;

        return $"Object1: [{pos1.x}, {-pos1.z}, {pos1.y}], " +
            $"Object2: [{pos2.x}, {-pos2.z}, {pos2.y}], " +
            $"Object3: [{pos3.x}, {-pos3.z}, {pos3.y}], " +
            $"Object4: [{pos4.x}, {-pos4.z}, {pos4.y}], " +
            $"Object5: [{pos5.x}, {-pos5.z}, {pos5.y}], " +
            $"Object6: [{pos6.x}, {-pos6.z}, {pos6.y}]";
    }


    void SendMessageToServer(string message)
    {
        byte[] data = Encoding.ASCII.GetBytes(message);
        stream.Write(data, 0, data.Length);
    }

    void OnApplicationQuit()
    {
        stream.Close();
        client.Close();
    }
}
