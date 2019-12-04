using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.Protocols;
using RosSharp.RosBridgeClient.MessageTypes.Geometry;
using RosSharp.RosBridgeClient.MessageTypes.Std;

using UnityEngine;

public class ROS { 
    RosSocket socket;

    public ROS(string address) {
        Debug.Log($"Attempting connection to {address}.");
        socket = new RosSocket(new WebSocketNetProtocol(address));
        InitTopics();
    }

    void InitTopics() {
        socket.Subscribe<Pose2D>("/sparki/odometry", UpdateOdometry);
        socket.Subscribe<Int16>("/sparki/set_servo", UpdateServo);
    }

    static void UpdateOdometry(Pose2D pose) {
        Debug.Log($"Odometry update received! ({pose.x}, {pose.y}, {pose.theta})");
    }
    
    static void UpdateServo(Int16 angle) {
        Debug.Log($"Servo update received! ({angle.data})");
    }
}