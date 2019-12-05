using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Geometry;
using RosSharp.RosBridgeClient.Protocols;

using UnityEngine;

public class ROS { 
    RosSocket socket;

    public ROS(IProtocol address) {
        socket = new RosSocket(address);
        InitTopics();
    }

    void InitTopics() {
        socket.Subscribe<Pose2D>("/sparki/odometry", UpdateOdometry);
    }

    static void UpdateOdometry(Pose2D pose) {
        Debug.Log($"Odometry update received! ({pose.x}, {pose.y}, {pose.theta})");
    }
}