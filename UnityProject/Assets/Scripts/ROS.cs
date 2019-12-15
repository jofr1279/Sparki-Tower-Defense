using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Geometry;
using RosSharp.RosBridgeClient.MessageTypes.Std;

using UnityEngine;
using Vector3 = UnityEngine.Vector3;

public class ROS { 
    RosSocket socket;

    string addObstaclePublisher;
    string removeObstaclePublisher;

    public ROS(RosConnector connector) {
        socket = connector.RosSocket;
        
        InitTopics();
    }

    void InitTopics() {
        socket.Subscribe<Pose2D>("/sparki/odometry", UpdateOdometry);
        socket.Subscribe<Int16>("/sparki/set_servo", UpdateServo);
        addObstaclePublisher = socket.Advertise<String>("/game/add_obstacle");
        removeObstaclePublisher = socket.Advertise<String>("/game/remove_obstacle");
        Debug.Log("Topics initialized!");
    }

    static void UpdateOdometry(Pose2D pose) {
        Debug.Log($"Odometry update received! ({pose.x}, {pose.y}, {pose.theta})");
    }
    
    static void UpdateServo(Int16 angle) {
        Debug.Log($"Servo update received! ({angle.data})");
    }

    public void AddObstacle(Vector3 position, bool isTarget) {
        var data = $"{{" +
                   $"    \"position\": {{" +
                   $"        \"x:\" {position.x}," +
                   $"        \"y:\" {position.z}" +
                   $"    }}," +
                   $"    \"is_target\": {isTarget}" +
                   $"}}";
        
        Debug.Log(data);
        
        socket.Publish(addObstaclePublisher, new String(data));
    }
    
    public void RemoveObstacle(Vector3 position) {
        var data = $"{{" +
                   $"    \"position\": {{" +
                   $"        \"x:\" {position.x}," +
                   $"        \"y:\" {position.z}" +
                   $"    }}" +
                   $"}}";
        
        Debug.Log(data);
        
        socket.Publish(removeObstaclePublisher, new String(data));
    } 
}