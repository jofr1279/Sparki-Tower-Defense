using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Std;
using RosSharp.RosBridgeClient.Protocols;
using UnityEngine;
using Vector3 = UnityEngine.Vector3;

public class ROS { 
    RosSocket socket;
    static Sparki sparki;

    string addObstaclePublisher;
    string removeObstaclePublisher;

    public ROS(string address, Sparki sparki) {
        socket = new RosSocket(new WebSocketNetProtocol(address));
        ROS.sparki = sparki;
        
        InitTopics();
    }

    void InitTopics() {
        var forwardCommandSubscriber = socket.Subscribe<Float32>("/sparki/forward_command", ForwardCommand);
        var turnCommandSubscriber = socket.Subscribe<Float32>("/sparki/turn_command", TurnCommand);
        var turnSetServoSubscriber = socket.Subscribe<Int16>("/sparki/set_servo", SetServo);
        
        addObstaclePublisher = socket.Advertise<String>("/game/add_obstacle");
        removeObstaclePublisher = socket.Advertise<String>("/game/remove_obstacle");
        
        Debug.Log("Topics initialized.");
    }

    static void ForwardCommand(Float32 distance) {
        // Ignore the distance because Sparki never moves more than 1 square.
        
        Debug.Log($"Moving Sparki forward {distance} cm.");
        sparki.needToMove = true;
    }

    static void TurnCommand(Float32 angle) {
        Debug.Log($"Turning Sparki {angle} degrees.");
        sparki.needToTurn = (int)angle.data;
    }
    
    static void SetServo(Int16 angle) {
        Debug.Log($"Setting Sparki's servo to {angle.data} degrees.");
        sparki.angle = angle.data;
    }

    public void AddObstacle(Vector3 position, bool isTarget) {
        var isTargetString = isTarget ? "true" : "false";
        var data = $"{{" +
                   $"    \"position\": {{" +
                   $"        \"x\": {position.x}," +
                   $"        \"y\": {position.z}" +
                   $"    }}," +
                   $"    \"is_target\": {isTargetString}" +
                   $"}}";
        
        Debug.Log(data);
        
        socket.Publish(addObstaclePublisher, new String(data));
    }
    
    public void RemoveObstacle(Vector3 position) {
        var data = $"{{" +
                   $"    \"position\": {{" +
                   $"        \"x\": {position.x}," +
                   $"        \"y\": {position.z}" +
                   $"    }}" +
                   $"}}";
        
        Debug.Log(data);
        
        socket.Publish(removeObstaclePublisher, new String(data));
    } 
}