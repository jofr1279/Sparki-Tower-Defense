using UnityEngine;

public class GridController : MonoBehaviour {
    public GameObject line;
    public int gridSize;
    
    void Start() {
        for (var i = -gridSize; i <= gridSize; i++) {
            var lineGameObject = Instantiate(line, new Vector3(i + 0.5f, 0, 0.5f), Quaternion.identity, transform);
            lineGameObject.GetComponent<LineRenderer>().SetPosition(0, new Vector3(0, 0, -gridSize));
            lineGameObject.GetComponent<LineRenderer>().SetPosition(1, new Vector3(0, 0, gridSize));

            lineGameObject = Instantiate(line, new Vector3(0.5f, 0, i + 0.5f), Quaternion.identity, transform);
            lineGameObject.GetComponent<LineRenderer>().SetPosition(0, new Vector3(-gridSize, 0, 0));
            lineGameObject.GetComponent<LineRenderer>().SetPosition(1, new Vector3(gridSize, 0, 0));
        }
    }
}