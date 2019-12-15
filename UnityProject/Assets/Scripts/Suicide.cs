using UnityEngine;

public class Suicide : MonoBehaviour {
    public float time;

    private void Start() {
        Destroy(gameObject, time);
    }
}