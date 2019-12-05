using System;
using UnityEngine;
using UnityEngine.UI;

public class Controller : MonoBehaviour {
    public Camera arCamera;
    public GameObject tower;
    public Transform scaler;
    public Text moneyText;
    public UIController uiController;
    
    public int money;
    public int towerCost;

    private bool fingerDown;
    private ROS ros;

    private void Start() {
        UpdateMoneyText();
    }

    void Update() {
        if (Input.touchCount >= 1) {
            if (fingerDown) return;
            fingerDown = true;
            
            if (money >= towerCost) {
                Debug.Log("Building...");
                BuildTurret(Input.touches[0].position);
                money -= towerCost;
                UpdateMoneyText();
            }
            else {
                Debug.Log("Insufficient funds.");
            }
        }
        else {
            fingerDown = false;
        }
    }
    
    void Connect(string address) {
        ros = new ROS(address);
    }

    void BuildTurret(Vector2 position) {
        var ray = arCamera.ScreenPointToRay(Input.GetTouch(0).position);
        RaycastHit hit;
        if (Physics.Raycast(ray, out hit)) {
            if (hit.transform.CompareTag("Raycast Blanket")) {
                var local = scaler.transform.InverseTransformPoint(hit.point);
                var gridPoint = new Vector3((float) Math.Round(local.x), 0, (float) Math.Round(local.z));
                Debug.Log($"{hit.point} -> {local} -> {gridPoint}");
                var turret = Instantiate(tower, Vector3.zero, scaler.rotation, scaler);
                turret.transform.localPosition = gridPoint;
                
            }
        }
    }

    void UpdateMoneyText() {
        moneyText.text = "Money: $" + money.ToString();
    }
}