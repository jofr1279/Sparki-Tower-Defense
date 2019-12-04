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
        Ray ray = arCamera.ScreenPointToRay(Input.GetTouch(0).position);
        RaycastHit hit;
        if (Physics.Raycast(ray, out hit)) {
            if (hit.transform.CompareTag("Raycast Blanket")) {
                Instantiate(tower, hit.point, scaler.rotation, scaler);
            }
        }
        Debug.Log(position);
    }

    void UpdateMoneyText() {
        moneyText.text = "Money: $" + money.ToString();
    }
}