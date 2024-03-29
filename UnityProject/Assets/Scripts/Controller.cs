﻿using System;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Std;
using UnityEngine;
using UnityEngine.UI;

public class Controller : MonoBehaviour {
    public Camera arCamera;
    public GameObject tower;
    public Transform scaler;
    public Text moneyText;
    public Sparki sparki;

    public InputField addressInput;
    
    public int money;
    public int towerCost;

    bool fingerDown;
    public ROS ros;

    private void Start() {
        UpdateMoneyText();

    }

    public void Connect() {
        ros = new ROS(addressInput.text, sparki);
    }

    void Update() {
        if (Input.touchCount >= 1) {
            if (fingerDown) return;
            fingerDown = true;
            
            if (money >= towerCost) {
                BuildTurret(Input.touches[0].position);
                money -= towerCost;
                UpdateMoneyText();
            }
            else {
            }
        }
        else {
            fingerDown = false;
        }
    }
    
    void BuildTurret(Vector2 position) {
        var ray = arCamera.ScreenPointToRay(Input.GetTouch(0).position);
        RaycastHit hit;
        if (Physics.Raycast(ray, out hit)) {
            if (hit.transform.CompareTag("Raycast Blanket")) {
                var local = scaler.transform.InverseTransformPoint(hit.point);
                var gridPoint = new Vector3((float) Math.Round(local.x), 0, (float) Math.Round(local.z));
                var turret = Instantiate(tower, Vector3.zero, scaler.rotation, scaler);
                turret.transform.localPosition = gridPoint;
                
                Debug.Log(local);
                
                ros.AddObstacle(gridPoint, true);
            }
        }
    }

    void UpdateMoneyText() {
        moneyText.text = "Money: $" + money.ToString();
    }
}