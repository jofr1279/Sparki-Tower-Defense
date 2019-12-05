using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Sparki : MonoBehaviour {
    public int health;
    public float maxCooldown;
    
    float cooldown;

    void Start() {
        cooldown = maxCooldown;
    }
    
    void Update() {
        if (cooldown > 0) {
            cooldown -= Time.deltaTime;
        }
        else {
            Fire();
            cooldown = maxCooldown;
        }
    }

    void Fire() {
        
    }
}