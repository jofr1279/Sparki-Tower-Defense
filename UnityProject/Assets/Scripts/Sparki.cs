using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Sparki : MonoBehaviour {
    public int maxHealth;
    public float maxCooldown;
    
    public ParticleSystem fireParticles;
    public ParticleSystem damageParticles;

    int health;
    float cooldown;
    int particleThreshold;

    public float damage;
    public int angle;

    void Start() {
        health = maxHealth;
        cooldown = maxCooldown;
        particleThreshold = maxHealth / 2;
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
        var p1 = transform.position + (Vector3.up / 4);
        var p2 = Quaternion.AngleAxis(angle, Vector3.up) * Vector3.forward;

        Debug.DrawRay(p1, p2, Color.magenta, 4);
        fireParticles.transform.localRotation = Quaternion.AngleAxis(angle, Vector3.up);
        fireParticles.Play();
        
        RaycastHit hit;
        if (Physics.Raycast(p1, p2, out hit)) {
            Debug.Log($"Raycast hit! {hit.transform.name}");
            hit.transform.SendMessage("Damage", damage);
        }
    }

    public void Damage(int hp) {
        health -= hp;
        
        if (health < particleThreshold) {
            damageParticles.Play();
        }

        if (health <= 0) {
            Destroy(gameObject);
        }
    }
}