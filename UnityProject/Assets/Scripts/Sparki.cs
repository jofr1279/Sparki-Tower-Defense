using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Sparki : MonoBehaviour {
    public int maxHealth;
    public float maxCooldown;
    
    public ParticleSystem fireParticles;
    public ParticleSystem damageParticles;
    public GameObject explosion;

    int health;
    float cooldown;
    int particleThreshold;

    public int damage;
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
        var p1 = transform.position + transform.up / 4;
        var p2 = Quaternion.AngleAxis(angle, transform.up) * transform.forward;

        Debug.DrawRay(p1, p2, Color.magenta, 4);
        fireParticles.transform.localRotation = Quaternion.AngleAxis(angle, Vector3.up);
        fireParticles.Play();
        
        RaycastHit hit;
        if (Physics.Raycast(p1, p2, out hit)) {
            Debug.Log($"Raycast hit! {hit.transform.name}");
            if (hit.transform.CompareTag("Tower")) {
                Debug.Log("Damaging...");
                hit.transform.GetComponent<TowerController>().Damage(damage);
            }
        }
    }

    public void Damage(int hp) {
        health -= hp;
        
        if (health < particleThreshold) {
            damageParticles.Play();
        }

        if (health <= 0) {
            Instantiate(explosion, transform.position, Quaternion.identity);
            Destroy(gameObject);
        }
    }
}