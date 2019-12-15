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

    public static int[] xChange = {0, 1, 0, -1};
    public static int[] yChange = {1, 0, -1, 0};

    public bool needToMove = false;
    public int needToTurn = 0;
    
    // 0 = N
    // 1 = E
    // 2 = S
    // 3 = W
    public int direction;

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

        if (needToMove) {
            MoveForward();
            needToMove = false;
        }

        if (needToTurn != 0) {
            Turn();
            needToTurn = 0;
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
                hit.transform.GetComponent<TowerController>().Damage(damage);
            }
            if (hit.transform.CompareTag("Base")) {
                hit.transform.GetComponent<Base>().Damage(damage);
            }
        }
    }

    void MoveForward() {
        var sparkiTransform = transform;
        var localPosition = sparkiTransform.localPosition;
        sparkiTransform.localPosition = new Vector3(
            localPosition.x + xChange[direction],
            0,
            localPosition.z + yChange[direction]  
        );
    }

    void Turn() {
        if (needToTurn > 0) {
            switch (direction) {
                case 0:
                    direction = 1;
                    break;
                case 1:
                    direction = 2;
                    break;
                case 2:
                    direction = 3;
                    break;
                case 3:
                    direction = 0;
                    break;
            }
        }
        else {
            switch (direction) {
                case 0:
                    direction = 3;
                    break;
                case 1:
                    direction = 0;
                    break;
                case 2:
                    direction = 1;
                    break;
                case 3:
                    direction = 2;
                    break;
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