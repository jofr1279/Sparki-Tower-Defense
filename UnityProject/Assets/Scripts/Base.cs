using UnityEngine;

public class Base : MonoBehaviour {
    public ParticleSystem damageParticles;
    public int maxHealth;
    int particleThreshold;
    
    int health;

    void Start() {
        health = maxHealth;
        particleThreshold = maxHealth / 2;
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