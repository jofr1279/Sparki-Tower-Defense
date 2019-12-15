using UnityEngine;

public class Base : MonoBehaviour {
    public ParticleSystem damageParticles;
    public GameObject explosion;

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
            Instantiate(explosion, transform.position, Quaternion.identity);
            Destroy(gameObject);
        }
    }
}