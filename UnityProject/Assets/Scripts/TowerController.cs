using UnityEngine;

public class TowerController : MonoBehaviour {
    Transform target;
    public Transform turret;

    public ParticleSystem damageParticles;
    public ParticleSystem fireParticles;

    AudioSource audioSource;
    
    public int range;
    public int maxHealth;
    public float maxCooldown;
    public int damage;
    
    int particleThreshold;
    int health;
    float cooldown;

    void Start() {
        audioSource = GetComponent<AudioSource>();
        target = GameObject.FindWithTag("Sparki").transform;
        
        health = maxHealth;
        particleThreshold = maxHealth / 2;

        cooldown = maxCooldown;
    }

    void Update() {
        if (!target) return;
        
        if (Vector3.Distance(transform.position, target.position) <= range) {
            // Hacky way to get the turret to only rotate on its Y axis
            turret.LookAt(target);
            turret.localRotation = Quaternion.Euler(
                0,
                turret.localRotation.eulerAngles.y,
                0
            );

            if (cooldown <= 0) {
                Fire();
                cooldown = maxCooldown;
            }
        }

        if (cooldown > 0) {
            cooldown -= Time.deltaTime;
        }
    }

    void Fire() {
        fireParticles.Play();
        audioSource.Play();

        if (!target) return;
        target.GetComponent<Sparki>().Damage(damage);
    }

    public void Damage(int hp) {
        health -= hp;
        
        if (health < particleThreshold) {
            Debug.Log("Playing!");
            damageParticles.Play();
        }

        if (health <= 0) {
            Destroy(gameObject);
        }
    }
}
