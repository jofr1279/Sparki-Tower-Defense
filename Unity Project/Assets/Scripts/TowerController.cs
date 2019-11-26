using System;
using UnityEngine;

public class TowerController : MonoBehaviour {
    private Transform target;
    public Transform turret;

    public ParticleSystem fireParticles;
    AudioSource _audioSource;
    
    public int range;
    public float maxCooldown;
    float _cooldown;

    void Start() {
        _audioSource = GetComponent<AudioSource>();
        target = GameObject.FindWithTag("Sparki").transform;
        _cooldown = maxCooldown;
    }

    void Update() {
        if (Vector3.Distance(transform.position, target.position) <= range) {
            // Hacky way to get the turret to only rotate on its Y axis
            turret.LookAt(target);
            turret.localRotation = Quaternion.Euler(
                0,
                turret.localRotation.eulerAngles.y,
                0
            );

            if (_cooldown <= 0) {
                Fire();
                _cooldown = maxCooldown;
            }
        }

        if (_cooldown > 0) {
            _cooldown -= Time.deltaTime;
        }
    }

    void Fire() {
        fireParticles.Play();
        _audioSource.Play();
    }
}
