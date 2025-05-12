using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class CarController : MonoBehaviour
{
    public float acceleration = 500f;
    public float steering = 30f;
    private Rigidbody rb;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.centerOfMass = new Vector3(0, -0.9f, 0);
    }

    void FixedUpdate()
    {
        // un demo simplu: mașina înainte continuu
        rb.AddForce(transform.forward * acceleration * Time.fixedDeltaTime);
    }
}

