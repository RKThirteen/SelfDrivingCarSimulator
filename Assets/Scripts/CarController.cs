using System.Collections.Generic;
using System.Collections;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class CarController : MonoBehaviour
{
    [System.Serializable]
    public class Gear
    {
        public string name;
        public float minSpeed;
        public float maxSpeed;
        public float torqueMultiplier;
        public bool isReverse;
    }
    [Header("Transmission")]
    public List<Gear> gears = new List<Gear>();
    public int currentGearIndex = 0;
    public float shiftDelay = 1f;
    public bool isShifting = false;
    public float rpm;
    [Header("Chassis & Wheels")]
    public Rigidbody rb;
    public WheelCollider[] wheelColliders;
    public Transform[] wheelMeshes;

    [Header("Engine")]
    public float maxMotorTorque = 1500f;
    public float maxBrakeTorque = 3000f;
    public AnimationCurve torqueCurve;
    public float maxRPM = 6000f;

    [Header("Steering")]
    public float maxSteerAngle = 30f;
    [Range(0, 0.5f)] public float ackermannFactor = 0.25f;

    [Header("Suspension")]
    public float suspensionSpring = 35000f;
    public float suspensionDamper = 4500f;
    public float suspensionTargetPos = 0.5f;
    public float forwardStiffness = 1.5f;
    public float sidewaysStiffness = 1.8f;

    [Header("Aerodynamics")]
    public float dragCoefficient = 0.3f;
    public float frontalArea = 2.2f;
    public float airDensity = 1.225f;
    public float downforceCoefficient = 0.1f;

    private float throttleInput;
    private float steerInput;
    private Vector3 lastVelocity;
    private Vector3 acceleration;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        if (torqueCurve == null || torqueCurve.length == 0)
        {
            torqueCurve = new AnimationCurve(
                new Keyframe(0f, 1f),   // cuplu maxim la RPM mic
                new Keyframe(0.5f, 1f), // constant în zona medie
                new Keyframe(1f, 0.85f)  // scade puțin la maxim RPM
            );
        }
        rb.centerOfMass = new Vector3(0, -0.9f, 0);

        foreach (WheelCollider wheel in wheelColliders)
        {
            JointSpring spring = wheel.suspensionSpring;
            spring.spring = suspensionSpring;
            spring.damper = suspensionDamper;
            spring.targetPosition = suspensionTargetPos;
            wheel.suspensionSpring = spring;

            WheelFrictionCurve forwardFriction = wheel.forwardFriction;
            forwardFriction.stiffness = forwardStiffness;
            wheel.forwardFriction = forwardFriction;

            WheelFrictionCurve sidewaysFriction = wheel.sidewaysFriction;
            sidewaysFriction.stiffness = sidewaysStiffness;
            wheel.sidewaysFriction = sidewaysFriction;
        }
    }

    public IEnumerator GearShiftDelay(int newGearIndex)
    {
        if (newGearIndex == currentGearIndex || isShifting) yield break;

        isShifting = true;
        yield return new WaitForSeconds(shiftDelay);
        currentGearIndex = newGearIndex;
        isShifting = false;
    }

    void HandleManualGearInput()
    {
        if (isShifting) return;

        if (Input.GetKeyDown(KeyCode.E))
            ShiftUp();
        else if (Input.GetKeyDown(KeyCode.Q))
            ShiftDown();
        //Debug.Log($"Current Gear: {gears[currentGearIndex].name}, RPM: {rpm}");
    }

    void ShiftUp()
    {
        if (currentGearIndex < gears.Count - 1)
        {
            currentGearIndex++;
            StartCoroutine(GearShiftDelay());
        }
    }

    void ShiftDown()
    {
        if (currentGearIndex > 0)
        {
            currentGearIndex--;
            StartCoroutine(GearShiftDelay());
        }
    }

    IEnumerator GearShiftDelay()
    {
        isShifting = true;
        yield return new WaitForSeconds(shiftDelay);
        isShifting = false;
    }


    void UpdateRPM()
    {
        float avgWheelRPM = (wheelColliders[2].rpm + wheelColliders[3].rpm) / 2f;
        rpm = Mathf.Lerp(1000f, maxRPM, Mathf.Abs(avgWheelRPM) / maxRPM);
    }


    void Update()
    {
        HandleManualGearInput();
        UpdateRPM();
    }


    void FixedUpdate()
    {
        
        acceleration = (rb.velocity - lastVelocity) / Time.fixedDeltaTime;
        lastVelocity = rb.velocity;

        bool groundedRearLeft = wheelColliders[2].GetGroundHit(out _);
        bool groundedRearRight = wheelColliders[3].GetGroundHit(out _);
        //Debug.Log($"Rear wheels grounded: {groundedRearLeft && groundedRearRight}");

        ApplyEngine();
        ApplyBrake();
        ApplySteering();
        ApplyAerodynamics();
        ApplyWeightTransfer();
        UpdateWheelMeshes();

    }

    public void SetThrottle(float t) => throttleInput = Mathf.Clamp(t, -1f, 1f);
    public void SetSteer(float s) => steerInput = Mathf.Clamp(s, -1f, 1f);
    public void SetBrake(float brakeInput) => ApplyBrake(brakeInput);
    public Vector3 GetAcceleration() => acceleration;

    private void ApplyEngine()
    {
        if (isShifting) return;

        Gear gear = gears[currentGearIndex];

        float direction = gear.isReverse ? -1f : 1f;

        float avgWheelRPM = (wheelColliders[2].rpm + wheelColliders[3].rpm) / 2f;
        float normalizedRPM = Mathf.Clamp01(Mathf.Abs(avgWheelRPM) / maxRPM);
        float baseTorque = maxMotorTorque * torqueCurve.Evaluate(normalizedRPM) * gear.torqueMultiplier;

        // Use throttle sign for direction
        float torque = throttleInput * baseTorque;
        // Fix: allow torque if nearly stopped OR wheels spinning the right direction
        bool isNearStop = rb.velocity.magnitude < 0.5f;
        bool spinningCorrectly = Mathf.Sign(Vector3.Dot(rb.velocity, transform.forward)) == direction;
        bool allowTorque;
        if (!gear.isReverse) {
            // in Drive, as soon as throttle>0 we push forward
            allowTorque = throttleInput > 0f;
        } else {
            // your existing reverse-slip logic
            allowTorque = spinningCorrectly || (isNearStop && Mathf.Abs(avgWheelRPM) < 5f);
        }

        if (allowTorque)
        {
            wheelColliders[2].motorTorque = torque * direction;
            wheelColliders[3].motorTorque = torque * direction;
        }
        else
        {
            wheelColliders[2].motorTorque = 0f;
            wheelColliders[3].motorTorque = 0f;
        }
        //Debug.Log($"Gear: {gear.name}, RPM: {avgWheelRPM}, Velocity: {rb.velocity.magnitude}, AllowTorque: {allowTorque}");
    }

    private void ApplyBrake(float brakeInput = 0f)
    {
        float engineBrake = (Mathf.Abs(throttleInput) < 0.1f) ?
            maxBrakeTorque * 0.1f * Mathf.Clamp01(rb.velocity.magnitude / 5f) :
            0f;

        float userBrake = Mathf.Clamp01(-throttleInput) * maxBrakeTorque;

        foreach (WheelCollider wheel in wheelColliders)
        {
            wheel.brakeTorque = Mathf.Clamp(engineBrake + userBrake + brakeInput, 0, maxBrakeTorque);
        }
    }

    private void ApplySteering()
    {
        float steerAngle = steerInput * maxSteerAngle;
        float innerAngle = steerAngle / (1 + (ackermannFactor * steerAngle / maxSteerAngle));
        float outerAngle = steerAngle / (1 - (ackermannFactor * steerAngle / maxSteerAngle));

        wheelColliders[0].steerAngle = innerAngle;
        wheelColliders[1].steerAngle = outerAngle;
    }

    private void ApplyAerodynamics()
    {
        Vector3 velocity = rb.velocity;
        float speedSquared = velocity.sqrMagnitude;

        Vector3 drag = -0.5f * airDensity * dragCoefficient * frontalArea * speedSquared * velocity.normalized;
        Vector3 downforce = -transform.up * (speedSquared * downforceCoefficient);

        rb.AddForce(drag + downforce, ForceMode.Force);
    }

    private void ApplyWeightTransfer()
    {
        Vector3 lateralAccel = transform.InverseTransformDirection(acceleration).x * transform.right;
        float transferForce = rb.mass * lateralAccel.magnitude * 0.3f;
        rb.AddForce(-transferForce * lateralAccel.normalized, ForceMode.Force);
    }

    private void UpdateWheelMeshes()
    {
        if (wheelColliders == null || wheelMeshes == null) return;

        for (int i = 0; i < wheelColliders.Length; i++)
        {
            if (wheelColliders[i] == null || wheelMeshes[i] == null) continue;

            // Obține poziția și rotația actuală de la Wheel Collider
            wheelColliders[i].GetWorldPose(out Vector3 pos, out Quaternion rot);

            // Actualizează Wheel Mesh
            wheelMeshes[i].position = pos;
            wheelMeshes[i].rotation = rot;
        }
    }
}