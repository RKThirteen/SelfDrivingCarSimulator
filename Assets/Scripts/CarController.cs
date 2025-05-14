using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class CarController : MonoBehaviour
{
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

    void FixedUpdate()
    {
        acceleration = (rb.velocity - lastVelocity) / Time.fixedDeltaTime;
        lastVelocity = rb.velocity;

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
        float wheelRPM = (wheelColliders[2].rpm + wheelColliders[3].rpm) / 2f;
        float normalizedRPM = Mathf.Clamp01(Mathf.Abs(wheelRPM) / maxRPM);
        float torque = throttleInput * maxMotorTorque * torqueCurve.Evaluate(normalizedRPM);

        if (Mathf.Sign(wheelRPM) == Mathf.Sign(throttleInput) || throttleInput == 0)
        {
            wheelColliders[2].motorTorque = torque;
            wheelColliders[3].motorTorque = torque;
        }
        else
        {
            ApplyBrake(maxBrakeTorque * 0.5f);
        }
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