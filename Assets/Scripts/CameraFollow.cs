using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    public Transform target;

    [Header("Positioning")]
    public float height = 5f;
    public float distance = 10f;
    public float followSpeed = 5f;
    public float rotationDamping = 3f;

    [Header("Zoom & Orbit Controls")]
    public float zoomSpeed = 5f;
    public float minDistance = 5f;
    public float maxDistance = 20f;
    public float sideOffset = 0f; // can be adjusted with A/D
    public float sideAdjustSpeed = 5f;

    void LateUpdate()
    {
        if (target == null) return;

        // Update distance with W/S
        if (Input.GetKey(KeyCode.W))
            distance -= zoomSpeed * Time.deltaTime;
        if (Input.GetKey(KeyCode.S))
            distance += zoomSpeed * Time.deltaTime;
        distance = Mathf.Clamp(distance, minDistance, maxDistance);

        // Update side offset with A/D
        if (Input.GetKey(KeyCode.A))
            sideOffset -= sideAdjustSpeed * Time.deltaTime;
        if (Input.GetKey(KeyCode.D))
            sideOffset += sideAdjustSpeed * Time.deltaTime;

        // Desired rotation behind the car
        Quaternion desiredRotation = Quaternion.Euler(0, target.eulerAngles.y, 0);
        Vector3 behindPosition = target.position - (desiredRotation * Vector3.forward * distance)
                                               + (desiredRotation * Vector3.right * sideOffset)
                                               + Vector3.up * height;

        // Smooth camera position and rotation
        transform.position = Vector3.Lerp(transform.position, behindPosition, followSpeed * Time.deltaTime);
        transform.rotation = Quaternion.Slerp(transform.rotation, Quaternion.LookRotation(target.position - transform.position), rotationDamping * Time.deltaTime);
    }
}
