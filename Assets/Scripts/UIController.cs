using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;
public class UIController : MonoBehaviour
{
    [SerializeField] private TMP_Text speedText;
    [SerializeField] private Rigidbody targetRigidbody;

    void Update()
    {
        if (targetRigidbody != null && speedText != null)
        {
            float speed = targetRigidbody.velocity.magnitude;
            speedText.text = $"Speed: {speed:F1} m/s";
        }
    }
}
