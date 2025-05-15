using UnityEngine;

public class TrafficLightZone : MonoBehaviour
{
    public TrafficLight linkedLight; // Assign in Inspector

    private void OnTriggerEnter(Collider other)
    {
        CarAI car = other.GetComponentInParent<CarAI>();
        if (car != null && linkedLight != null)
        {
            car.EnterTrafficZone(linkedLight);
        }
    }

    private void OnTriggerExit(Collider other)
    {
        CarAI car = other.GetComponentInParent<CarAI>();
        if (car != null)
        {
            car.ExitTrafficZone();
        }
    }
}