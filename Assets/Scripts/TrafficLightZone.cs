using UnityEngine;

public class TrafficLightZone : MonoBehaviour
{
    private void OnTriggerEnter(Collider other)
    {
        CarAI car = other.GetComponent<CarAI>();
        if (car != null)
        {
            car.EnterTrafficZone();
        }
    }

    private void OnTriggerExit(Collider other)
    {
        CarAI car = other.GetComponent<CarAI>();
        if (car != null)
        {
            car.ExitTrafficZone();
        }
    }
}