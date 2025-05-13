using UnityEngine;

public class TrafficLightZone : MonoBehaviour
{
    public TrafficLight trafficLight;

    void OnTriggerEnter(Collider other)
    {
        // Urcăm în ierarhie să căutăm CarAI
        CarAI ai = other.GetComponentInParent<CarAI>();
        if (ai != null && trafficLight != null)
        {
            ai.EnterTrafficZone(trafficLight);
        }
    }

    void OnTriggerExit(Collider other)
    {
        CarAI ai = other.GetComponentInParent<CarAI>();
        if (ai != null)
        {
            ai.ExitTrafficZone();
        }
    }
}
