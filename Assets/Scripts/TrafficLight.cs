using UnityEngine;

public enum LightState { Red, Green }

public class TrafficLight : MonoBehaviour
{
    public LightState currentState = LightState.Red;
    public float switchInterval = 5f;

    private float timer;
    public Renderer lightRenderer;
    public Color redColor = Color.red;
    public Color greenColor = Color.green;

    void Start()
    {
        timer = switchInterval;
        UpdateLightColor();
    }

    void Update()
    {
        timer -= Time.deltaTime;
        if (timer <= 0f)
        {
            Toggle();
            timer = switchInterval;
        }
    }

    void Toggle()
    {
        currentState = currentState == LightState.Red ? LightState.Green : LightState.Red;
        UpdateLightColor();
    }

    void UpdateLightColor()
    {
        if (lightRenderer != null)
        {
            lightRenderer.material.color = currentState == LightState.Red ? redColor : greenColor;
        }
    }
}
