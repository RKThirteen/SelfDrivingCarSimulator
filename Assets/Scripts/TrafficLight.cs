using UnityEngine;

public enum LightState { Red, Yellow, Green }

public class TrafficLight : MonoBehaviour
{
    public LightState currentState = LightState.Red;
    public float redTime = 5f;
    public float yellowTime = 2f;
    public float greenTime = 5f;

    private float timer;
    public Renderer lightRenderer;
    public Color redColor = Color.red;
    public Color yellowColor = Color.yellow;
    public Color greenColor = Color.green;

    void Start()
    {
        timer = redTime;
        UpdateLightColor();
    }

    void Update()
    {
        timer -= Time.deltaTime;
        if (timer <= 0f)
        {
            SwitchState();
        }
    }

    void SwitchState()
    {
        if (currentState == LightState.Red)
        {
            currentState = LightState.Green;
            timer = greenTime;
        }
        else if (currentState == LightState.Green)
        {
            currentState = LightState.Yellow;
            timer = yellowTime;
        }
        else // Yellow
        {
            currentState = LightState.Red;
            timer = redTime;
        }

        UpdateLightColor();
    }

    void UpdateLightColor()
    {
        if (lightRenderer != null)
        {
            switch (currentState)
            {
                case LightState.Red: lightRenderer.material.color = redColor; break;
                case LightState.Yellow: lightRenderer.material.color = yellowColor; break;
                case LightState.Green: lightRenderer.material.color = greenColor; break;
            }
        }
    }
}

