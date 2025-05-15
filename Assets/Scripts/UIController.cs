using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;
public class UIController : MonoBehaviour
{
    [Header("References")]
    public Rigidbody carRb;
    public CarController carController;
    public CarAI carAI;

    [Header("UI Elements")]
    [SerializeField] private TMP_Text speedText;
    [SerializeField] private TMP_Text gearText;
    [SerializeField] private TMP_Text cameraHelpText;
    [SerializeField] private TMP_Text trafficLightText;
    [SerializeField] private TMP_Text stateText;
    [SerializeField] private TMP_Text rpmText;

    [Header("Options")]
    public bool showHelpText = true;

    void Start()
    {
        if (cameraHelpText != null && showHelpText)
        {
            cameraHelpText.text = "Camera Controls:\nW/S = Zoom In/Out\nA/D = Orbit";
        }
        else if (cameraHelpText != null)
        {
            cameraHelpText.text = "";
        }
    }

    void Update()
    {
        if (carRb != null)
            UpdateSpeed(carRb.velocity.magnitude);

        if (carController != null)
        {
            UpdateGear(carController.gears[carController.currentGearIndex].name,
                       carController.gears[carController.currentGearIndex].isReverse);

            UpdateRPM(carController.rpm);
        }

        if (carAI != null)
        {
            UpdateCarState(carAI.currentState);

            if (carAI.inTrafficZone && carAI.currentTrafficLight != null)
            {
                UpdateTrafficLightState(carAI.currentTrafficLight.currentState);
            }
            else
            {
                ClearTrafficLightState();
            }
        }
    }

    public void UpdateSpeed(float speed)
    {
        if (speedText != null)
            speedText.text = "Speed: " + Mathf.RoundToInt(speed * 3.6f) + " km/h"; // m/s to km/h
    }

    public void UpdateGear(string gearName, bool isReverse)
    {
        if (gearText != null)
        {
            gearText.text = "Gear: " + gearName;
            gearText.color = isReverse ? Color.red : Color.white;
        }
    }

    public void UpdateRPM(float rpm)
    {
        if (rpmText != null)
            rpmText.text = "RPM: " + Mathf.RoundToInt(rpm);
    }

    public void UpdateCarState(CarAI.CarState state)
    {
        if (stateText != null)
            stateText.text = "State: " + state.ToString();
    }

    public void UpdateTrafficLightState(LightState state)
    {
        if (trafficLightText != null)
        {
            trafficLightText.text = "Traffic Light: " + state;
            trafficLightText.color = state == LightState.Red ? Color.red : (state == LightState.Yellow ? Color.yellow : Color.green);
        }
    }

    public void ClearTrafficLightState()
    {
        if (trafficLightText != null)
        {
            trafficLightText.text = "";
        }
    }
}
