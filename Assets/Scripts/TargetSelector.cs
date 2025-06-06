using System.Collections.Generic;
using UnityEngine;

public class TargetSelector : MonoBehaviour
{
    public List<Transform> targets;
    public int currentIndex = 0;

    public Transform CurrentTarget => targets.Count > 0 ? targets[currentIndex] : null;

    public CarAI carAI;

    public void SetCarAIReference(CarAI ai)
    {
        carAI = ai;
        if (carAI != null && CurrentTarget != null)
        {
            carAI.TransitionToState(CarAI.CarState.Pathfind);
        }
    }

    public void NextTarget()
    {
        if (targets.Count == 0) return;

        currentIndex = (currentIndex + 1) % targets.Count;
        Debug.Log("Target switched to: " + CurrentTarget.name);

        if (carAI != null)
        {
            carAI.TransitionToState(CarAI.CarState.Pathfind);
        }
    }
}
