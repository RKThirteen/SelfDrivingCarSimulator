using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(CarController))]
public class CarAI : MonoBehaviour
{
    public enum CarState { Idle, Pathfind, Drive, AvoidObstacle, Stop }
    public CarState currentState = CarState.Pathfind;

    [Header("Core References")]
    public GridManager gridManager;
    public TargetSelector targetSelector;
    private CarController controller;
    private Rigidbody rb;

    [Header("Pathfinding")]
    public float waypointRadius = 2f;
    public float recalculateDistance = 5f;
    private List<Transform> waypoints = new List<Transform>();
    private int currentWaypoint = 0;

    [Header("AI Parameters")]
    public float desiredSpeed = 12f;
    public float maxSteerAngle = 25f;
    public float steerResponsiveness = 0.8f;
    public float speedAnticipation = 1.5f;
    public float brakeDistanceMultiplier = 0.8f;

    [Header("Obstacle Avoidance")]
    public float[] rayAngles = { -30f, 0f, 30f };
    public float rayRange = 8f;
    public LayerMask obstacleMask;
    public float avoidanceForce = 2f;
    private Vector3 avoidanceVector;

    void Awake()
    {
        controller = GetComponent<CarController>();
        rb = GetComponent<Rigidbody>();
        InitializeReferences();
    }

    void InitializeReferences()
    {
        if (gridManager == null)
            gridManager = FindObjectOfType<GridManager>();

        if (targetSelector == null)
            targetSelector = FindObjectOfType<TargetSelector>();
    }

    void Update()
    {
        switch (currentState)
        {
            case CarState.Idle: UpdateIdle(); break;
            case CarState.Pathfind: UpdatePathfinding(); break;
            case CarState.Drive: UpdateDriving(); break;
            case CarState.AvoidObstacle: UpdateAvoidance(); break;
            case CarState.Stop: UpdateStop(); break;
        }
    }

    void UpdateIdle()
    {
        controller.SetThrottle(0f);
        controller.SetSteer(0f);

        if (targetSelector.CurrentTarget != null)
            TransitionToState(CarState.Pathfind);
    }

    void UpdatePathfinding()
    {
        CalculatePath();
    }

    void UpdateDriving()
    {
        if (waypoints.Count == 0 || currentWaypoint >= waypoints.Count)
        {
            TransitionToState(CarState.Pathfind);
            return;
        }

        Vector3 targetPos = waypoints[currentWaypoint].position;
        Vector3 localTarget = transform.InverseTransformPoint(targetPos);

        float steerDirection = localTarget.x / localTarget.magnitude;
        float speedFactor = Mathf.Clamp01(rb.velocity.magnitude / desiredSpeed);
        float adjustedSteer = steerDirection * maxSteerAngle * steerResponsiveness * (1 - speedFactor * 0.4f);

        float distanceToWaypoint = Vector3.Distance(transform.position, targetPos);
        float anticipatedSpeed = rb.velocity.magnitude + (controller.GetAcceleration().magnitude * speedAnticipation);
        float throttleInput = (anticipatedSpeed < desiredSpeed) ? 1f : 0f;

        float brakeInput = 0f;
        if (distanceToWaypoint < (desiredSpeed * brakeDistanceMultiplier))
            brakeInput = Mathf.Clamp01(1 - (distanceToWaypoint / (desiredSpeed * brakeDistanceMultiplier)));

        controller.SetSteer(Mathf.Clamp(adjustedSteer / maxSteerAngle, -1f, 1f));
        controller.SetThrottle(throttleInput * (1 - brakeInput));
        controller.SetBrake(brakeInput);

        if (distanceToWaypoint < waypointRadius)
            currentWaypoint++;

        if (DetectObstacle())
            TransitionToState(CarState.AvoidObstacle);
    }

    void UpdateAvoidance()
    {
        Vector3 avoidanceSteer = CalculateAvoidance();
        controller.SetSteer(avoidanceSteer.x);
        controller.SetThrottle(0.7f);

        if (!DetectObstacle())
            TransitionToState(CarState.Drive);
    }

    void UpdateStop()
    {
        controller.SetThrottle(0f);
        controller.SetBrake(1f);

        if (CheckClearPath())
            TransitionToState(CarState.Drive);
    }

    bool DetectObstacle()
    {
        foreach (float angle in rayAngles)
        {
            Quaternion rotation = Quaternion.AngleAxis(angle, transform.up);
            if (Physics.Raycast(transform.position + Vector3.up * 0.5f, rotation * transform.forward, rayRange, obstacleMask))
                return true;
        }
        return false;
    }

    Vector3 CalculateAvoidance()
    {
        Vector3 totalForce = Vector3.zero;

        foreach (float angle in rayAngles)
        {
            Quaternion rotation = Quaternion.AngleAxis(angle, transform.up);
            Ray ray = new Ray(transform.position + Vector3.up * 0.5f, rotation * transform.forward);

            if (Physics.Raycast(ray, out RaycastHit hit, rayRange, obstacleMask))
                totalForce += rotation * Vector3.right * avoidanceForce * (1 - (hit.distance / rayRange));
        }

        return transform.InverseTransformDirection(totalForce.normalized);
    }
    // Adaugă aceste metode în CarAI.cs
    private void RetracePath(Node startNode, Node endNode)
    {
        List<Node> path = new List<Node>();
        Node currentNode = endNode;

        while (currentNode != startNode)
        {
            path.Add(currentNode);
            currentNode = currentNode.parent;
            if (currentNode == null) // Prevenire buclă infinită
            {
                Debug.LogError("Path retrace failed - null parent!");
                return;
            }
        }
        path.Reverse();

        // Distruge waypoint-urile vechi
        foreach (Transform wp in waypoints)
        {
            if (wp != null) Destroy(wp.gameObject);
        }
        waypoints.Clear();

        // Creează waypoint-uri noi
        foreach (Node node in path)
        {
            GameObject wp = new GameObject("Waypoint");
            wp.transform.position = node.worldPosition;
            waypoints.Add(wp.transform);
        }
    }

    private int GetDistance(Node a, Node b)
    {
        // Distanța Manhattan pentru grid rectangular
        int dstX = Mathf.Abs(a.gridX - b.gridX);
        int dstY = Mathf.Abs(a.gridY - b.gridY);

        // 10 = cost drept, 14 = cost diagonal (pentru mișcare pe diagonală)
        return dstX > dstY ?
            14 * dstY + 10 * (dstX - dstY) :
            14 * dstX + 10 * (dstY - dstX);
    }
    void CalculatePath()
    {
        waypoints.Clear();
        currentWaypoint = 0;

        if (gridManager == null || targetSelector.CurrentTarget == null)
        {
            Debug.LogError("GridManager sau Target lipsă!");
            return;
        }

        Node startNode = gridManager.NodeFromWorldPoint(transform.position);
        Node targetNode = gridManager.NodeFromWorldPoint(targetSelector.CurrentTarget.position);

        // Verifică dacă nodurile sunt valide
        if (startNode == null || targetNode == null || !targetNode.walkable)
        {
            Debug.LogWarning("Noduri invalide!");
            return;
        }

        List<Node> openSet = new List<Node>();
        HashSet<Node> closedSet = new HashSet<Node>();
        openSet.Add(startNode);

        while (openSet.Count > 0)
        {
            Node currentNode = openSet[0];
            for (int i = 1; i < openSet.Count; i++)
            {
                if (openSet[i].fCost < currentNode.fCost ||
                    (openSet[i].fCost == currentNode.fCost && openSet[i].hCost < currentNode.hCost))
                {
                    currentNode = openSet[i];
                }
            }

            openSet.Remove(currentNode);
            closedSet.Add(currentNode);

            if (currentNode == targetNode)
            {
                RetracePath(startNode, targetNode);
                return;
            }

            foreach (Node neighbour in gridManager.GetNeighbours(currentNode))
            {
                if (!neighbour.walkable || closedSet.Contains(neighbour))
                    continue;

                int newCost = currentNode.gCost + GetDistance(currentNode, neighbour);
                if (newCost < neighbour.gCost || !openSet.Contains(neighbour))
                {
                    neighbour.gCost = newCost;
                    neighbour.hCost = GetDistance(neighbour, targetNode);
                    neighbour.parent = currentNode;

                    if (!openSet.Contains(neighbour))
                        openSet.Add(neighbour);
                }
            }
        }
    }

    public void TransitionToState(CarState newState)
    {
        switch (newState)
        {
            case CarState.Drive:
                controller.SetBrake(0f);
                break;

            case CarState.AvoidObstacle:
                avoidanceVector = CalculateAvoidance();
                break;
        }
        currentState = newState;
    }

    bool CheckClearPath()
    {
        return !Physics.Raycast(transform.position, targetSelector.CurrentTarget.position - transform.position,
            Vector3.Distance(transform.position, targetSelector.CurrentTarget.position), obstacleMask);
    }

    public void EnterTrafficZone() => TransitionToState(CarState.Stop);
    public void ExitTrafficZone() => TransitionToState(CarState.Drive);

    void OnDrawGizmos()
    {
        if (waypoints != null && waypoints.Count > 0)
        {
            Gizmos.color = Color.green;
            foreach (Transform wp in waypoints)
                Gizmos.DrawSphere(wp.position, 0.5f);
        }
    }
}