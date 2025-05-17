using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(CarController))]
public class CarAI : MonoBehaviour
{
    public enum CarState { Idle, Pathfind, Drive, AvoidObstacle, Stop, Recover }
    public CarState currentState = CarState.Pathfind;

    [Header("Lap Counter")]
    public int lapCount = 0;

    [Header("Core References")]
    public GridManager gridManager;
    public TargetSelector targetSelector;
    private CarController controller;
    private Rigidbody rb;

    [Header("Reverse Recovery")]
    public bool reversing = false;
    public float reverseTimer = 2f;          // how long to reverse
    public float reverseSpeed = 10f;          // speed while reversing
    public float reverseBackoffDistance = 3f;   // how far to reverse
    private enum RecoverPhase { BackOff, Pivot }
    private Vector3 recoverStartPos;
    public float frontOffset = 1.5f;           // meters ahead of center
    private float lastFrontDist = Mathf.Infinity;
    private float lastAngle = 0f;
    public float angleDeadzone = 1f;           // degrees
    public float startReverseAngle = 30f;      // deg before throttle
    public float finishAngleThreshold = 10f;   // deg to finish recover

    [Header("Pathfinding")]
    public float waypointRadius = 2f;
    public float recalculateDistance = 5f;
    private List<Transform> waypoints = new List<Transform>();
    private int currentWaypoint = 0;
    private float pathRecalcTimer = 0f;

    [Header("AI Parameters")]
    public float desiredSpeed = 12f;
    public float maxSteerAngle = 25f;
    public float steerResponsiveness = 0.8f;
    public float speedAnticipation = 1.5f;
    public float brakeDistanceMultiplier = 0.8f;

    [Header("Throttle/Brake Smoothing")]
    public float throttleSmoothTime = 0.5f;
    public float brakeSmoothTime = 0.3f;
    private float currentThrottle = 0f;
    private float currentBrake = 0f;


    [Header("Predictive Avoidance")]

    public float reactionTime    = 1.2f;  // seconds until brake onset
    public float maxDeceleration = 8f;    // m/s², how hard you can brake
    public float safetyBuffer    = 7f;    // metres extra beyond stoppingDistance
    public float predictiveRange = 15f;
    public float slowdownThreshold = 10f; // how far before obstacle we start slowing down
    public float minThrottle = 0.3f;

    [Header("Obstacle Avoidance")]
    public float[] rayAngles = { -45f, -30f, -15f, 0f, 15f, 30f, 45f };
    public float rayRange = 15f;
    public LayerMask obstacleMask;
    public float avoidanceForce = 4f;
    private Vector3 avoidanceVector;

    public TrafficLight currentTrafficLight = null;
    public bool inTrafficZone = false;

    void Awake()
    {
        waypoints = new List<Transform>();
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
            case CarState.Recover: UpdateReverseRecovery(); break;

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

    void AutoShift()
    {
        if (controller == null || controller.gears == null || controller.gears.Count == 0)
            return;

        if (controller.isShifting) return;

        float speed = rb.velocity.magnitude * 3.6f; // km/h
        float throttle = currentThrottle;

        // Estimate upcoming curve sharpness
        float steeringDemand = Mathf.Abs(currentWaypoint < waypoints.Count
            ? Vector3.SignedAngle(transform.forward, waypoints[currentWaypoint].position - transform.position, Vector3.up)
            : 0f);

        bool approachingCurve = steeringDemand > 30f;

        int bestGearIndex = controller.currentGearIndex;

        for (int i = 0; i < controller.gears.Count; i++)
        {
            var gear = controller.gears[i];

            if (!gear.isReverse && speed >= gear.minSpeed && speed < gear.maxSpeed)
            {
                bestGearIndex = i;

                // Shift to lower gear earlier if a curve is coming
                if (approachingCurve && i > 1)
                    bestGearIndex = Mathf.Max(i - 1, 1);

                break;
            }
        }

        if (bestGearIndex != controller.currentGearIndex)
            StartCoroutine(controller.GearShiftDelay(bestGearIndex));
    }
    
    private void EnforceSpeedLimit()
    {
        float speed = rb.velocity.magnitude;
        if (speed > desiredSpeed)
        {
            Debug.Log($"Enforcing speed limit: {desiredSpeed} for state {currentState}");
            // no throttle, light brake proportional to overshoot
            float overshoot = speed - desiredSpeed;
            float brakeForce = Mathf.Clamp01(overshoot / desiredSpeed);
            UpdateSpeedControl(0f, brakeForce);
        }
    }

    private void UpdateSpeedControl(float targetThrottle, float targetBrake)
    {
        currentThrottle = Mathf.Lerp(currentThrottle, targetThrottle, Time.deltaTime / throttleSmoothTime);
        currentBrake = Mathf.Lerp(currentBrake, targetBrake, Time.deltaTime / brakeSmoothTime);
        controller.SetThrottle(currentThrottle);
        controller.SetBrake(currentBrake);
    }

    void UpdateDriving()
    {
        if (waypoints == null || waypoints.Count == 0 || 
            currentWaypoint < 0 || currentWaypoint >= waypoints.Count)
        {
            TransitionToState(CarState.Pathfind);
            return;
        }
        EnforceSpeedLimit();
        if (rb.velocity.magnitude > desiredSpeed) return;
        AutoShift();

        

        if (currentWaypoint < waypoints.Count - 1)
        {
            Vector3 wpCurrent = waypoints[currentWaypoint].position;
            Vector3 wpNext = waypoints[currentWaypoint + 1].position;

            float distCurrent = Vector3.Distance(transform.position, wpCurrent);
            float distNext = Vector3.Distance(transform.position, wpNext);

            // Skip to next waypoint if we are closer to it or if we are facing away from the current one
            if (distNext + 0.1f < distCurrent ||
                Vector3.Dot(transform.forward, (wpCurrent - transform.position).normalized) < 0f)
            {
                Debug.Log($"Skipping waypoint {currentWaypoint} to {currentWaypoint + 1}");
                currentWaypoint++;
            }
        }
        if (currentWaypoint >= waypoints.Count)
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
        float speed = rb.velocity.magnitude;
        float throttleInput = Mathf.Clamp01(1f - (speed / desiredSpeed)); // scales from 1 to 0 as we approach desiredSpeed

        if (PredictObstacle(out float obsDist, out float lookAhead))
        {
            float slowFactor = Mathf.Clamp01(obsDist / slowdownThreshold);
            throttleInput = Mathf.Lerp(minThrottle, throttleInput, slowFactor);
            if (obsDist < lookAhead)
            {
                TransitionToState(CarState.AvoidObstacle);
                return;
            }
        }

        if (inTrafficZone && currentTrafficLight != null)
        {
            if (currentTrafficLight.currentState == LightState.Red)
            {
                TransitionToState(CarState.Stop);
                return;
            }
            else if (currentTrafficLight.currentState == LightState.Yellow)
            {
                float distanceToLight = Vector3.Distance(transform.position, currentTrafficLight.transform.position);
                if (distanceToLight < 20f)
                {
                    throttleInput = Mathf.Clamp(throttleInput * 0.5f, 0f, 0.5f);
                }
            }
        }

        float brakeInput = 0f;
        if (distanceToWaypoint < (desiredSpeed * brakeDistanceMultiplier))
            brakeInput = Mathf.Clamp01(1 - (distanceToWaypoint / (desiredSpeed * brakeDistanceMultiplier)));

        UpdateSpeedControl(throttleInput * (1 - brakeInput), brakeInput);

        controller.SetSteer(Mathf.Clamp(adjustedSteer / maxSteerAngle, -1f, 1f));

        if (distanceToWaypoint < waypointRadius)
        {
            currentWaypoint++;
            if (currentWaypoint >= waypoints.Count)
            {
                // Gata cu path-ul actual → trecem la următorul target
                if (targetSelector != null)
                {
                    if (targetSelector.currentIndex == 0)
                        lapCount++;
                    targetSelector.NextTarget(); // schimba targetul
                }

                TransitionToState(CarState.Pathfind); // recalculează path-ul spre noul target
                return;
            }
        }


        if (DetectObstacle())
        {
            float angleToWaypoint = Vector3.Angle(transform.forward, (waypoints[currentWaypoint].position - transform.position).normalized);

            // Car is slow and facing ~wrong direction (i.e. diagonal wall)
            if (speed < 1f && angleToWaypoint > 45f)
            {
                TransitionToState(CarState.Recover);
            }
            else
            {
                TransitionToState(CarState.AvoidObstacle);
            }
        }
        
    }

    float avoidanceTimer = 0f;
    void UpdateAvoidance()
    {

        EnforceSpeedLimit();
        if (rb.velocity.magnitude > desiredSpeed)
            return;
        AutoShift();

        // Predict upcoming obstacle
        float obstacleDist, lookAhead;
        bool hasObstacle = PredictObstacle(out obstacleDist, out lookAhead);
        avoidanceTimer += Time.deltaTime;
         if (avoidanceTimer >= 0.3f && !hasObstacle)
        {
            avoidanceTimer = 0;
            CalculatePath();
            TransitionToState(CarState.Drive);
        }

        // Steering: steer away from the obstacle
        Vector3 avoidance = CalculateAvoidance();
        controller.SetSteer(Mathf.Clamp(avoidance.x, -1f, 1f));

        // Speed control
        float speed = rb.velocity.magnitude;

        if (speed < 1f)
        {
            TransitionToState(CarState.Recover);
        }

        // base throttle to maintain desiredSpeed
        float baseThrottle = Mathf.Clamp01(1f - (speed / desiredSpeed));

        // scale it down by how close we are to the obstacle
        float slowdownFactor = hasObstacle 
            ? Mathf.Clamp01(obstacleDist / slowdownThreshold) 
            : 1f;
        float throttleTarget = Mathf.Min(
            Mathf.Lerp(minThrottle, baseThrottle, slowdownFactor),
            baseThrottle
        );

        // brake only if really close
        float brakeTarget = (hasObstacle && obstacleDist < slowdownThreshold * 0.5f) ? 1f : 0f;

        UpdateSpeedControl(throttleTarget, brakeTarget);

        if (!hasObstacle || obstacleDist > slowdownThreshold * 1.2f)
        {
            pathRecalcTimer = 0f;
            avoidanceTimer = 0;
            CalculatePath();
            TransitionToState(CarState.Drive);
        }
    }




    void UpdateStop()
    {
        controller.SetThrottle(0f);
        controller.SetBrake(1f);

        if (CheckClearPath() && currentTrafficLight!=null && currentTrafficLight.currentState != LightState.Red)
            TransitionToState(CarState.Drive);
    }

    void EnterRecover()
    {
        recoverStartPos = transform.position;
        reverseTimer = 2f;
        reversing = true;
    }

    void UpdateReverseRecovery()
    {
        // Ensure we’re in reverse gear and steering straight
        controller.currentGearIndex = 0;
        controller.SetSteer(0f);
        controller.SetBrake(0f);
        controller.SetThrottle(1f);

        // Count down timer and distance backed
        reverseTimer -= Time.deltaTime;
        float backed = Vector3.Distance(transform.position, recoverStartPos);

        // Once we’ve backed far enough or time’s up, stop and re-path
        if (backed >= reverseBackoffDistance || reverseTimer <= 0f)
        {
            controller.SetThrottle(0f);
            CalculatePath();
            TransitionToState(CarState.Pathfind);
        }
    }

    float GetStoppingDistance()
    {
        float v = rb.velocity.magnitude; //m/s
        float gearFactor = 1f + controller.currentGearIndex * 0.1f;
        float stopD = reactionTime * gearFactor * v // reaction phase
                    + v * v / (2f * maxDeceleration); // braking distance
        return stopD + safetyBuffer;
    }

    bool PredictObstacle(out float obstacleDistance, out float lookAhead)
    {
        lookAhead = GetStoppingDistance();
        obstacleDistance = Mathf.Infinity;
        lookAhead = Mathf.Min(lookAhead, predictiveRange);

        Vector3 origin = transform.position + Vector3.up * 0.5f;
        foreach (float angle in rayAngles)
        {
            Quaternion rot = Quaternion.AngleAxis(angle, transform.up);
            Vector3 dir = rot * transform.forward;

            if (Physics.Raycast(origin, dir, out RaycastHit hit, lookAhead, obstacleMask))
                obstacleDistance = Mathf.Min(obstacleDistance, hit.distance);
        }

        return obstacleDistance < lookAhead;
    }



    bool DetectObstacle()
    {
        Vector3 origin = rb.position + Vector3.up;
        foreach (float angle in rayAngles)
        {
            Quaternion rotation = Quaternion.AngleAxis(angle, transform.up);
            Vector3 direction = rotation * transform.forward;
            Debug.DrawRay(origin, direction * rayRange, Color.yellow, 1f);
            // Raycast for obstacle detection
            if (Physics.SphereCast(origin, 0.5f, direction, out RaycastHit hit, rayRange, obstacleMask))
            {
                Debug.DrawLine(origin, origin + direction * hit.distance, Color.red, 0.2f);
                Debug.Log("Obstacle detected: " + hit.collider.name);
                // If obstacle has Rigidbody, check velocity
                Rigidbody obstacleRb = hit.collider.attachedRigidbody;
                if (obstacleRb != null)
                {
                    Vector3 relativeVelocity = obstacleRb.velocity - rb.velocity;
                    float closingSpeed = Vector3.Dot(relativeVelocity, direction);
                    if (closingSpeed > 0f) // moving toward us
                        return true;
                }
                else
                {
                    return true; // Static obstacle
                }
            }
        }
        return false;
    }

    Vector3 CalculateAvoidance()
    {
        Vector3 totalForce = Vector3.zero;
        Vector3 origin = transform.position + Vector3.up * 0.5f;

        foreach (float angle in rayAngles)
        {
            Quaternion rot = Quaternion.AngleAxis(angle, transform.up);
            Vector3 dir = rot * transform.forward;

            if (Physics.Raycast(origin, dir, out RaycastHit hit, rayRange, obstacleMask))
            {
                Vector3 avoidDir;

                if (Mathf.Approximately(angle, 0f))
                {
                    // For the direct-forward ray, use the obstacle's surface normal
                    avoidDir = Vector3.ProjectOnPlane(hit.normal, Vector3.up).normalized;
                }
                else
                {
                    Vector3 toObs = (hit.point - transform.position).normalized;
                    avoidDir = Vector3.Cross(Vector3.up, toObs).normalized;
                }

                // Scale by proximity
                float strength = avoidanceForce * (1f - (hit.distance / rayRange));
                totalForce += avoidDir * strength;

                // Extra push when very close
                if (hit.distance < rayRange * 0.3f)
                {
                    float pushForce = avoidanceForce * 2f * (1f - (hit.distance / rayRange));
                    totalForce    += avoidDir * pushForce;
                }
            }
        }

        if (totalForce == Vector3.zero)
            return Vector3.zero;

        // Convert world-space force into local-steer vector
        Vector3 localForce = transform.InverseTransformDirection(totalForce);
        return localForce.normalized;
    }
    // Adaugă aceste metode în CarAI.cs
    private void RetracePath(Node startNode, Node endNode)
    {
        Debug.Log("Retracing path from " +
            startNode.gridX + "," + startNode.gridY +
            " to " + endNode.gridX + "," + endNode.gridY);

        if (startNode == null || endNode == null)
        {
            Debug.LogWarning("Start or end node is null.");
            return;
        }
        if (endNode.parent == null)
        {
            Debug.LogWarning("End node has no parent. Cannot retrace path.");
            return;
        }

        List<Node> path = new List<Node>();
        Node currentNode = endNode;
        while (currentNode != startNode)
        {
            path.Add(currentNode);
            currentNode = currentNode.parent;
            if (currentNode == null)
            {
                Debug.LogError("Path retrace failed - null parent!");
                return;
            }
        }
        path.Reverse();

        int firstValid = 0;
        for (int i = 0; i < path.Count; i++)
        {
            Vector3 toNode = path[i].worldPosition - transform.position;
            if (Vector3.Dot(transform.forward, toNode.normalized) > 0f)
            {
                firstValid = i;
                break;
            }
        }
        if (firstValid > 0)
            path.RemoveRange(0, firstValid);

        if (path.Count == 0)
        {
            Debug.Log("Reached target (no forward nodes). Switching to next.");
            if (targetSelector != null)
                targetSelector.NextTarget();
            TransitionToState(CarState.Pathfind);
            return;
        }

        foreach (Transform wp in waypoints)
        {
            if (wp != null)
                Destroy(wp.gameObject);
        }
        waypoints.Clear();

        foreach (Node node in path)
        {
            GameObject wp = new GameObject("Waypoint");
            wp.transform.position = node.worldPosition;
            waypoints.Add(wp.transform);
            Debug.DrawRay(node.worldPosition, Vector3.up * 2, Color.green, 5f);
        }

        currentWaypoint = 0;
        TransitionToState(CarState.Drive);
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
            Debug.LogError("GridManager or Target missing!");
            return;
        }

        Node startNode  = gridManager.NodeFromWorldPoint(transform.position);
        Node targetNode = gridManager.NodeFromWorldPoint(targetSelector.CurrentTarget.position);
        if (startNode == null || targetNode == null || !targetNode.walkable)
        {
            Debug.LogWarning("Invalid start or target node!");
            return;
        }

        var openSet   = new List<Node> { startNode };
        var closedSet = new HashSet<Node>();

        while (openSet.Count > 0)
        {
            //–– find node with lowest fCost
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

            //–– buffer: any neighbour adjacent to an obstacle gets skipped
            foreach (Node neighbour in gridManager.GetNeighbours(currentNode))
            {
                if (!neighbour.walkable || closedSet.Contains(neighbour))
                    continue;

                // **new!** skip neighbours that border a non-walkable node
                bool touchesObstacle = false;
                foreach (Node adj in gridManager.GetNeighbours(neighbour))
                {
                    if (!adj.walkable)
                    {
                        touchesObstacle = true;
                        break;
                    }
                }
                if (touchesObstacle)
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
        Debug.Log($"Transitioning from {currentState} to {newState}");
        switch (newState)
        {
            case CarState.Drive:
                controller.SetBrake(1f);               // lock the wheels for one frame
                controller.currentGearIndex = 1;       // first forward gear
                controller.SetThrottle(0f);
                break;

            case CarState.AvoidObstacle:
                avoidanceTimer = 0f;    
                avoidanceVector = CalculateAvoidance();
                break;

            case CarState.Recover:
                EnterRecover();
                break;
        }
        currentState = newState;
    }

    bool CheckClearPath()
    {
        return !Physics.Raycast(transform.position, targetSelector.CurrentTarget.position - transform.position,
            Vector3.Distance(transform.position, targetSelector.CurrentTarget.position), obstacleMask);
    }

    public void EnterTrafficZone(TrafficLight light)
    {
        currentTrafficLight = light;
        inTrafficZone = true;
        Debug.Log("Entered traffic zone with light: " + light.name);
    }

    public void ExitTrafficZone()
    {
        currentTrafficLight = null;
        inTrafficZone = false;
        Debug.Log("Exited traffic zone");
        TransitionToState(CarState.Drive);
    }

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