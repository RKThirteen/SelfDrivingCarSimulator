using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CarAI : MonoBehaviour
{
    public enum CarState
    {
        Idle,
        Drive,
        AvoidObstacle,
        Stop,
        Pathfind
    }
    public CarState currentState = CarState.Idle;

    [Header("Detection")]
    public float obstacleDetectionRange = 5f;
    public LayerMask obstacleLayer;
    public bool trafficLightRed = false;

    [Header("Path Following")]
    private List<Transform> waypoints;
    private int currentWaypoint = 0;
    public float speed = 5f;
    public float turnSpeed = 5f;
    public float waypointReachDistance = 1f;
    private Transform target; // Target for pathfinding
    private TargetSelector selector;
    // Start is called before the first frame update
    private Rigidbody rb;
    private GridManager gridManager;
    public void TransitionToState(CarState newState)
    {
        currentState = newState;
        Debug.Log("Transitioning to state: " + newState);
    }

    bool ObstacleDetected()
    {
        RaycastHit hit;
        Vector3 origin = transform.position + Vector3.up * 0.5f;

        Debug.DrawRay(origin, transform.forward * obstacleDetectionRange, Color.red);

        if (Physics.Raycast(origin, transform.forward, out hit, obstacleDetectionRange, obstacleLayer))
        {
            Debug.Log("Detected obstacle: " + hit.collider.gameObject.name);
            return true;
        }

        return false;
    }

    void Awake()
    {
        gridManager = FindObjectOfType<GridManager>();
        selector = FindObjectOfType<TargetSelector>();
        if (gridManager == null )
        {
            Debug.LogError("GridManager not found in the scene.");
        }
        if (selector == null)
        {
            Debug.LogError("TargetSelector not found in the scene.");
        }
    }
    void Start()
    {
        rb = GetComponent<Rigidbody>();
        TransitionToState(CarState.Pathfind);
        waypoints = new List<Transform>();
    }
    void Idle()
    {
        if (waypoints != null)
        {
            foreach (Transform wp in waypoints)
            {
                if (wp != null) Destroy(wp.gameObject);
            }
            waypoints.Clear();
        }
        else
        {
            waypoints = new List<Transform>();
        }
        // Așteaptă să primească comenzi
        if (Input.GetKeyDown(KeyCode.KeypadEnter))
        {
            TransitionToState(CarState.Pathfind);
        }
    }
     void Drive()
    {   
        if (waypoints == null)
        {
            Debug.LogWarning("Waypoints list is null in Drive().");
            return;
        }
        if (ObstacleDetected())
        {
            TransitionToState(CarState.AvoidObstacle);
            return;
        }

        if (trafficLightRed)
        {
            TransitionToState(CarState.Stop);
            return;
        }

        if (waypoints.Count == 0 || waypoints==null) return;

        Vector3 direction = (waypoints[currentWaypoint].position - transform.position).normalized;
        Quaternion targetRotation = Quaternion.LookRotation(direction);
        transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, turnSpeed * Time.deltaTime);
        rb.velocity = transform.forward * speed;

        if (Vector3.Distance(transform.position, waypoints[currentWaypoint].position) < waypointReachDistance)
        {
            currentWaypoint++;
            if (currentWaypoint >= waypoints.Count)
            {
                // Gata cu path-ul actual → trecem la următorul target
                if (selector != null)
                {
                    selector.NextTarget(); // schimba targetul
                }

                TransitionToState(CarState.Pathfind); // recalculează path-ul spre noul target
            }
        }
    }

    void AvoidObstacle()
    {
        Quaternion turn = Quaternion.Euler(0, 45, 0); // virează ușor la dreapta
        transform.rotation = Quaternion.Slerp(transform.rotation, transform.rotation * turn, turnSpeed * Time.deltaTime);
        rb.velocity = transform.forward * speed;

        if (!ObstacleDetected())
        {
            TransitionToState(CarState.Drive);
        }
    }

    void StopAtSignal()
    {
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        if (!trafficLightRed)
        {
            TransitionToState(CarState.Drive);
        }
    }

    int GetDistance(Node a, Node b)
    {
        int dstX = Mathf.Abs(a.gridX - b.gridX);
        int dstY = Mathf.Abs(a.gridY - b.gridY);
        return 10 * (dstX + dstY); // cost constant, simplificat
    }

    void RetracePath(Node startNode, Node endNode)
    {
        Debug.Log("Retracing path from " + startNode.gridX + "," + startNode.gridY + " to " + endNode.gridX + "," + endNode.gridY);
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

        // Asigurare că lista e creată
        if (waypoints != null)
        {
            foreach (Transform wp in waypoints)
            {
                if (wp != null) Destroy(wp.gameObject);
            }
            waypoints.Clear();
        }
        else
        {
            waypoints = new List<Transform>();
        }
        foreach (Transform wp in waypoints)
        {
            if (wp != null) Destroy(wp.gameObject);
        }
        List<Node> path = new List<Node>();
        Node currentNode = endNode;

        while (currentNode != startNode)
        {
            path.Add(currentNode);
            currentNode = currentNode.parent;
        }
        path.Reverse();

        waypoints = new List<Transform>();
        foreach (Node n in path)
        {
            GameObject wp = new GameObject("Waypoint");
            wp.transform.position = n.worldPosition;
            waypoints.Add(wp.transform);
            Debug.DrawRay(n.worldPosition, Vector3.up * 2, Color.green, 5f);
        }

        currentWaypoint = 0;
        TransitionToState(CarState.Drive);
    }
    void CalculatePath()
    {
        if (selector == null || selector.CurrentTarget == null)
        {
            Debug.LogWarning("No target available");
            return;
        }

        target = selector.CurrentTarget;

        Node startNode = gridManager.NodeFromWorldPoint(transform.position);
        Node targetNode = gridManager.NodeFromWorldPoint(target.position);

        List<Node> openSet = new List<Node>();
        HashSet<Node> closedSet = new HashSet<Node>();
        openSet.Add(startNode);

        while (openSet.Count > 0)
        {
            Node currentNode = openSet[0];
            for (int i = 1; i < openSet.Count; i++)
            {
                if (openSet[i].fCost < currentNode.fCost ||
                    openSet[i].fCost == currentNode.fCost && openSet[i].hCost < currentNode.hCost)
                {
                    currentNode = openSet[i];
                }
            }

            openSet.Remove(currentNode);
            closedSet.Add(currentNode);

            if (currentNode == targetNode)
            {
                if (targetNode.parent != null)
                {
                    RetracePath(startNode, targetNode);
                }
                else
                {
                    Debug.LogWarning("No valid path to target found!");
                    TransitionToState(CarState.Idle);
                }
                return;
            }

            foreach (Node neighbour in gridManager.GetNeighbours(currentNode))
            {
                if (!neighbour.walkable || closedSet.Contains(neighbour))
                    continue;

                int newCostToNeighbour = currentNode.gCost + GetDistance(currentNode, neighbour);
                if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour))
                {
                    neighbour.gCost = newCostToNeighbour;
                    neighbour.hCost = GetDistance(neighbour, targetNode);
                    neighbour.parent = currentNode;

                    if (!openSet.Contains(neighbour))
                        openSet.Add(neighbour);
                }
            }
        }
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            TransitionToState(CarState.Idle);
        }
        switch (currentState)
        {
            case CarState.Idle:
                Idle();
                break;

            case CarState.Drive:
                Drive();
                break;

            case CarState.AvoidObstacle:
                AvoidObstacle();
                break;

            case CarState.Stop:
                StopAtSignal();
                break;

            case CarState.Pathfind:
                CalculatePath();
                break;
        }
    }

}
