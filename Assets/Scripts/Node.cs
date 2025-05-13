using UnityEngine;

public class Node
{
    public bool walkable;
    public Vector3 worldPosition;
    public int gridX, gridY;

    public int gCost, hCost;
    public Node parent;

    public int fCost => gCost + hCost;

    public Node(bool walkable, Vector3 worldPosition, int x, int y)
    {
        this.walkable = walkable;
        this.worldPosition = worldPosition;
        this.gridX = x;
        this.gridY = y;
    }
}
