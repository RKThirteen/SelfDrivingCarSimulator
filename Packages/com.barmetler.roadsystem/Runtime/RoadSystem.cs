using System;
using System.Collections.Generic;
using System.Linq;
using Barmetler.RoadSystem.Util;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using NodeType = Barmetler.RoadSystem.RoadSystem.Graph.Node.NodeType;

namespace Barmetler.RoadSystem
{
    using PointList = List<Bezier.OrientedPoint>;

    [ExecuteAlways]
    public class RoadSystem : MonoBehaviour
    {
        [SerializeField, HideInInspector]
        private Intersection[] intersections;

        [SerializeField, HideInInspector]
        private Road[] roads;

        [SerializeField, HideInInspector]
        private Graph graph = new Graph();

        public bool ShowDebugInfo = true;
        public bool ShowEdgeWeights = true;

        public float DistanceFactor = 1000;

        public struct Edge
        {
            public Vector3 start, end;
            public float cost;
        }

        private void OnEnable()
        {
            graph.InitNativeArray();
        }

        private void OnDisable()
        {
            graph.DisposeNativeArray();
        }

        private void OnValidate()
        {
            intersections = GetComponentsInChildren<Intersection>();
            roads = GetComponentsInChildren<Road>();
            graph.roadSystem = this;
        }

        public void RebuildAllRoads()
        {
            ConstructGraph();
            foreach (var road in roads)
            {
                road.OnCurveChanged(true);
            }
        }

        public Intersection[] Intersections => intersections;
        public Road[] Roads => roads;

        public float GetMinDistance(Vector3 worldPosition, float stepSize, float yScale,
            out Road closestRoad, out Vector3 closestPoint, out float distanceAlongRoad)
        {
            // Initiate output parameters
            closestRoad = null;
            closestPoint = Vector3.zero;
            distanceAlongRoad = 0;

            var minDst = float.PositiveInfinity;

            foreach (var road in roads)
            {
                if (road.IsMaybeCloser(worldPosition, minDst * yScale, yScale))
                {
                    var dst = road.GetMinDistance(worldPosition, stepSize, yScale, out var newClosestPoint,
                        out var newDistanceAlongRoad);
                    if (dst < minDst)
                    {
                        minDst = dst;
                        closestRoad = road;
                        closestPoint = newClosestPoint;
                        distanceAlongRoad = newDistanceAlongRoad;
                    }
                }
            }

            return minDst;
        }

        public float GetMinDistance(Vector3 worldPosition, float yScale,
            out Intersection closestIntersection, out RoadAnchor closestAnchor, out Vector3 closestPoint,
            out float distanceAlongRoad)
        {
            // Initiate output parameters
            closestIntersection = null;
            closestAnchor = null;
            closestPoint = Vector3.zero;
            distanceAlongRoad = 0;

            var minDst = float.PositiveInfinity;

            foreach (var intersection in intersections)
            {
                if (!intersection) continue;
                var dst =
                    Vector3.Scale(intersection.transform.position - worldPosition, new Vector3(1, yScale, 1))
                        .magnitude -
                    intersection.Radius;
                if (dst < minDst)
                {
                    foreach (var anch in intersection.AnchorPoints)
                    {
                        var a = intersection.transform.position;
                        var b = anch.transform.position;
                        var l = (b - a).magnitude;
                        var n = (b - a).normalized;
                        var along = Vector3.Dot(worldPosition - a, n);
                        float newDst;
                        Vector3 closestPt;
                        if (along < 0)
                        {
                            closestPt = a;
                            along = 0;
                        }
                        else if (along > l)
                        {
                            closestPt = b;
                            along = l;
                        }
                        else
                        {
                            closestPt = a + n * along;
                        }

                        newDst = Vector3.Scale(worldPosition - closestPt, new Vector3(1, yScale, 1)).magnitude;
                        if (newDst < minDst)
                        {
                            minDst = newDst;
                            closestPoint = closestPt;
                            distanceAlongRoad = along;
                            closestIntersection = intersection;
                            closestAnchor = anch;
                        }
                    }
                }
            }

            return minDst;
        }

        public void ConstructGraph()
        {
            OnValidate();
            graph.ConstructGraph();
        }

        public List<Edge> GetGraphEdges()
        {
            return graph.GetEdges();
        }

        public PointList FindPath(
            Vector3 startPosWorld, Vector3 goalPosWorld, List<Edge> edges = null,
            float yScale = 1, float stepSize = 1, float minDstToRoadToConnect = 10
        ) => FindPath(
            startPosWorld, goalPosWorld,
            out _, edges,
            yScale: yScale, stepSize: stepSize,
            minDstToRoadToConnect: minDstToRoadToConnect
        );

        public PointList FindPath(
            Vector3 startPosWorld, Vector3 goalPosWorld,
            out int stepsTaken, List<Edge> edges = null,
            float yScale = 1, float stepSize = 1, float minDstToRoadToConnect = 10)
        {
            var startDistRoad = GetMinDistance(
                startPosWorld, stepSize, yScale,
                out var startRoad, out var startPosition1, out var startDstAlongRoad1);
            var startDistIntersection = GetMinDistance(
                startPosWorld, yScale,
                out _, out var startAnchor,
                out var startPosition2, out var startDstAlongRoad2);
            var startUseRoad = startDistRoad < startDistIntersection;

            var goalDistRoad = GetMinDistance(
                goalPosWorld, stepSize, yScale,
                out var goalRoad, out var goalPosition1, out var goalDstAlongRoad1);
            var goalDistIntersection = GetMinDistance(
                goalPosWorld, yScale,
                out _, out var goalAnchor,
                out var goalPosition2, out var goalDstAlongRoad2);
            var goalUseRoad = goalDistRoad < goalDistIntersection;

            if (!graph.NativeArrayIsCreated)
                graph.InitNativeArray();

            var nodes = graph.FindPathBurst(
                startUseRoad ? startPosition1 : startPosition2,
                startUseRoad ? startRoad : null,
                startUseRoad ? null : startAnchor,
                startUseRoad ? startDstAlongRoad1 : startDstAlongRoad2,
                goalUseRoad ? goalPosition1 : goalPosition2,
                goalUseRoad ? goalRoad : null,
                goalUseRoad ? null : goalAnchor,
                goalUseRoad ? goalDstAlongRoad1 : goalDstAlongRoad2,
                out stepsTaken,
                edges: edges
            );

            return GenerateSmoothPath(startPosWorld, goalPosWorld, nodes, stepSize, minDstToRoadToConnect);
        }

        private static PointList GenerateSmoothPath(
            Vector3 startPosWorld, Vector3 goalPosWorld, List<Graph.Node> nodes,
            float stepSize = 1, float minDstToRoadToConnect = 10, bool onlyNodes = false,
            bool subdivideStraightLines = false)
        {
            var pathPoints = new PointList();

            if (onlyNodes)
            {
                pathPoints.AddRange(nodes.Select(node =>
                    new Bezier.OrientedPoint(node.GetWorldPosition(), Vector3.forward, Vector3.up)));
                return pathPoints;
            }

            for (var nodeIndex = 0; nodeIndex < nodes.Count - 1; ++nodeIndex)
            {
                var node = nodes[nodeIndex];
                var nextNode = nodes[nodeIndex + 1];
                if (!node.roadSystem)
                    throw new InvalidOperationException($"nodes contains uninitialized node at index {nodeIndex}.");
                if (!nextNode.roadSystem)
                    throw new InvalidOperationException($"nodes contains uninitialized node at index {nodeIndex + 1}.");

                switch (node.nodeType, nextNode.nodeType)
                {
                    // nodes on the same road
                    case (NodeType.ENTRY_EXIT, NodeType.ENTRY_EXIT) when node.road && node.road == nextNode.road:
                    {
                        var from = Mathf.Min(node.distanceAlongRoad, nextNode.distanceAlongRoad);
                        var to = Mathf.Max(node.distanceAlongRoad, nextNode.distanceAlongRoad);
                        var roadPoints = node.road.GetEvenlySpacedPoints(stepSize);
                        var dist = 0f;
                        var pathPointsCount = pathPoints.Count;
                        for (var i = 0; i < roadPoints.Length; ++i)
                        {
                            if (i > 0)
                                dist += Vector3.Distance(roadPoints[i - 1].position, roadPoints[i].position);
                            if (dist > to) break;
                            if (dist >= from)
                                pathPoints.Add(roadPoints[i].ToWorldSpace(node.road.transform));
                        }

                        if (pathPoints.Count > pathPointsCount)
                        {
                            if (node.distanceAlongRoad > nextNode.distanceAlongRoad)
                                pathPoints.Reverse(pathPointsCount, pathPoints.Count - pathPointsCount);
                            pathPoints.Insert(pathPointsCount, new Bezier.OrientedPoint(
                                node.GetWorldPosition(), pathPoints[pathPointsCount].forward,
                                pathPoints[pathPointsCount].normal));
                            pathPoints.Add(new Bezier.OrientedPoint(
                                nextNode.GetWorldPosition(), pathPoints[pathPoints.Count - 1].forward,
                                pathPoints[pathPoints.Count - 1].normal));
                        }

                        break;
                    }

                    case (NodeType.ANCHOR, NodeType.ANCHOR)
                        when node.anchor && nextNode.anchor && node.anchor != nextNode.anchor &&
                             node.anchor.GetConnectedRoad() &&
                             node.anchor.GetConnectedRoad() == nextNode.anchor.GetConnectedRoad():
                    {
                        var road = node.anchor.GetConnectedRoad(out var forward);
                        var roadPoints = road.GetEvenlySpacedPoints(stepSize);
                        // the start point should already be on the path if it's not the first node
                        pathPoints.AddRange(
                            (forward ? roadPoints : roadPoints.Reverse())
                            .Select(point => point.ToWorldSpace(road.transform))
                            .Skip(pathPoints.Count > 0 ? 1 : 0));
                        break;
                    }

                    case (NodeType.ENTRY_EXIT, NodeType.ANCHOR)
                        when node.road && nextNode.anchor && node.road == nextNode.anchor.GetConnectedRoad():
                    case (NodeType.ANCHOR, NodeType.ENTRY_EXIT)
                        when node.anchor && nextNode.road && node.anchor.GetConnectedRoad() == nextNode.road:
                    {
                        // current node is on the road
                        var isStart = node.nodeType == NodeType.ENTRY_EXIT;
                        var road = isStart ? node.road : nextNode.road;
                        var anchor = isStart ? nextNode.anchor : node.anchor;
                        // line connects from road.start to node on road
                        var isFromStart = road.start == anchor;
                        var distanceAlongRoad = isStart ? node.distanceAlongRoad : nextNode.distanceAlongRoad;
                        var roadPoints = road.GetEvenlySpacedPoints(stepSize);
                        var pathPointsCount = pathPoints.Count;
                        var dist = 0f;
                        for (var i = 0; i < roadPoints.Length; ++i)
                        {
                            if (i > 0)
                                dist += Vector3.Distance(roadPoints[i - 1].position, roadPoints[i].position);
                            if (isFromStart && dist > distanceAlongRoad) break;
                            if (isFromStart ? dist <= distanceAlongRoad : dist >= distanceAlongRoad)
                                pathPoints.Add(roadPoints[i].ToWorldSpace(road.transform));
                        }

                        // added points need to be reversed if we come from road.end to node on road
                        // or from node on road to road.start
                        var isReverse = isStart == isFromStart;
                        if (pathPoints.Count > pathPointsCount)
                        {
                            if (isReverse) pathPoints.Reverse(pathPointsCount, pathPoints.Count - pathPointsCount);
                            if (isStart)
                                pathPoints.Insert(pathPointsCount, new Bezier.OrientedPoint(
                                    node.GetWorldPosition(), pathPoints[pathPointsCount].forward,
                                    pathPoints[pathPointsCount].normal));
                            else
                                pathPoints.Add(new Bezier.OrientedPoint(
                                    nextNode.GetWorldPosition(), pathPoints[pathPoints.Count - 1].forward,
                                    pathPoints[pathPoints.Count - 1].normal));
                        }

                        break;
                    }

                    default:
                    {
                        var pos1 = node.GetWorldPosition();
                        var norm1 = node.anchor?.transform.up ?? node.intersection?.transform.up ?? Vector3.up;
                        var pos2 = nextNode.GetWorldPosition();
                        var norm2 = nextNode.anchor?.transform.up ?? nextNode.intersection?.transform.up ?? Vector3.up;
                        var dist = Vector3.Distance(pos1, pos2);
                        var n = (pos2 - pos1).normalized;
                        var nPoints = subdivideStraightLines ? Mathf.CeilToInt(dist / stepSize) + 1 : 2;
                        for (var i = pathPoints.Count > 0 ? 1 : 0; i < nPoints; ++i)
                        {
                            var t = Mathf.Lerp(0, dist, (float)i / Mathf.Max(1, nPoints - 1));
                            pathPoints.Add(new Bezier.OrientedPoint(pos1 + t * n, n,
                                Vector3.Lerp(norm1, norm2, t / dist)));
                        }

                        break;
                    }
                }
            }

            // Connect Start and End Point to road
            if (pathPoints.Count > 0)
            {
                for (var end = 0; end < 2; ++end)
                {
                    var pos = end == 0 ? startPosWorld : goalPosWorld;
                    var point = pathPoints[end * (pathPoints.Count - 1)];
                    var dst = (pos - point.position).magnitude;
                    if (dst >= minDstToRoadToConnect)
                    {
                        var count = (int)(dst / stepSize);
                        for (var i = count - 1; i >= 0; --i)
                        {
                            var t = (float)i / count;
                            var newPoint = new Bezier.OrientedPoint(
                                Vector3.Lerp(pos, point.position, t), point.forward, point.normal);
                            if (end == 0)
                                pathPoints.Insert(0, newPoint);
                            else
                                pathPoints.Add(newPoint);
                        }
                    }
                }
            }

            return pathPoints;
        }

        [Serializable]
        internal class Graph
        {
            [Serializable]
            public class Node : AStar.NodeBase
            {
                public enum NodeType
                {
                    INTERSECTION,
                    ANCHOR,
                    ENTRY_EXIT
                }

                public NodeType nodeType;
                public RoadSystem roadSystem;

                public Intersection intersection;
                public RoadAnchor anchor;
                public Road road;
                public float distanceAlongRoad;

                public Node(RoadSystem roadSystem)
                {
                    this.roadSystem = roadSystem;
                    nodeType = NodeType.ENTRY_EXIT;
                }

                public Node(Vector3 worldPosition, Road road, float distanceAlongRoad, RoadSystem roadSystem)
                {
                    this.roadSystem = roadSystem;
                    nodeType = NodeType.ENTRY_EXIT;

                    position = roadSystem.transform.InverseTransformPoint(worldPosition);
                    this.road = road;
                    this.distanceAlongRoad = distanceAlongRoad;
                }

                public Node(Vector3 worldPosition, RoadAnchor anchor, float distanceAlongRoad, RoadSystem roadSystem)
                {
                    this.roadSystem = roadSystem;
                    nodeType = NodeType.ENTRY_EXIT;

                    position = roadSystem.transform.InverseTransformPoint(worldPosition);
                    this.anchor = anchor;
                    this.distanceAlongRoad = distanceAlongRoad;
                }

                public Node(Intersection intersection, RoadSystem roadSystem)
                {
                    this.roadSystem = roadSystem;
                    nodeType = NodeType.INTERSECTION;

                    this.intersection = intersection;
                    position = roadSystem.transform.InverseTransformPoint(intersection.transform.position);
                }

                public Node(RoadAnchor anchor, RoadSystem roadSystem)
                {
                    this.roadSystem = roadSystem;
                    nodeType = NodeType.ANCHOR;

                    this.anchor = anchor;
                    position = roadSystem.transform.InverseTransformPoint(anchor.transform.position);
                }

                public Vector3 GetWorldPosition()
                {
                    return roadSystem.transform.TransformPoint(position);
                }
            }

            [SerializeField]
            public RoadSystem roadSystem;

            [SerializeField]
            private List<Node> nodes = new List<Node>();

            [SerializeField]
            private TwoDimensionalArray<float> weights = new TwoDimensionalArray<float>(0, 0);

            private TwoDimensionalNativeArray<float> _weightsNativeArray;

            public bool NativeArrayIsCreated => _weightsNativeArray.IsCreated;

            public void InitNativeArray()
            {
                if (_weightsNativeArray.IsCreated) _weightsNativeArray.Dispose();
                _weightsNativeArray =
                    new TwoDimensionalNativeArray<float>(weights.Width, weights.Height, Allocator.Persistent);
                _weightsNativeArray.CopyFrom(weights.DirectArray);
            }

            public void DisposeNativeArray()
            {
                if (_weightsNativeArray.IsCreated) _weightsNativeArray.Dispose();
            }

            /// <summary>
            /// Constructs the graph from the road system.
            /// </summary>
            public void ConstructGraph()
            {
                nodes = new List<Node>();
                var count = roadSystem.intersections.Select(intersection => 1 + intersection.AnchorPoints.Length).Sum();
                weights = new TwoDimensionalArray<float>(count, count);

                for (var i = 0; i < count; ++i)
                for (var j = 0; j < count; ++j)
                    weights[i, j] = float.PositiveInfinity;

                foreach (var intersection in roadSystem.intersections)
                {
                    intersection.Invalidate(false);
                    var index = nodes.Count;
                    nodes.Add(new Node(intersection, roadSystem));
                    for (var i = 0; i < intersection.AnchorPoints.Length; ++i)
                    {
                        nodes.Add(new Node(intersection.AnchorPoints[i], roadSystem));
                        weights[index, index + i + 1] = weights[index + i + 1, index] =
                            (nodes[index].position - nodes[index + i + 1].position).magnitude;
                    }
                }

                foreach (var road in roadSystem.roads)
                {
                    road.OnValidate();
                    if (road.start && road.end)
                    {
                        var startIndex = FindIndex(road.start, nodes);
                        var endIndex = FindIndex(road.end, nodes);
                        if (startIndex != -1 && endIndex != -1)
                            weights[startIndex, endIndex] = weights[endIndex, startIndex] = road.GetLength();
                    }
                }

                // Connect all nodes with 1000x their distance, so that islands can still connect, and not exception will be thrown.
                // This shouldn't really be needed, because islands shouldn't exist in a roadsystem, so it's more of a failsafe.
                for (var i = 0; i < count; ++i)
                {
                    for (var j = i; j < count; ++j)
                    {
                        var weight = (nodes[i].position - nodes[j].position).magnitude * roadSystem.DistanceFactor;
                        if (float.IsInfinity(weights[i, j])) weights[i, j] = weight;
                        if (float.IsInfinity(weights[j, i])) weights[j, i] = weight;
                    }
                }

                InitNativeArray();
            }

            /// <summary>
            /// For debugging purposes.
            /// </summary>
            /// <returns>All edges in the graph.</returns>
            public List<Edge> GetEdges()
            {
                var ret = new List<Edge>();
                for (var from = 0; from < nodes.Count; ++from)
                {
                    for (var to = 0; to < nodes.Count; ++to)
                    {
                        if (weights[from, to] < 5e3 && weights[from, to] > 1e-3)
                        {
                            var edge = new Edge
                            {
                                start = roadSystem.transform.TransformPoint(nodes[from].position),
                                end = roadSystem.transform.TransformPoint(nodes[to].position),
                                cost = weights[from, to]
                            };
                            ret.Add(edge);
                        }
                    }
                }

                return ret;
            }

            private static int FindIndex(Intersection intersection, List<Node> nodes)
            {
                if (!intersection) return -1;
                return nodes.FindIndex(0, nodes.Count,
                    node => node.nodeType == NodeType.INTERSECTION && node.intersection == intersection);
            }

            private static int FindIndex(RoadAnchor anchor, List<Node> nodes)
            {
                if (!anchor) return -1;
                return nodes.FindIndex(0, nodes.Count,
                    node => node.nodeType == NodeType.ANCHOR && node.anchor == anchor);
            }

            /// <summary>
            /// Find the shortest path from one point to another in the road system. This burst version is about 3x as
            /// fast as the non-burst version.
            /// </summary>
            /// <param name="startPosWorld">World position of the start point.</param>
            /// <param name="startRoad">Road the start point is on, or null if it is on an intersection.</param>
            /// <param name="startAnchor">Anchor the start point is on, or null if it is on a road.</param>
            /// <param name="startDistanceAlongRoad">Distance along the road or anchor the start point is at.</param>
            /// <param name="goalPosWorld">World position of the goal point.</param>
            /// <param name="goalRoad">Road the goal point is on, or null if it is on an intersection.</param>
            /// <param name="goalAnchor">Anchor the goal point is on, or null if it is on a road.</param>
            /// <param name="goalDistanceAlongRoad">Distance along the road or anchor the goal point is at.</param>
            /// <param name="stepsTaken"></param>
            /// <param name="edges">May be null. If not null, will be populated with the edges in the path.
            /// </param>
            /// <returns></returns>
            /// <exception cref="InvalidOperationException"></exception>
            /// <exception cref="ArgumentException"></exception>
            public List<Node> FindPathBurst(
                Vector3 startPosWorld, Road startRoad, RoadAnchor startAnchor, float startDistanceAlongRoad,
                Vector3 goalPosWorld, Road goalRoad, RoadAnchor goalAnchor, float goalDistanceAlongRoad,
                out int stepsTaken, List<Edge> edges = null)
            {
                if (startRoad == null && startAnchor == null)
                    throw new ArgumentException("Start must be either a road or an anchor.");
                if (goalRoad == null && goalAnchor == null)
                    throw new ArgumentException("Goal must be either a road or an anchor.");

                if (!_weightsNativeArray.IsCreated)
                    throw new InvalidOperationException("NativeArray not created.");

                if (_weightsNativeArray.Width != nodes.Count || _weightsNativeArray.Height != nodes.Count)
                    throw new InvalidOperationException("NativeArray size does not match nodes size.");

                var nodesList = nodes.ToList();

                nodesList.Insert(0,
                    startRoad != null
                        ? new Node(startPosWorld, startRoad, startDistanceAlongRoad, roadSystem)
                        : new Node(startPosWorld, startAnchor, startDistanceAlongRoad, roadSystem));

                nodesList.Insert(1,
                    goalRoad != null
                        ? new Node(goalPosWorld, goalRoad, goalDistanceAlongRoad, roadSystem)
                        : new Node(goalPosWorld, goalAnchor, goalDistanceAlongRoad, roadSystem));

                const int startIndex = 0;
                const int goalIndex = 1;

                var startRoadStartIndex = startRoad != null ? FindIndex(startRoad.start, nodesList) : -1;
                var startRoadEndIndex = startRoad != null ? FindIndex(startRoad.end, nodesList) : -1;
                var goalRoadStartIndex = goalRoad != null ? FindIndex(goalRoad.start, nodesList) : -1;
                var goalRoadEndIndex = goalRoad != null ? FindIndex(goalRoad.end, nodesList) : -1;
                var startAnchorIndex = startAnchor != null ? FindIndex(startAnchor, nodesList) : -1;
                var goalAnchorIndex = goalAnchor != null ? FindIndex(goalAnchor, nodesList) : -1;
                var startIntersectionIndex = startAnchor != null ? FindIndex(startAnchor.Intersection, nodesList) : -1;
                var goalIntersectionIndex = goalAnchor != null ? FindIndex(goalAnchor.Intersection, nodesList) : -1;

                // if start or goal is coincident with an existing node, A* will fail, so we just move them a bit.
                // This will only affect the search, not the rendering of the path.
                // This is very hacky, but it works.
                for (var i = 0; i < 2; ++i)
                {
                    var nodeIndex = i == 0 ? startIndex : goalIndex;
                    var roadStartIndex = i == 0 ? startRoadStartIndex : goalRoadStartIndex;
                    var roadEndIndex = i == 0 ? startRoadEndIndex : goalRoadEndIndex;
                    var anchorIndex = i == 0 ? startAnchorIndex : goalAnchorIndex;
                    var intersectionIndex = i == 0 ? startIntersectionIndex : goalIntersectionIndex;
                    const float threshold = 1e-3f;
                    foreach (var otherIndex in new[]
                                 { roadStartIndex, roadEndIndex, anchorIndex, intersectionIndex })
                    {
                        if (otherIndex == -1 ||
                            !(Vector3.Distance(nodesList[otherIndex].position, nodesList[nodeIndex].position) <
                              threshold)) continue;
                        nodesList[nodeIndex].position += Vector3.one * threshold;
                        if (i == 0)
                            startDistanceAlongRoad += threshold;
                        else
                            goalDistanceAlongRoad += threshold;
                    }
                }

                var weightsWithStartAndGoal = new ExtendedTwoDimensionalNativeArray<float>(
                    _weightsNativeArray, 2, 2,
                    weights.Width + 2, weights.Height + 2, Allocator.TempJob);

                for (var i = 0; i < weightsWithStartAndGoal.Width; ++i)
                for (var j = 0; j < 2; ++j)
                    weightsWithStartAndGoal[i, j == 0 ? startIndex : goalIndex] =
                        weightsWithStartAndGoal[j == 0 ? startIndex : goalIndex, i] =
                            (nodesList[j == 0 ? startIndex : goalIndex].position - nodesList[i].position).magnitude *
                            roadSystem.DistanceFactor;

                // connect start and goal to roadStart and roadEnd, if they are on roads. Otherwise, connect them to the intersection.
                for (var i = 0; i < 2; ++i)
                {
                    var nodeIndex = i == 0 ? startIndex : goalIndex;
                    if (i == 0 ? startRoad != null : goalRoad != null)
                    {
                        var road = i == 0 ? startRoad : goalRoad;
                        var distanceAlongRoad = i == 0 ? startDistanceAlongRoad : goalDistanceAlongRoad;
                        var roadStartIndex = i == 0 ? startRoadStartIndex : goalRoadStartIndex;
                        var roadEndIndex = i == 0 ? startRoadEndIndex : goalRoadEndIndex;
                        var roadLength = road.GetLength();
                        if (roadStartIndex != -1)
                            weightsWithStartAndGoal[roadStartIndex, nodeIndex] =
                                weightsWithStartAndGoal[nodeIndex, roadStartIndex] = distanceAlongRoad;
                        if (roadEndIndex != -1)
                            weightsWithStartAndGoal[roadEndIndex, nodeIndex] =
                                weightsWithStartAndGoal[nodeIndex, roadEndIndex] = roadLength - distanceAlongRoad;
                    }
                    else
                    {
                        var anchorIndex = i == 0 ? startAnchorIndex : goalAnchorIndex;
                        var intersectionIndex = i == 0 ? startIntersectionIndex : goalIntersectionIndex;
                        if (anchorIndex != -1)
                            weightsWithStartAndGoal[anchorIndex, nodeIndex] =
                                weightsWithStartAndGoal[nodeIndex, anchorIndex] =
                                    Vector3.Distance(nodesList[anchorIndex].position, nodesList[nodeIndex].position);
                        if (intersectionIndex != -1)
                            weightsWithStartAndGoal[intersectionIndex, nodeIndex] =
                                weightsWithStartAndGoal[nodeIndex, intersectionIndex] =
                                    Vector3.Distance(nodesList[intersectionIndex].position,
                                        nodesList[nodeIndex].position);
                    }
                }

                if (startRoad != null && startRoad == goalRoad)
                {
                    // start and goal are along the same road, so they themselves should be connected.
                    weightsWithStartAndGoal[startIndex, goalIndex] = weightsWithStartAndGoal[goalIndex, startIndex] =
                        Mathf.Abs(startDistanceAlongRoad - goalDistanceAlongRoad);
                }
                else if (startAnchor != null && startAnchor == goalAnchor)
                {
                    // start and goal are along the same line between an anchor and the corresponding intersection
                    // origin, so they themselves should be connected.
                    weightsWithStartAndGoal[startIndex, goalIndex] = weightsWithStartAndGoal[goalIndex, startIndex] =
                        Vector3.Distance(nodesList[startIndex].position, nodesList[goalIndex].position);
                }

                // For debugging purposes.
                if (edges != null)
                {
                    edges.Clear();

                    for (var i = 0; i < nodesList.Count; ++i)
                    for (var j = 0; j < nodesList.Count; ++j)
                    {
                        if (!(weightsWithStartAndGoal[i, j] < 5e3) || !(weightsWithStartAndGoal[i, j] > 1e-3)) continue;
                        var edge = new Edge
                        {
                            start = roadSystem.transform.TransformPoint(nodesList[i].position),
                            end = roadSystem.transform.TransformPoint(nodesList[j].position),
                            cost = weightsWithStartAndGoal[i, j]
                        };
                        edges.Add(edge);
                    }
                }

                var nativeNodes = new NativeArray<float3>(nodesList.Count, Allocator.TempJob);
                for (var i = 0; i < nodesList.Count; ++i)
                    nativeNodes[i] = nodesList[i].position;

                var pathIndices = AStar.FindShortestPath(nativeNodes, weightsWithStartAndGoal, startIndex, goalIndex,
                    out stepsTaken, AStar.DistanceHeuristic);

                nativeNodes.Dispose();
                weightsWithStartAndGoal.Dispose();

                var path = new List<Node>(pathIndices.Length);
                path.AddRange(pathIndices.Select(t => nodesList[t]));

                return path;
            }
        }
    }
}
