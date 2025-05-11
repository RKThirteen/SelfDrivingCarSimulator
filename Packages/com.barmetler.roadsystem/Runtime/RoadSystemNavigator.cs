using System.Collections.Generic;
using Barmetler.RoadSystem.Util;
using Unity.Profiling;
using UnityEngine;

namespace Barmetler.RoadSystem
{
    using PointList = List<Bezier.OrientedPoint>;

    public class RoadSystemNavigator : MonoBehaviour
    {
        public RoadSystem currentRoadSystem;

        public Vector3 Goal = Vector3.zero;

        public float GraphStepSize = 1;
        public float MinDistanceYScale = 1;
        public float MinDistanceToRoadToConnect = 10;

        public PointList CurrentPoints { private set; get; } = new PointList();
        private AsyncUpdater<PointList> _currentPoints;

        private void Awake()
        {
            _currentPoints = new AsyncUpdater<PointList>(this, GetNewWayPoints, new PointList(), 1f / 144);
        }

        private void Update()
        {
            _currentPoints.Update();
        }

        private void FixedUpdate()
        {
            var points = _currentPoints.GetData();
            if (points != CurrentPoints)
            {
                CurrentPoints = points;
                RemovePointsAhead();
            }

            RemovePointsBehind();
        }

        public float GetMinDistance(out Road road, out Vector3 closestPoint, out float distanceAlongRoad)
        {
            if (!currentRoadSystem)
            {
                road = null;
                closestPoint = Vector3.zero;
                distanceAlongRoad = 0;
                return float.PositiveInfinity;
            }

            return currentRoadSystem.GetMinDistance(transform.position, Mathf.Max(0.1f, GraphStepSize),
                MinDistanceYScale, out road, out closestPoint, out distanceAlongRoad);
        }

        public float GetMinDistance(
            out Intersection intersection, out RoadAnchor anchor, out Vector3 closestPoint, out float distanceAlongRoad)
        {
            if (!currentRoadSystem)
            {
                intersection = null;
                anchor = null;
                closestPoint = Vector3.zero;
                distanceAlongRoad = 0;
                return float.PositiveInfinity;
            }

            return currentRoadSystem.GetMinDistance(
                transform.position, MinDistanceYScale, out intersection, out anchor, out closestPoint,
                out distanceAlongRoad);
        }

        private void RemovePointsBehind()
        {
            var pos = transform.position;
            var count = 0;
            for (; count < CurrentPoints.Count - 1; ++count)
            {
                // if next point is further away, stop (but don't stop if current point is really close)
                var sqrDst = (CurrentPoints[count].position - pos).sqrMagnitude;
                if (
                    sqrDst < (CurrentPoints[count + 1].position - pos).sqrMagnitude &&
                    sqrDst > GraphStepSize / 2 * GraphStepSize / 2
                ) break;
            }

            if (count > 0)
            {
                CurrentPoints.RemoveRange(0, count);
            }
        }

        private void RemovePointsAhead()
        {
            var pos = Goal;
            var count = 0;
            for (; count < CurrentPoints.Count - 1; ++count)
            {
                // if next point is further away, stop (but don't stop if current point is really close)
                var sqrDst = (CurrentPoints[CurrentPoints.Count - 1 - count].position - pos).sqrMagnitude;
                if (
                    sqrDst < (CurrentPoints[CurrentPoints.Count - 1 - count - 1].position - pos).sqrMagnitude &&
                    sqrDst > GraphStepSize / 2 * GraphStepSize / 2
                ) break;
            }

            if (count > 0)
            {
                CurrentPoints.RemoveRange(CurrentPoints.Count - count, count);
            }
        }

        public void CalculateWayPointsSync()
        {
            CurrentPoints = GetNewWayPoints();
        }

        private static readonly ProfilerMarker GetNewWayPointsPerfMarker =
            new ProfilerMarker("RoadSystemNavigator.cs GetNewWayPoints");

        private PointList GetNewWayPoints()
        {
            using var marker = GetNewWayPointsPerfMarker.Auto();
            return currentRoadSystem.FindPath(
                transform.position, Goal,
                yScale: MinDistanceYScale,
                stepSize: Mathf.Max(0.1f, GraphStepSize),
                minDstToRoadToConnect: MinDistanceToRoadToConnect
            );
        }
    }
}
