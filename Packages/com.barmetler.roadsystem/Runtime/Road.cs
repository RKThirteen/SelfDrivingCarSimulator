using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using Barmetler.RoadSystem.Util;
using Unity.Profiling;
using UnityEngine;

namespace Barmetler.RoadSystem
{
    public class Road : MonoBehaviour
    {
        public RoadAnchor start;
        public RoadAnchor end;

        [SerializeField, HideInInspector]
        private bool autoSetControlPoints = false;

        [SerializeField, HideInInspector]
        private List<Vector3> points = new List<Vector3>();

        [SerializeField, HideInInspector]
        private List<Vector3> normals = new List<Vector3>();

        [SerializeField, HideInInspector]
        private List<float> angles = new List<float>();

        [SerializeField, HideInInspector]
        private Bounds bounds = new Bounds();

        [SerializeField, HideInInspector]
        private List<Bounds> boundingBoxes = new List<Bounds>();

        public Bounds BoundingBox => bounds;

        public List<Bounds> BoundingBoxes => boundingBoxes;

        public bool AutoSetControlPoints
        {
            get => autoSetControlPoints;
            set
            {
                if (autoSetControlPoints == value) return;
                autoSetControlPoints = value;
                if (autoSetControlPoints)
                {
                    AutoSetAllControlPoints();
                }
            }
        }

        public struct EvenlySpacedPointsContext
        {
            public EvenlySpacedPointsContext(float _spacing, float _resolution)
            {
                spacing = _spacing;
                resolution = _resolution;
            }

            public float spacing;
            public float resolution;

            [SuppressMessage("ReSharper", "CompareOfFloatsByEqualityOperator")]
            public override bool Equals(object obj)
            {
                return obj is EvenlySpacedPointsContext other && other.spacing == spacing &&
                       other.resolution == resolution;
            }

            public override int GetHashCode()
            {
                return $"{spacing}-{resolution}".GetHashCode();
            }
        }

        private readonly ContextDataCache<Bezier.OrientedPoint[], EvenlySpacedPointsContext> evenlySpacedPointsCache =
            new ContextDataCache<Bezier.OrientedPoint[], EvenlySpacedPointsContext>();

        private readonly ContextDataCache<float, EvenlySpacedPointsContext> lengthCache =
            new ContextDataCache<float, EvenlySpacedPointsContext>();

        public Vector3 this[int i]
        {
            get => points[LoopIndex(i)];
            private set => points[LoopIndex(i)] = value;
        }

        public int NumPoints => points.Count;

        public int NumSegments => points.Count / 3;

        public Road()
        {
            evenlySpacedPointsCache.children.Add(lengthCache);
        }

        public void Clear()
        {
            points.Clear();
            normals.Clear();
        }

        public void RefreshEndPoints(bool updatemesh = true)
        {
            if (start != null) start.SetRoad(this, true);
            if (end != null) end.SetRoad(this, false);

            // Convert to using normals:
            if (angles.Count == NumSegments + 1)
            {
                normals.Clear();
                for (var i = 0; i < NumSegments + 1; ++i)
                {
                    var forward = i == 0 ? (this[1] - this[0]).normalized : (this[i] - this[i - 1]).normalized;
                    normals.Add(Bezier.NormalFromAngle(forward, angles[i]));
                }

                angles.Clear();
            }

            if (points.Count == 0 || normals.Count == 0)
            {
                points = new List<Vector3>
                    { new Vector3(0, 0, 0), new Vector3(0, 0, 1), new Vector3(0, 0, 3), new Vector3(0, 0, 4) };
                normals = new List<Vector3> { Vector3.up, Vector3.up };

                if (start != null)
                {
                    points[0] = transform.InverseTransformPoint(start.transform.position);
                    normals[0] = transform.InverseTransformDirection(start.transform.up);
                }

                if (end != null)
                {
                    points[3] = transform.InverseTransformPoint(end.transform.position);
                    normals[1] = transform.InverseTransformDirection(end.transform.up);
                }

                if (start != null)
                {
                    points[1] = this[0] + transform.InverseTransformDirection(start.transform.forward) *
                        (this[3] - this[0]).magnitude / 2;
                }

                if (end != null)
                {
                    points[2] = this[3] + transform.InverseTransformDirection(end.transform.forward) *
                        (this[0] - this[3]).magnitude / 2;
                }

                if (autoSetControlPoints)
                    AutoSetAllControlPoints();

                OnCurveChanged(updatemesh);
            }
            else if (NumPoints > 1)
            {
                if (start != null)
                {
                    var a = this[0];
                    var b = this[1];
                    var n = normals[0];
                    var startLength = (this[1] - this[0]).magnitude;
                    this[0] = transform.InverseTransformPoint(start.transform.position);
                    this[1] = this[0] + transform.InverseTransformDirection(start.transform.forward) * startLength;
                    normals[0] = transform.InverseTransformDirection(start.transform.up);
                    if (a != this[0] || b != this[1] || n != normals[0])
                        OnCurveChanged(updatemesh);
                }

                if (end != null)
                {
                    var a = this[-1];
                    var b = this[-2];
                    var n = normals[normals.Count - 1];
                    var endLength = (this[-2] - this[-1]).magnitude;
                    this[-1] = transform.InverseTransformPoint(end.transform.position);
                    this[-2] = this[-1] + transform.InverseTransformDirection(end.transform.forward) * endLength;
                    normals[normals.Count - 1] = transform.InverseTransformDirection(end.transform.up);
                    if (a != this[-1] || b != this[-2] || n == normals[normals.Count - 1])
                        OnCurveChanged(updatemesh);
                }
            }
        }

        public Vector3[] GetPointsInSegment(int i)
        {
            return new Vector3[] { points[i * 3], points[i * 3 + 1], points[i * 3 + 2], points[LoopIndex(i * 3 + 3)] };
        }

        public void AppendSegment(Vector3 pos, bool isStart, Vector3 normal = default)
        {
            if (isStart && start != null) return;
            if (!isStart && end != null) return;

            if (normal == default(Vector3)) normal = Vector3.up;

            if (isStart)
            {
                points.InsertRange(0, new Vector3[] { pos, Vector3.zero, Vector3.zero });
                this[2] = this[3] - (this[4] - this[3]).normalized * (0.5f * (this[0] - this[3]).magnitude);
                this[1] = pos + 0.85f * (this[2] - this[0]);
                this[1] -= Vector3.Dot(normal, (this[1] - this[0])) * normal;
                var angle = Bezier.AngleFromNormal(this[0] - this[1], normal);
                normals.Insert(0, normal);
            }
            else
            {
                points.AddRange(new Vector3[] { Vector3.zero, Vector3.zero, pos });
                this[-3] = this[-4] - (this[-5] - this[-4]).normalized * (0.5f * (this[-1] - this[-4]).magnitude);
                this[-2] = pos + 0.85f * (this[-3] - this[-1]);
                this[-2] -= Vector3.Dot(normal, (this[-2] - this[-1])) * normal;
                var angle = Bezier.AngleFromNormal(this[-1] - this[-2], normal);
                normals.Add(normal);
            }

            if (autoSetControlPoints)
            {
                AutoSetAllAffectedControlPoints(isStart ? 0 : NumPoints - 1);
            }

            OnCurveChanged();
        }

        [Obsolete("Use InsertSegment(Vector3, int) instead!")]
        public void InsertSegment(Vector3 pos, int segmentIndex)
        {
            points.InsertRange(segmentIndex * 3 + 2, new Vector3[] { Vector3.zero, pos, Vector3.zero });
            normals.Insert(segmentIndex + 1, Vector3.up);

            if (autoSetControlPoints)
            {
                AutoSetAllAffectedControlPoints(segmentIndex * 3 + 3);
            }
            else
            {
                AutoSetAnchorControlPoints(segmentIndex * 3 + 3);
            }

            OnCurveChanged();
        }

        public void InsertSegment(int segmentIndex, float t, Vector3 normal)
        {
            if (points.Count < 4) return;
            var segment = GetPointsInSegment(segmentIndex);
            var newSegments = Bezier.SubdivideCubic(segment[0], segment[1], segment[2], segment[3], t);
            points.RemoveRange(segmentIndex * 3, 4);
            points.InsertRange(segmentIndex * 3, newSegments);
            normals.Insert(segmentIndex + 1, normal);
            OnCurveChanged();
        }

        public void DeleteAnchor(int anchorIndex)
        {
            if (anchorIndex % 3 != 0) return;
            if (NumSegments > 1)
            {
                if (anchorIndex == 0 && start == null)
                {
                    points.RemoveRange(0, 3);
                    normals.RemoveRange(0, 1);
                }
                else if (anchorIndex == NumPoints - 1 && end == null)
                {
                    points.RemoveRange(anchorIndex - 2, 3);
                    normals.RemoveRange(anchorIndex / 3, 1);
                }
                else if (anchorIndex > 0 && anchorIndex < NumPoints - 1)
                {
                    normals.RemoveAt(anchorIndex / 3);
                    var unSubdivided = Bezier.UnSubdivideCubic(
                        points[anchorIndex - 3],
                        points[anchorIndex - 2],
                        points[anchorIndex - 1],
                        points[anchorIndex],
                        points[anchorIndex + 1],
                        points[anchorIndex + 2],
                        points[anchorIndex + 3]);
                    points.RemoveRange(anchorIndex - 1, 3);
                    points[anchorIndex - 2] = unSubdivided[1];
                    points[anchorIndex - 1] = unSubdivided[2];
                }
            }

            OnCurveChanged();
        }

        public void MovePoint(int i, Vector3 pos)
        {
            var oldPos = this[i];

            if (i % 3 == 0)
            {
                if ((start != null && i == 0) || (end != null && i == NumPoints - 1)) return;
                if (i > 0) this[i - 1] += pos - oldPos;
                if (i < NumPoints - 1) this[i + 1] += pos - oldPos;
                this[i] = pos;

                if (autoSetControlPoints)
                    AutoSetAllAffectedControlPoints(i);
            }
            else
            {
                if (autoSetControlPoints) return;

                var nextIsAnchor = (i + 1) % 3 == 0;
                var correspondingIndex = nextIsAnchor ? i + 2 : i - 2;
                var anchor = this[nextIsAnchor ? i + 1 : i - 1];

                if ((start != null && i == 1) || (end != null && i == NumPoints - 2))
                {
                    var isStart = i == 1;
                    var forward = transform.InverseTransformDirection((isStart ? start : end).transform.forward);
                    this[i] = this[isStart ? 0 : (NumPoints - 1)] +
                              Mathf.Max(1e-1f, Vector3.Dot(pos - this[isStart ? 0 : (NumPoints - 1)], forward))
                              * forward;
                }
                else if (i > 1 && i < NumPoints - 2)
                {
                    var correspondingDistance = (this[correspondingIndex] - anchor).magnitude;
                    this[i] = pos;
                    var direction = (pos - anchor).normalized;
                    this[correspondingIndex] = anchor - direction * correspondingDistance;
                }
                else
                {
                    this[i] = pos;
                }

                FixNormal((i + 1) / 3);
            }

            OnCurveChanged();
        }

        private void FixNormal(int index)
        {
            var forward = index == 0
                ? (this[1] - this[0]).normalized
                : (this[index * 3] - this[index * 3 - 1]).normalized;
            normals[index] = Vector3.ProjectOnPlane(normals[index], forward).normalized;
        }

        private void FixNormals()
        {
            for (var i = 0; i <= NumSegments; ++i)
                FixNormal(i);
        }

        [Obsolete("Use MoveNormal instead!")]
        public void MoveAngle(int i, float angle)
        {
            if ((i == 0 && start != null) || (i == normals.Count - 1 && end != null)) return;
            var forward = i == 0 ? points[1] - points[0] : points[3 * i] - points[3 * i - 1];
            MoveNormal(i, Bezier.NormalFromAngle(forward, angle));

            OnCurveChanged();
        }

        [Obsolete("Use GetNormal instead!")]
        public float GetAngle(int i)
        {
            var forward = i == 0 ? points[1] - points[0] : points[3 * i] - points[3 * i - 1];
            return Bezier.AngleFromNormal(forward, normals[i]);
        }

        public void MoveNormal(int i, Vector3 normal)
        {
            if ((i == 0 && start != null) || (i == normals.Count - 1 && end != null)) return;
            normals[i] = normal;
            FixNormal(i);

            OnCurveChanged();
        }

        public Vector3 GetNormal(int i)
        {
            return normals[i];
        }

        public void OnValidate()
        {
            RefreshEndPoints(false);
            if (start != null) start.SetRoad(this, true);
            if (end != null) end.SetRoad(this, false);
        }

        #region Automatic Control Point Functions

        private void AutoSetAllAffectedControlPoints(int updatedAnchorIndex)
        {
            for (var i = updatedAnchorIndex - 3; i <= updatedAnchorIndex + 3; i += 3)
            {
                if (i >= 0 && i < NumPoints)
                    AutoSetAnchorControlPoints(i);
            }

            AutoSetStartAndEndControls();

            OnCurveChanged();
        }

        public void AutoSetAllControlPoints()
        {
            for (var i = 0; i < NumPoints; i += 3)
            {
                AutoSetAnchorControlPoints(i);
            }

            AutoSetStartAndEndControls();

            OnCurveChanged();
        }

        private void AutoSetAnchorControlPoints(int anchorIndex)
        {
            var anchorPos = this[anchorIndex];
            var direction = Vector3.zero;
            var neighborDistances = new float[2];
            if (anchorIndex - 3 >= 0)
            {
                var offset = this[anchorIndex - 3] - anchorPos;
                direction += offset.normalized;
                neighborDistances[0] = offset.magnitude;
            }

            if (anchorIndex + 3 <= NumPoints - 1)
            {
                var offset = this[anchorIndex + 3] - anchorPos;
                direction -= offset.normalized;
                neighborDistances[1] = -offset.magnitude;
            }

            direction.Normalize();

            for (var i = 0; i < 2; ++i)
            {
                var controlIndex = anchorIndex + i * 2 - 1;
                if (controlIndex >= 0 && controlIndex < NumPoints)
                {
                    this[controlIndex] = anchorPos + direction * (neighborDistances[i] * 0.5f);
                }
            }

            FixNormal(anchorIndex / 3);

            OnCurveChanged();
        }

        private void AutoSetStartAndEndControls()
        {
            if (start == null)
                this[1] = (this[0] + this[2]) * 0.5f;
            else
                this[1] = this[0] + transform.InverseTransformDirection(start.transform.forward) *
                    ((this[0] - this[3]) * 0.5f).magnitude;

            if (end == null)
                this[-2] = (this[-1] + this[-3]) * 0.5f;
            else
                this[-2] = this[-1] + transform.InverseTransformDirection(end.transform.forward) *
                    ((this[-1] - this[-4]) * 0.5f).magnitude;

            OnCurveChanged();
        }

        // end Auto Control Points

        #endregion

        public Bezier.OrientedPoint[] GetEvenlySpacedPoints(float spacing, float resolution = 1)
        {
            CalculateEvenlySpacedPoints(spacing, resolution);
            return evenlySpacedPointsCache.GetData(new EvenlySpacedPointsContext(spacing, resolution));
        }

        private static readonly ProfilerMarker CalculateEvenlySpacedPointsPerfMarker =
            new ProfilerMarker("Road.cs CalculateEvenlySpacedPoints");

        private void CalculateEvenlySpacedPoints(float spacing, float resolution = 1,
            bool calculateBoundingBoxes = false)
        {
            var context = new EvenlySpacedPointsContext(spacing, resolution);
            if (evenlySpacedPointsCache.IsValid(context) && !calculateBoundingBoxes) return;

            Bezier.OrientedPoint[] result;

            using (CalculateEvenlySpacedPointsPerfMarker.Auto())
            {
                if (calculateBoundingBoxes)
                {
                    result = Bezier.GetEvenlySpacedPoints(points, normals, out bounds, boundingBoxes, spacing,
                        resolution);
                }
                else
                {
                    result = Bezier.GetEvenlySpacedPoints(points, normals, spacing, resolution);
                }
            }

            evenlySpacedPointsCache.SetData(result, context);
        }

        public float GetLength(float spacing = 1, float resolution = 1)
        {
            var context = new EvenlySpacedPointsContext(spacing, resolution);
            if (lengthCache.IsValid(context))
                return lengthCache.GetData(context);

            var points = GetEvenlySpacedPoints(spacing, resolution);

            float length = 0;
            var lastPoint = Vector3.zero;
            for (var i = 0; i < points.Length; ++i)
            {
                if (i > 0) length += (points[i].position - lastPoint).magnitude;
                lastPoint = points[i].position;
            }

            lengthCache.SetData(length, context);
            return length;
        }

        // to be used before GetMinDistance
        public bool IsMaybeCloser(Vector3 worldPosition, float minDistance, float yScale)
        {
            float sqrDst;
            var localPos = transform.InverseTransformPoint(worldPosition);

            // Check overall bounding box
            sqrDst = bounds.SqrDistance(Vector3.Scale(localPos, new Vector3(1, yScale, 1)));
            if (!bounds.Contains(Vector3.Scale(localPos, new Vector3(1, yScale, 1))) &&
                sqrDst >= minDistance * minDistance) return false;

            // calculate min distance to all bounding boxes
            sqrDst = float.PositiveInfinity;
            foreach (var bounds in boundingBoxes)
            {
                if (bounds.Contains(new Vector3(localPos.x, bounds.center.y + yScale * (localPos.y - bounds.center.y),
                        localPos.z)))
                    return true;
                sqrDst = Mathf.Min(sqrDst,
                    bounds.SqrDistance(new Vector3(localPos.x,
                        bounds.center.y + yScale * (localPos.y - bounds.center.y), localPos.z)));
            }

            // check min distance to all bounding boxes
            if (sqrDst >= minDistance * minDistance) return false;

            return true;
        }

        public float GetMinDistance(Vector3 worldPosition, float stepSize, float yScale, out Vector3 closestPoint,
            out float distanceAlongRoad)
        {
            float currDistAlongRoad = 0;
            var localPos = transform.InverseTransformPoint(worldPosition);
            var closestPointLocal = Vector3.zero;

            distanceAlongRoad = 0;
            var minDst = float.PositiveInfinity;

            var points = GetEvenlySpacedPoints(stepSize);

            for (var i = 0; i < points.Length; ++i)
            {
                var point = points[i];

                var dst = Vector3.Scale(point.position - localPos, new Vector3(1, yScale, 1)).magnitude;
                if (dst < minDst)
                {
                    var a = point.position;
                    Vector3 b;
                    float l;
                    var n = Vector3.zero;
                    var correct = false;
                    var backwards = false;
                    float along = 0;
                    if (i < points.Length - 1)
                    {
                        b = points[i + 1].position;
                        l = (b - a).magnitude;
                        n = (b - a).normalized;
                        float f;
                        if ((f = Vector3.Dot(localPos - a, n)) > 0 && f < l)
                        {
                            along = f;
                            correct = true;
                        }
                    }

                    if (i > 0 && !correct)
                    {
                        b = points[i - 1].position;
                        l = (b - a).magnitude;
                        n = (b - a).normalized;
                        float f;
                        if ((f = Vector3.Dot(localPos - a, n)) > 0 && f < l)
                        {
                            along = f;
                            correct = true;
                            backwards = true;
                        }
                    }

                    var pt = point.position;

                    if (correct)
                    {
                        pt = a + along * n;
                    }

                    dst = Vector3.Scale(pt - localPos, new Vector3(1, yScale, 1)).magnitude;
                    if (dst < minDst)
                    {
                        minDst = dst;
                        distanceAlongRoad = currDistAlongRoad + (backwards ? -along : along);
                        closestPointLocal = pt;
                    }
                }

                currDistAlongRoad += stepSize;
            }

            closestPoint = transform.TransformPoint(closestPointLocal);
            return minDst;
        }

        private static ProfilerMarker _roadMeshGeneratorPerfMarker = new ProfilerMarker("RoadMeshGenerator");

        public void OnCurveChanged(bool updateMesh = true)
        {
            evenlySpacedPointsCache.Invalidate();
            CalculateEvenlySpacedPoints(1, 1, true);

            if (GetComponent<RoadMeshGenerator>().Let(out var generator))
            {
                using (_roadMeshGeneratorPerfMarker.Auto())
                    generator.Invalidate(updateMesh);
            }
        }

        public int LoopIndex(int i)
        {
            return ((i % NumPoints) + NumPoints) % NumPoints;
        }
    }
}
