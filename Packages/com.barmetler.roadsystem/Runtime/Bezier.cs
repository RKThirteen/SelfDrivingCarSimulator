using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace Barmetler.RoadSystem
{
    public static class Bezier
    {
        public static Vector3 EvaluateQuadratic(Vector3 a, Vector3 b, Vector3 c, float t)
        {
            var p0 = Vector3.Lerp(a, b, t);
            var p1 = Vector3.Lerp(b, c, t);
            return Vector3.Lerp(p0, p1, t);
        }

        public static Vector3 EvaluateCubic(Vector3 a, Vector3 b, Vector3 c, Vector3 d, float t)
        {
            // Bernstein polynomials
            return a * ((1 - t) * (1 - t) * (1 - t)) +
                   b * (3 * (1 - t) * (1 - t) * t) +
                   c * (3 * (1 - t) * t * t) +
                   d * (t * t * t);
        }

        public static Vector3 DeriveQuadratic(Vector3 a, Vector3 b, Vector3 c, float t)
        {
            return Vector3.Lerp(2 * (b - a), 2 * (c - b), t);
        }

        public static Vector3 DeriveCubic(Vector3 a, Vector3 b, Vector3 c, Vector3 d, float t)
        {
            return EvaluateQuadratic(3 * (b - a), 3 * (c - b), 3 * (d - c), t);
        }

        /// <summary>
        /// Approximate the inverse of a cubic bezier curve.
        /// </summary>
        public static float InverseCubic(Vector3 a, Vector3 b, Vector3 c, Vector3 d, Vector3 p)
        {
            var t = 0.5f;
            for (var i = 0; i < 100; ++i)
            {
                var p2 = EvaluateCubic(a, b, c, d, t);
                var dp = DeriveCubic(a, b, c, d, t);
                var delta = p2 - p;
                var dot = Vector3.Dot(delta, dp);
                if (dot == 0) // Usually happens at i ~ 3
                    break;
                t -= Vector3.Dot(delta, dp) / Vector3.Dot(dp, dp);
            }

            return t;
        }

        /// <summary>
        /// Subdivide a cubic bezier curve.
        /// </summary>
        /// <returns>7 points (2 segments)</returns>
        public static Vector3[] SubdivideCubic(Vector3 a, Vector3 b, Vector3 c, Vector3 d, float t)
        {
            var ab = Vector3.Lerp(a, b, t);
            var bc = Vector3.Lerp(b, c, t);
            var cd = Vector3.Lerp(c, d, t);
            var abbc = Vector3.Lerp(ab, bc, t);
            var bccd = Vector3.Lerp(bc, cd, t);
            var abbcBccd = Vector3.Lerp(abbc, bccd, t);
            return new[] { a, ab, abbc, abbcBccd, bccd, cd, d };
        }

        // TODO: not accurate enough, but better than nothing
        public static Vector3[] UnSubdivideCubic(
            Vector3 p0, Vector3 h01,
            Vector3 h10, Vector3 p1, Vector3 h11,
            Vector3 h20, Vector3 p2
        )
        {
            return new[]
            {
                p0,
                p0 + Vector3.Distance(p0, h10) / Vector3.Distance(p0, h01) * (h01 - p0),
                p2 + Vector3.Distance(p2, h11) / Vector3.Distance(p2, h20) * (h20 - p2),
                p2
            };
        }

        /// <summary>
        /// Position and Direction Vectors.
        /// </summary>
        public struct OrientedPoint
        {
            public Vector3 position;
            public Vector3 forward;
            public Vector3 normal;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public OrientedPoint(Vector3 p, Vector3 f, Vector3 n)
            {
                position = p;
                forward = f;
                normal = n;
            }

            public OrientedPoint ToWorldSpace(Transform transform)
            {
                var p = transform.TransformPoint(position);
                var f = transform.TransformDirection(forward);
                var n = transform.TransformDirection(normal);
                return new OrientedPoint(p, f, n);
            }

            public OrientedPoint ToLocalSpace(Transform transform)
            {
                var p = transform.InverseTransformPoint(position);
                var f = transform.InverseTransformDirection(forward);
                var n = transform.InverseTransformDirection(normal);
                return new OrientedPoint(p, f, n);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static OrientedPoint[] GetEvenlySpacedPoints(
            IEnumerable<Vector3> points, IEnumerable<Vector3> normals, float spacing, float resolution = 1)
        {
            return GetEvenlySpacedPoints(points, normals, out _, null, spacing, resolution);
        }

        public static OrientedPoint[] GetEvenlySpacedPoints(
            IEnumerable<Vector3> points, IEnumerable<Vector3> normals, out Bounds bounds, List<Bounds> boundingBoxes,
            float spacing, float resolution = 1)
        {
            var job = new GetEvenlySpacedPointsBurstJob
            {
                Points = new NativeArray<Vector3>(points.ToArray(), Allocator.TempJob),
                Normals = new NativeArray<Vector3>(normals.ToArray(), Allocator.TempJob),
                Spacing = spacing,
                Resolution = resolution,
                Result = new NativeList<OrientedPoint>(Allocator.TempJob),
                Bounds = new NativeArray<Bounds>(1, Allocator.TempJob),
                BoundingBoxes = new NativeList<Bounds>(Allocator.TempJob),
                LineLength = new NativeArray<float>(1, Allocator.TempJob)
            };
            job.Run();
            var result = job.Result.AsArray().ToArray();
            bounds = job.Bounds[0];
            boundingBoxes?.Clear();
            boundingBoxes?.AddRange(job.BoundingBoxes.AsArray().ToArray());
            job.Points.Dispose();
            job.Normals.Dispose();
            job.Result.Dispose();
            job.Bounds.Dispose();
            job.BoundingBoxes.Dispose();
            job.LineLength.Dispose();
            return result;
        }

        [BurstCompile(CompileSynchronously = true)]
        private struct GetEvenlySpacedPointsBurstJob : IJob
        {
            [ReadOnly]
            public NativeArray<Vector3> Points;

            [ReadOnly]
            public NativeArray<Vector3> Normals;

            [ReadOnly]
            public float Spacing;

            [ReadOnly]
            public float Resolution;

            public NativeList<OrientedPoint> Result;

            /// <summary>
            /// 1-element array, in order to box the data as Bounds is a value type.
            /// </summary>
            public NativeArray<Bounds> Bounds;

            public NativeList<Bounds> BoundingBoxes;

            /// <summary>
            /// 1-element array, in order to box the data.
            /// </summary>
            public NativeArray<float> LineLength;

            private int _numPoints;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private int LoopIndex(int i)
            {
                return (i % _numPoints + _numPoints) % _numPoints;
            }

            private struct Segment
            {
                public float3 p0, p1, p2, p3;

                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                public Segment(float3 p0, float3 p1, float3 p2, float3 p3)
                {
                    this.p0 = p0;
                    this.p1 = p1;
                    this.p2 = p2;
                    this.p3 = p3;
                }

                public float3 this[int i] => i switch
                {
                    0 => p0,
                    1 => p1,
                    2 => p2,
                    3 => p3,
                    _ => throw new ArgumentOutOfRangeException()
                };
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private Segment GetPointsInSegment(int i)
            {
                return new Segment(Points[i * 3], Points[i * 3 + 1], Points[i * 3 + 2], Points[LoopIndex(i * 3 + 3)]);
            }

            public void Execute()
            {
                _numPoints = Points.Length;
                var numSegments = _numPoints / 3;
                if (Normals.Length < numSegments + 1)
                    throw new ArgumentException("not enough normals!");

                var boundsValue = Bounds[0];
                boundsValue.min = Vector3.positiveInfinity;
                boundsValue.max = Vector3.negativeInfinity;
                BoundingBoxes.Clear();

                LineLength[0] = 0;

                var previousPoint = Points[0] - (Points[1] - Points[0]).normalized * Spacing;
                float dstSinceLastEvenPoint = 0;

                for (var segment = 0; segment < numSegments; ++segment)
                {
                    var segmentBounds = new Bounds
                    {
                        min = Vector3.positiveInfinity,
                        max = Vector3.negativeInfinity
                    };

                    var p = GetPointsInSegment(segment);

                    var normalOnCurve = Normals[segment];

                    // Initialize bounding box
                    segmentBounds.Encapsulate(p[0]);
                    segmentBounds.Encapsulate(p[3]);

                    var previousPointOnCurve = p[0];
                    float segmentLength = 0;

                    var controlNetLength = Vector3.Distance(p[0], p[1]) + Vector3.Distance(p[1], p[2]) +
                                           Vector3.Distance(p[2], p[3]);
                    var estimatedCurveLength = Vector3.Distance(p[0], p[3]) + 0.5f * controlNetLength;
                    var divisions = Mathf.CeilToInt(estimatedCurveLength * Resolution * 10);
                    if (divisions > 0)
                    {
                        var startIndex = Result.Length;
                        var t = startIndex == 0 ? -1f / divisions : 0;
                        Vector3 forwardOnCurve;
                        while (t <= 1)
                        {
                            t += 1f / divisions;
                            var pointOnCurve = EvaluateCubic(p[0], p[1], p[2], p[3], t);
                            if (t > -0.5f / divisions)
                                segmentLength += Vector3.Distance(pointOnCurve, previousPointOnCurve);
                            previousPointOnCurve = pointOnCurve;
                            forwardOnCurve = DeriveCubic(p[0], p[1], p[2], p[3], Mathf.Clamp01(t)).normalized;
                            normalOnCurve = Vector3.Cross(forwardOnCurve, Vector3.Cross(normalOnCurve, forwardOnCurve))
                                .normalized;
                            dstSinceLastEvenPoint += Vector3.Distance(previousPoint, pointOnCurve);

                            while (dstSinceLastEvenPoint >= Spacing)
                            {
                                var overshootDst = dstSinceLastEvenPoint - Spacing;
                                var newEvenlySpacedPoint =
                                    pointOnCurve + (previousPoint - pointOnCurve).normalized * overshootDst;

                                // Update bounding box
                                segmentBounds.Encapsulate(newEvenlySpacedPoint);

                                Result.Add(new OrientedPoint(newEvenlySpacedPoint, forwardOnCurve, normalOnCurve));

                                dstSinceLastEvenPoint = overshootDst;
                                previousPoint = newEvenlySpacedPoint;
                            }

                            previousPoint = pointOnCurve;
                        }

                        var endIndexExclusive = Result.Length;

                        if (startIndex != endIndexExclusive)
                        {
                            segmentLength += Vector3.Distance(previousPointOnCurve, p[3]);
                            LineLength[0] += segmentLength;

                            forwardOnCurve = DeriveCubic(p[0], p[1], p[2], p[3], 1).normalized;
                            normalOnCurve = Vector3.Cross(forwardOnCurve, Vector3.Cross(normalOnCurve, forwardOnCurve))
                                .normalized;
                            var angleError = Vector3.SignedAngle(normalOnCurve, Normals[segment + 1], forwardOnCurve);

                            // Iterate over evenly spaced points in this segment, and gradually correct angle error
                            var tStep = Spacing / segmentLength;
                            var tStart = Vector3.Distance(Result[startIndex].position, p[0]) / segmentLength;
                            for (var i = startIndex; i < endIndexExclusive; ++i)
                            {
                                var t_ = (i - startIndex) * tStep + tStart;
                                // TODO: make weight non-linear, depending on handle lengths
                                var correction = t_ * angleError;
                                var element = Result[i];
                                element.normal = Quaternion.AngleAxis(correction, element.forward) * element.normal;
                                Result[i] = element;
                            }
                        }
                    }

                    if (segment == 0) boundsValue = segmentBounds;
                    else boundsValue.Encapsulate(segmentBounds);
                    BoundingBoxes.Add(segmentBounds);
                }

                if (Result.Length <= 0) goto end_cleanup;
                var start = Result[0];
                start.position = Points[0];
                start.normal = Normals[0];
                start.forward = DeriveCubic(Points[0], Points[1], Points[2], Points[3], 0).normalized;
                Result[0] = start;
                if (Result.Length == 1)
                {
                    Result.Add(new OrientedPoint(
                        Points[LoopIndex(-1)],
                        DeriveCubic(Points[LoopIndex(-4)], Points[LoopIndex(-3)], Points[LoopIndex(-2)],
                            Points[LoopIndex(-1)], 1).normalized,
                        Normals[Normals.Length - 1]
                    ));
                    if (BoundingBoxes.Length > 0)
                        BoundingBoxes[BoundingBoxes.Length - 1].Encapsulate(Points[LoopIndex(-1)]);
                    boundsValue.Encapsulate(Points[LoopIndex(-1)]);
                    goto end_cleanup;
                }

                var end = Result[Result.Length - 1];
                end.position = Points[LoopIndex(-1)];
                end.normal = Normals[Normals.Length - 1];
                end.forward = DeriveCubic(Points[LoopIndex(-4)], Points[LoopIndex(-3)],
                    Points[LoopIndex(-2)], Points[LoopIndex(-1)], 1).normalized;
                Result[Result.Length - 1] = end;
                end_cleanup:
                Bounds[0] = boundsValue;
            }
        }

        public static float AngleFromNormal(Vector3 forward, Vector3 normal)
        {
            forward = forward.normalized;
            normal = normal.normalized;
            normal = (normal - Vector3.Dot(forward, normal) * forward).normalized;
            var right = Vector3.Cross(Vector3.up, forward).normalized;
            var up = Vector3.Cross(forward, right).normalized;
            return Vector3.SignedAngle(normal, up, forward);
        }

        public static Vector3 NormalFromAngle(Vector3 forward, float angle)
        {
            var right = Vector3.Cross(Vector3.up, forward).normalized;
            var up = Vector3.Cross(forward, right).normalized;
            return Quaternion.AngleAxis(-angle, forward) * up;
        }
    }
}
