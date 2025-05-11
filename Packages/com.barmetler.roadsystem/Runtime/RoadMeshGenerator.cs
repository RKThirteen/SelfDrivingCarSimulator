using System;
using System.Diagnostics.CodeAnalysis;
using System.Linq;
using System.Runtime.InteropServices;
using Barmetler.RoadSystem.Util;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Profiling;
using UnityEngine;
using UnityEngine.Rendering;
using static Unity.Mathematics.math;

namespace Barmetler.RoadSystem
{
    [RequireComponent(typeof(Road)), RequireComponent(typeof(MeshFilter))]
    public class RoadMeshGenerator : MonoBehaviour
    {
        [Serializable]
        public class RoadMeshSettings
        {
            [Tooltip("Orientation of the Source Mesh")]
            public MeshConversion.MeshOrientation SourceOrientation = MeshConversion.MeshOrientation.Presets["BLENDER"];

            [Tooltip("By how much to displace uvs every time the mesh tiles")]
            public Vector2 uvOffset = Vector2.up;
        }

        [Tooltip("Settings regarding mesh generation")]
        public RoadMeshSettings settings;

        public bool AutoGenerate
        {
            get => autoGenerate;
            set
            {
                if (value)
                    GenerateRoadMesh();
                autoGenerate = value;
            }
        }

        [SerializeField, HideInInspector]
        private bool autoGenerate;

        [Tooltip("Drag the model to be used for mesh generation into this slot")]
        public MeshFilter SourceMesh;

        public bool Valid { private set; get; }

        private Road _road;
        private MeshFilter _mf;

        private void OnValidate()
        {
            _road = GetComponent<Road>();
            _mf = GetComponent<MeshFilter>();
        }

        private static ProfilerMarker _extractResultsMarker = new ProfilerMarker("Extract Results");
        private static ProfilerMarker _disposeMarker = new ProfilerMarker("Dispose");
        private static ProfilerMarker _setVerticesMarker = new ProfilerMarker("Set Vertices");
        private static ProfilerMarker _setIndicesMarker = new ProfilerMarker("Set Indices");
        private static ProfilerMarker _setUVsMarker = new ProfilerMarker("Set UVs");
        private static ProfilerMarker _recalculateNormalsMarker = new ProfilerMarker("Recalculate Normals");
        private static ProfilerMarker _recalculateTangentsMarker = new ProfilerMarker("Recalculate Tangents");
        private static ProfilerMarker _recalculateBoundsMarker = new ProfilerMarker("Recalculate Bounds");

        /// <summary>
        ///
        /// </summary>
        /// <param name="stepSize">
        /// distance between evenly spaced points for the spline.
        /// A bigger value results in straight sections of road.
        /// </param>
        public void GenerateRoadMesh(float stepSize = 1)
        {
            if (!_road) _road = GetComponent<Road>();
            if (!_mf) _mf = GetComponent<MeshFilter>();
            if (!_road || !_mf) return;
            if (!SourceMesh) return;

            var points = _road
                .GetEvenlySpacedPoints(stepSize)
                .Select(p => new GenerateRoadMeshV2Job.OrientedPoint
                {
                    Position = p.position,
                    Forward = p.forward,
                    Normal = p.normal
                })
                .ToArray();

            var sourceMesh = SourceMesh.sharedMesh;
            var vertexAttributeEnumValues = (VertexAttribute[])Enum.GetValues(typeof(VertexAttribute));
            // contains a mapping from VertexAttribute to VertexAttributeDescriptor.
            // because the amount of VertexAttributes is small, we can use a NativeArray, functioning as a map.
            var sourceAttributes =
                new NativeArray<VertexAttributeDescriptor>(vertexAttributeEnumValues.Length, Allocator.TempJob);
            foreach (var attributeDescriptor in sourceMesh.GetVertexAttributes())
                sourceAttributes[(int)attributeDescriptor.attribute] = attributeDescriptor;
            using var sourceMeshDataArray = Mesh.AcquireReadOnlyMeshData(sourceMesh);
            var sourceBounds = sourceMesh.bounds;

            var resultMeshData = Mesh.AllocateWritableMeshData(1);
            using var resultBounds = new NativeArray<float3>(2, Allocator.TempJob);

            new GenerateRoadMeshV2Job
            {
                StepSize = stepSize,
                UVOffset = settings.uvOffset,
                Points = new NativeArray<GenerateRoadMeshV2Job.OrientedPoint>(points, Allocator.TempJob),
                SourceOrientation = settings.SourceOrientation,
                SourceMeshData = sourceMeshDataArray[0],
                SourceVertexAttributes = sourceAttributes,
                SourceBounds = sourceBounds,
                ResultMeshData = resultMeshData[0],
                ResultBounds = resultBounds
            }.Run();

            var resultMesh = new Mesh
            {
                name = "Road Mesh"
            };
            Mesh.ApplyAndDisposeWritableMeshData(resultMeshData, resultMesh);
            resultMesh.bounds = new Bounds { min = resultBounds[0], max = resultBounds[1] };
            _mf.mesh = resultMesh;
            if (GetComponent<MeshCollider>().Let(out var coll))
                coll.sharedMesh = _mf.sharedMesh;

            Valid = true;
        }

        [BurstCompile(CompileSynchronously = true)]
        private struct GenerateRoadMeshV2Job : IJob
        {
            public struct OrientedPoint
            {
                public float3 Position, Forward, Normal;
            }

            [ReadOnly]
            public float StepSize;

            [ReadOnly]
            public float2 UVOffset;

            [ReadOnly]
            [DeallocateOnJobCompletion]
            public NativeArray<OrientedPoint> Points;

            [ReadOnly]
            public MeshConversion.MeshOrientation SourceOrientation;

            [ReadOnly]
            public Mesh.MeshData SourceMeshData;

            [ReadOnly]
            [DeallocateOnJobCompletion]
            public NativeArray<VertexAttributeDescriptor> SourceVertexAttributes;

            [ReadOnly]
            public Bounds SourceBounds;

            public Mesh.MeshData ResultMeshData;

            [WriteOnly]
            public NativeArray<float3> ResultBounds;

            [SuppressMessage("ReSharper", "ForCanBeConvertedToForeach")]
            public void Execute()
            {
                using var sourceAttributeData = new VertexAttributeData(SourceMeshData, SourceVertexAttributes);

                // The last point is repositioned to the end of the bezier, so the length of the line is the
                // amount of segments - 1 + the length of the last segment.
                var bezierLength = Points.Length > 1
                    ? StepSize * (Points.Length - 2) +
                      length(Points[Points.Length - 2].Position - Points[Points.Length - 1].Position)
                    : 0;

                var meshLength = SourceOrientation.forward switch
                {
                    MeshConversion.MeshOrientation.AxisDirection.X_POSITIVE => SourceBounds.size.x,
                    MeshConversion.MeshOrientation.AxisDirection.X_NEGATIVE => SourceBounds.size.x,
                    MeshConversion.MeshOrientation.AxisDirection.Y_POSITIVE => SourceBounds.size.y,
                    MeshConversion.MeshOrientation.AxisDirection.Y_NEGATIVE => SourceBounds.size.y,
                    MeshConversion.MeshOrientation.AxisDirection.Z_POSITIVE => SourceBounds.size.z,
                    MeshConversion.MeshOrientation.AxisDirection.Z_NEGATIVE => SourceBounds.size.z,
                    _ => throw new ArgumentOutOfRangeException()
                };

                var meshMinZ = SourceOrientation.forward switch
                {
                    MeshConversion.MeshOrientation.AxisDirection.X_POSITIVE => SourceBounds.min.x,
                    MeshConversion.MeshOrientation.AxisDirection.X_NEGATIVE => SourceBounds.max.x,
                    MeshConversion.MeshOrientation.AxisDirection.Y_POSITIVE => SourceBounds.min.y,
                    MeshConversion.MeshOrientation.AxisDirection.Y_NEGATIVE => SourceBounds.max.y,
                    MeshConversion.MeshOrientation.AxisDirection.Z_POSITIVE => SourceBounds.min.z,
                    MeshConversion.MeshOrientation.AxisDirection.Z_NEGATIVE => SourceBounds.max.z,
                    _ => throw new ArgumentOutOfRangeException()
                };

                var copyCount = (int)ceil(bezierLength / meshLength);
                var sourceVertexCount = SourceMeshData.vertexCount;
                var subMeshCount = SourceMeshData.subMeshCount;
                var guessedResultVertexCount = copyCount * sourceVertexCount;

                var sourceIndices = new IndexLists<ushort>(subMeshCount, Allocator.Temp);
                for (var subMeshIndex = 0; subMeshIndex < subMeshCount; ++subMeshIndex)
                {
                    var subMesh = SourceMeshData.GetSubMesh(subMeshIndex);
                    ref var sourceSubMeshIndices = ref sourceIndices[subMeshIndex];
                    sourceSubMeshIndices.ResizeUninitialized(subMesh.indexCount);
                    SourceMeshData.GetIndices(sourceSubMeshIndices.AsArray(), subMeshIndex);
                    if (SourceOrientation.isRightHanded)
                    {
                        for (var i = 0; i < sourceSubMeshIndices.Length; i += 3)
                        {
                            (sourceSubMeshIndices[i], sourceSubMeshIndices[i + 2]) =
                                (sourceSubMeshIndices[i + 2], sourceSubMeshIndices[i]);
                        }
                    }
                }

                var positions = new NativeList<float3>(Allocator.Temp);
                positions.ResizeUninitialized(guessedResultVertexCount);
                var normals = new NativeList<float3>(Allocator.Temp);
                normals.ResizeUninitialized(guessedResultVertexCount);
                var tangents = new NativeList<float4>(Allocator.Temp);
                tangents.ResizeUninitialized(guessedResultVertexCount);
                var uvs = new NativeList<float2>(Allocator.Temp);
                uvs.ResizeUninitialized(guessedResultVertexCount * sourceAttributeData.UVChannelCount);

                var indices = new IndexLists<ushort>(subMeshCount, Allocator.Temp);

                var sourceForward = SourceOrientation.forward.ToFloat3();
                var sourceUp = SourceOrientation.up.ToFloat3();
                var sourceRight = SourceOrientation.isRightHanded
                    ? cross(sourceForward, sourceUp)
                    : cross(sourceUp, sourceForward);

                for (var z = 0; z < copyCount; ++z)
                {
                    var zOffset = z * meshLength;
                    for (var sourceIndex = 0; sourceIndex < sourceVertexCount; ++sourceIndex)
                    {
                        var resultIndex = z * sourceVertexCount + sourceIndex;
                        sourceAttributeData.GetFloat3(sourceIndex, VertexAttribute.Position, out var position);
                        position = float3(dot(sourceRight, position), dot(sourceUp, position),
                            dot(sourceForward, position) - meshMinZ + zOffset);
                        sourceAttributeData.GetFloat3(sourceIndex, VertexAttribute.Normal, out var normal);
                        normal = float3(dot(sourceRight, normal), dot(sourceUp, normal), dot(sourceForward, normal));
                        sourceAttributeData.GetFloat4(sourceIndex, VertexAttribute.Tangent, out var tangent);
                        tangent = float4(dot(sourceRight, tangent.xyz), dot(sourceUp, tangent.xyz),
                            dot(sourceForward, tangent.xyz), tangent.w);

                        positions[resultIndex] = position;
                        normals[resultIndex] = normal;
                        tangents[resultIndex] = tangent;

                        for (var channel = 0; channel < sourceAttributeData.UVChannelCount; ++channel)
                        {
                            sourceAttributeData.GetFloat2(sourceIndex, VertexAttribute.TexCoord0 + channel,
                                out var uv);
                            uvs[resultIndex * sourceAttributeData.UVChannelCount + channel] = uv + UVOffset * z;
                        }
                    }

                    // copy indices
                    // the last set of triangles is not copied for now, but added and potentially clipped later
                    if (z == copyCount - 1) continue;

                    for (var subMeshIndex = 0; subMeshIndex < subMeshCount; ++subMeshIndex)
                    {
                        var src = sourceIndices[subMeshIndex];
                        var dst = indices[subMeshIndex];
                        dst.ResizeUninitialized(dst.Length + src.Length);
                        for (var i = 0; i < src.Length; ++i)
                            dst[dst.Length - src.Length + i] = (ushort)(z * sourceVertexCount + src[i]);
                    }
                }

                if (copyCount >= 1)
                {
#if NATIVE_HASHSET_PARALLEL
                    var intersectedIndices = new NativeParallelHashMap<int2, ushort>(128, Allocator.Temp);
#else
                    var intersectedIndices = new NativeHashMap<int2, ushort>(128, Allocator.Temp);
#endif

                    for (var subMeshIndex = 0; subMeshIndex < subMeshCount; ++subMeshIndex)
                    {
                        var src = sourceIndices[subMeshIndex];
                        var dst = indices[subMeshIndex];
                        var vertexOffset = (ushort)((copyCount - 1) * sourceVertexCount);
                        for (var i = 0; i + 2 < src.Length; i += 3)
                        {
                            var ia = (ushort)(src[i] + vertexOffset);
                            var ib = (ushort)(src[i + 1] + vertexOffset);
                            var ic = (ushort)(src[i + 2] + vertexOffset);
                            var a = positions[ia];
                            var b = positions[ib];
                            var c = positions[ic];
                            var insideCount = 0;
                            if (a.z <= bezierLength) ++insideCount;
                            if (b.z <= bezierLength) ++insideCount;
                            if (c.z <= bezierLength) ++insideCount;
                            switch (insideCount)
                            {
                                case 3:
                                    dst.Add(ia);
                                    dst.Add(ib);
                                    dst.Add(ic);
                                    break;
                                case 2:
                                {
                                    // shuffle to make a and b inside
                                    if (a.z > bezierLength)
                                    {
                                        (ia, ib, ic) = (ib, ic, ia);
                                        (a, b, c) = (b, c, a);
                                    }
                                    else if (b.z > bezierLength)
                                    {
                                        (ia, ib, ic) = (ic, ia, ib);
                                        (a, b, c) = (c, a, b);
                                    }

                                    // between a and c on the clipping plane
                                    AddBetween(
                                        positions, normals, tangents, uvs, sourceAttributeData.UVChannelCount,
                                        ia, ic, (bezierLength - a.z) / (c.z - a.z),
                                        intersectedIndices, out var iac
                                    );
                                    // between b and c on the clipping plane
                                    AddBetween(
                                        positions, normals, tangents, uvs, sourceAttributeData.UVChannelCount,
                                        ib, ic, (bezierLength - b.z) / (c.z - b.z),
                                        intersectedIndices, out var ibc
                                    );

                                    dst.Add(ia);
                                    dst.Add(ib);
                                    dst.Add(iac);
                                    dst.Add(iac);
                                    dst.Add(ib);
                                    dst.Add(ibc);

                                    break;
                                }
                                case 1:
                                {
                                    // shuffle to make b and c outside
                                    if (b.z <= bezierLength)
                                    {
                                        (ia, ib, ic) = (ib, ic, ia);
                                        (a, b, c) = (b, c, a);
                                    }
                                    else if (c.z <= bezierLength)
                                    {
                                        (ia, ib, ic) = (ic, ia, ib);
                                        (a, b, c) = (c, a, b);
                                    }

                                    // between a and b on the clipping plane
                                    AddBetween(
                                        positions, normals, tangents, uvs, sourceAttributeData.UVChannelCount,
                                        ia, ib, (bezierLength - a.z) / (b.z - a.z),
                                        intersectedIndices, out var iab
                                    );
                                    // between a and c on the clipping plane
                                    AddBetween(
                                        positions, normals, tangents, uvs, sourceAttributeData.UVChannelCount,
                                        ia, ic, (bezierLength - a.z) / (c.z - a.z),
                                        intersectedIndices, out var iac
                                    );

                                    dst.Add(ia);
                                    dst.Add(iab);
                                    dst.Add(iac);

                                    break;
                                }
                            }
                        }
                    }
                }

                // bend along bezier
                var resultVertexCount = positions.Length;
                for (var i = 0; i < resultVertexCount; ++i)
                {
                    var pos = positions[i];
                    var pointIndex = clamp((int)floor(pos.z / StepSize), 0, Points.Length - 2);
                    var weight = clamp(
                        pointIndex < Points.Length - 2
                            ? pos.z / StepSize - pointIndex
                            : (pos.z - StepSize * pointIndex) /
                              distance(Points[Points.Length - 1].Position, Points[Points.Length - 2].Position),
                        0, 1);
                    var centerPos = lerp(Points[pointIndex].Position, Points[pointIndex + 1].Position, weight);
                    var forward = normalize(lerp(Points[pointIndex].Forward, Points[pointIndex + 1].Forward, weight));
                    var up = normalize(lerp(Points[pointIndex].Normal, Points[pointIndex + 1].Normal, weight));
                    var right = cross(up, forward);

                    positions[i] = centerPos + right * pos.x + up * pos.y;
                    normals[i] = right * normals[i].x + up * normals[i].y + forward * normals[i].z;
                    tangents[i] = float4(
                        right * tangents[i].x + up * tangents[i].y + forward * tangents[i].z, tangents[i].w);
                }

                var vertexAttributes = new NativeArray<VertexAttributeDescriptor>(
                    3 + sourceAttributeData.UVChannelCount,
                    Allocator.Temp, NativeArrayOptions.UninitializedMemory);
                vertexAttributes[0] = new VertexAttributeDescriptor(
                    attribute: VertexAttribute.Position, format: VertexAttributeFormat.Float32, dimension: 3,
                    stream: 0);
                vertexAttributes[1] = new VertexAttributeDescriptor(
                    attribute: VertexAttribute.Normal, format: VertexAttributeFormat.Float32, dimension: 3, stream: 1);
                vertexAttributes[2] = new VertexAttributeDescriptor(
                    attribute: VertexAttribute.Tangent, format: VertexAttributeFormat.Float32, dimension: 4, stream: 2);
                for (var i = 0; i < sourceAttributeData.UVChannelCount; i++)
                {
                    var attr = SourceVertexAttributes[(int)(VertexAttribute.TexCoord0 + i)];
                    attr.stream = 3;
                    vertexAttributes[3 + i] = attr;
                }

                ResultMeshData.SetVertexBufferParams(resultVertexCount, vertexAttributes);
                vertexAttributes.Dispose();

                var resultPositions = ResultMeshData.GetVertexData<float3>(stream: 0);
                var resultNormals = ResultMeshData.GetVertexData<float3>(stream: 1);
                var resultTangents = ResultMeshData.GetVertexData<float4>(stream: 2);
                var resultUVs = ResultMeshData.GetVertexData<float2>(stream: 3);

                resultPositions.CopyFrom(positions.AsArray());
                resultNormals.CopyFrom(normals.AsArray());
                resultTangents.CopyFrom(tangents.AsArray());
                resultUVs.CopyFrom(uvs.AsArray());

                var boundsMin = new float3(float.MaxValue);
                var boundsMax = new float3(float.MinValue);

                foreach (var position in positions)
                {
                    boundsMin = min(boundsMin, position);
                    boundsMax = max(boundsMax, position);
                }

                if (positions.Length == 0)
                {
                    boundsMin = boundsMax = 0;
                }

                ResultBounds[0] = boundsMin;
                ResultBounds[1] = boundsMax;

                ResultMeshData.SetIndexBufferParams(indices.TotalIndexCount, IndexFormat.UInt16);
                ResultMeshData.subMeshCount = subMeshCount;
                var indexData = ResultMeshData.GetIndexData<ushort>();
                var indexOffset = 0;
                for (var subMeshIndex = 0; subMeshIndex < subMeshCount; ++subMeshIndex)
                {
                    var subMesh = SourceMeshData.GetSubMesh(subMeshIndex);
                    var subMeshIndices = indices[subMeshIndex];
                    indexData.GetSubArray(indexOffset, subMeshIndices.Length).CopyFrom(subMeshIndices);

                    boundsMin = new float3(float.MaxValue);
                    boundsMax = new float3(float.MinValue);
                    var minIndex = int.MaxValue;
#if NATIVE_HASHSET_PARALLEL
                    using var usedIndices = new NativeParallelHashSet<int>(subMeshIndices.Length, Allocator.Temp);
#else
                    using var usedIndices = new NativeHashSet<int>(subMeshIndices.Length, Allocator.Temp);
#endif

                    for (var i = 0; i < subMeshIndices.Length; ++i)
                    {
                        var index = subMeshIndices[i];
                        boundsMin = min(boundsMin, positions[index]);
                        boundsMax = max(boundsMax, positions[index]);
                        minIndex = min(minIndex, index);
                        usedIndices.Add(index);
                    }

                    ResultMeshData.SetSubMesh(subMeshIndex, new SubMeshDescriptor
                    {
                        bounds = new Bounds { min = boundsMin, max = boundsMax },
                        topology = subMesh.topology,
                        indexStart = indexOffset,
                        indexCount = subMeshIndices.Length,
                        firstVertex = minIndex,
#if NATIVE_HASHSET_COUNT_FUN
                        vertexCount = usedIndices.Count()
#else
                        vertexCount = usedIndices.Count
#endif
                    }, MeshUpdateFlags.DontRecalculateBounds);
                    indexOffset += subMeshIndices.Length;
                }

                positions.Dispose();
                normals.Dispose();
                tangents.Dispose();
                uvs.Dispose();
                sourceIndices.Dispose();
                indices.Dispose();
            }

            private static void AddBetween(
                NativeList<float3> positions,
                NativeList<float3> normals,
                NativeList<float4> tangents,
                NativeList<float2> uvs,
                int uvChannelCount,
                ushort ia,
                ushort ib,
                float t,
                NativeHashMap<int2, ushort> intersectedIndices,
                out ushort resultIndex
            )
            {
                if (intersectedIndices.TryGetValue(new int2(ia, ib), out var index))
                {
                    resultIndex = index;
                    return;
                }

                positions.Add(lerp(positions[ia], positions[ib], t));
                normals.Add(normalize(lerp(normals[ia], normals[ib], t)));
                tangents.Add(float4(normalize(lerp(tangents[ia].xyz, tangents[ib].xyz, t)), 1));
                for (var channel = 0; channel < uvChannelCount; ++channel)
                    uvs.Add(lerp(uvs[ia * uvChannelCount + channel], uvs[ib * uvChannelCount + channel], t));
                intersectedIndices[new int2(ia, ib)] = resultIndex = (ushort)(positions.Length - 1);
            }

            [StructLayout(LayoutKind.Sequential)]
            [SuppressMessage("ReSharper", "UnusedMember.Local")]
            private struct IndexLists<T> : IDisposable where T : unmanaged
            {
                public NativeList<T>
                    SubMesh0,
                    SubMesh1,
                    SubMesh2,
                    SubMesh3,
                    SubMesh4,
                    SubMesh5,
                    SubMesh6,
                    SubMesh7,
                    SubMesh8,
                    SubMesh9,
                    SubMesh10,
                    SubMesh11,
                    SubMesh12,
                    SubMesh13,
                    SubMesh14,
                    SubMesh15,
                    SubMesh16,
                    SubMesh17,
                    SubMesh18,
                    SubMesh19,
                    SubMesh20,
                    SubMesh21,
                    SubMesh22,
                    SubMesh23,
                    SubMesh24,
                    SubMesh25,
                    SubMesh26,
                    SubMesh27,
                    SubMesh28,
                    SubMesh29,
                    SubMesh30,
                    SubMesh31;

                public Allocator Allocator;

                private int _subMeshCount;

                public IndexLists(Allocator allocator)
                {
                    this = default;
                    Allocator = allocator;
                }

                public IndexLists(int subMeshCount, Allocator allocator)
                {
                    this = default;
                    Allocator = allocator;
                    Resize(subMeshCount);
                }

                private unsafe ref NativeList<T> GetUnchecked(int index) =>
                    ref UnsafeUtility.ArrayElementAsRef<NativeList<T>>(
                        UnsafeUtility.AddressOf(ref SubMesh0), index);

                public ref NativeList<T> this[int index]
                {
                    get
                    {
                        if (index < 0 || index >= _subMeshCount)
                            throw new IndexOutOfRangeException();
                        return ref GetUnchecked(index);
                    }
                }

                public void Resize(int subMeshCount)
                {
                    if (subMeshCount < 0 || subMeshCount > 32)
                        throw new ArgumentOutOfRangeException();
                    if (subMeshCount == _subMeshCount) return;
                    if (subMeshCount < _subMeshCount)
                    {
                        for (var i = subMeshCount; i < _subMeshCount; ++i)
                            GetUnchecked(i).Dispose();
                    }
                    else
                    {
                        for (var i = _subMeshCount; i < subMeshCount; ++i)
                            GetUnchecked(i) = new NativeList<T>(Allocator);
                    }

                    _subMeshCount = subMeshCount;
                }

                public int SubMeshCount
                {
                    get => _subMeshCount;
                    set => Resize(value);
                }

                public int TotalIndexCount
                {
                    get
                    {
                        var sum = 0;
                        for (var i = 0; i < _subMeshCount; ++i)
                            sum += this[i].Length;
                        return sum;
                    }
                }

                public void Dispose()
                {
                    for (var i = 0; i < _subMeshCount; ++i)
                        this[i].Dispose();
                }
            }
        }

        /// <summary>
        /// To be called whenever the road shape changes. Will regenerate the mesh if AutoGenerate is true.
        /// </summary>
        /// <param name="update">- whether to regenerate the mesh at all.</param>
        public void Invalidate(bool update = true)
        {
            Valid = false;
            if (AutoGenerate && update) GenerateRoadMesh();
        }
    }
}
