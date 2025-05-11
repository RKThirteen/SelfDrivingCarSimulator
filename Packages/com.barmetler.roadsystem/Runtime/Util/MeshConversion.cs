using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using UnityEngine;
using static Unity.Mathematics.math;
using float3 = Unity.Mathematics.float3;

namespace Barmetler.RoadSystem.Util
{
    public static class MeshConversion
    {
        /// <summary>
        /// Describes the coordinate space of a mesh.
        /// </summary>
        [Serializable]
        public struct MeshOrientation : IEquatable<MeshOrientation>
        {
            public enum AxisDirection
            {
                X_POSITIVE,
                X_NEGATIVE,
                Y_POSITIVE,
                Y_NEGATIVE,
                Z_POSITIVE,
                Z_NEGATIVE
            }

            [Tooltip("Which axis represents the forward vector?")]
            public AxisDirection forward;

            [Tooltip("Which axis represents the up vector?")]
            public AxisDirection up;

            [Tooltip("False for left-handed (like in Unity), true for right-handed (like in Blender)")]
            public bool isRightHanded;

            public string Preset
            {
                get
                {
                    if (PresetNames.TryGetValue(this, out var ret))
                        return ret;
                    foreach (var kv in Presets)
                    {
                        if (Equals(kv.Value))
                        {
                            PresetNames[this] = kv.Key;
                            return kv.Key;
                        }
                    }

                    return "CUSTOM";
                }
                set
                {
                    if (Presets.TryGetValue(value, out var v))
                    {
                        this = v;
                        PresetNames[v] = value;
                    }
                }
            }

            public static readonly Dictionary<string, MeshOrientation> Presets = new Dictionary<string, MeshOrientation>
            {
                ["BLENDER"] = new MeshOrientation
                {
                    forward = AxisDirection.Y_POSITIVE,
                    up = AxisDirection.Z_POSITIVE,
                    isRightHanded = true
                },
                ["UNITY"] = new MeshOrientation
                {
                    forward = AxisDirection.Z_POSITIVE,
                    up = AxisDirection.Y_POSITIVE,
                    isRightHanded = false
                }
            };

            private static readonly Dictionary<MeshOrientation, string> PresetNames =
                new Dictionary<MeshOrientation, string>();

            public bool Equals(MeshOrientation other)
            {
                return forward == other.forward && up == other.up && isRightHanded == other.isRightHanded;
            }

            public override bool Equals(object obj)
            {
                return obj is MeshOrientation other && Equals(other);
            }

            public override int GetHashCode()
            {
                unchecked
                {
                    var hashCode = (int)forward;
                    hashCode = (hashCode * 397) ^ (int)up;
                    hashCode = (hashCode * 397) ^ isRightHanded.GetHashCode();
                    return hashCode;
                }
            }
        }

        /// <summary>
        /// Perform a deep copy of a Mesh.
        /// </summary>
        /// <param name="m">- the Mesh to copy.</param>
        /// <returns>new Mesh instance.</returns>
        public static Mesh CopyMesh(Mesh m)
        {
            var ret = new Mesh
            {
                vertices = m.vertices.ToArray(),
                uv = m.uv.ToArray(),
                uv2 = m.uv2.ToArray(),
                tangents = m.tangents.ToArray(),
                normals = m.normals.ToArray(),
                colors32 = m.colors32.ToArray()
            };

            var t = new int[m.subMeshCount][];
            for (var i = 0; i < t.Length; i++)
                t[i] = m.GetTriangles(i);

            ret.name = m.name;
            ret.subMeshCount = m.subMeshCount;

            for (var i = 0; i < t.Length; i++)
                ret.SetTriangles(t[i], i);

            return ret;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 ToVector(this MeshOrientation.AxisDirection axis) => axis switch
        {
            MeshOrientation.AxisDirection.X_POSITIVE => Vector3.right,
            MeshOrientation.AxisDirection.X_NEGATIVE => -Vector3.right,
            MeshOrientation.AxisDirection.Y_POSITIVE => Vector3.up,
            MeshOrientation.AxisDirection.Y_NEGATIVE => -Vector3.up,
            MeshOrientation.AxisDirection.Z_POSITIVE => Vector3.forward,
            MeshOrientation.AxisDirection.Z_NEGATIVE => -Vector3.forward,
            _ => Vector3.zero, // can't happen
        };

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 ToFloat3(this MeshOrientation.AxisDirection axis) => axis switch
        {
            MeshOrientation.AxisDirection.X_POSITIVE => float3(1, 0, 0),
            MeshOrientation.AxisDirection.X_NEGATIVE => float3(-1, 0, 0),
            MeshOrientation.AxisDirection.Y_POSITIVE => float3(0, 1, 0),
            MeshOrientation.AxisDirection.Y_NEGATIVE => float3(0, -1, 0),
            MeshOrientation.AxisDirection.Z_POSITIVE => float3(0, 0, 1),
            MeshOrientation.AxisDirection.Z_NEGATIVE => float3(0, 0, -1),
            _ => float3.zero, // can't happen
        };

        /// <summary>
        /// Transform a Mesh from the supplied space to Unity's space.
        /// </summary>
        /// <param name="mesh">- the Mesh to transform</param>
        /// <param name="from">- the space the Mesh is currently in</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void TransformMesh(Mesh mesh, MeshOrientation from) =>
            TransformMesh(mesh, from, MeshOrientation.Presets["UNITY"]);

        /// <summary>
        /// Transform a Mesh from the one space to another
        /// </summary>
        /// <param name="mesh">- the Mesh to transform</param>
        /// <param name="from">- the space the Mesh is currently in</param>
        /// <param name="to">- the space the Mesh will be transformed to</param>
        public static void TransformMesh(Mesh mesh, MeshOrientation from, MeshOrientation to)
        {
            var fromForward = from.forward.ToVector();
            var fromUp = from.up.ToVector();
            var fromRight = from.isRightHanded
                ? Vector3.Cross(fromForward, fromUp)
                : Vector3.Cross(fromUp, fromForward);
            var toForward = to.forward.ToVector();
            var toUp = to.up.ToVector();
            var toRight = to.isRightHanded ? Vector3.Cross(toForward, toUp) : Vector3.Cross(toUp, toForward);

            var vertices = new Vector3[mesh.vertexCount];
            for (var i = 0; i < mesh.vertexCount; ++i)
            {
                var v = mesh.vertices[i];
                vertices[i] =
                    toRight * Vector3.Dot(fromRight, v) +
                    toForward * Vector3.Dot(fromForward, v) +
                    toUp * Vector3.Dot(fromUp, v);
            }

            mesh.SetVertices(vertices);

            for (var submesh = 0; submesh < mesh.subMeshCount; ++submesh)
            {
                var oldTriangles = mesh.GetTriangles(submesh);
                var triangles = new int[oldTriangles.Length];
                if (from.isRightHanded != to.isRightHanded)
                {
                    for (var tri = 0; tri + 3 <= triangles.Length; tri += 3)
                    {
                        triangles[tri] = oldTriangles[tri];
                        triangles[tri + 1] = oldTriangles[tri + 2];
                        triangles[tri + 2] = oldTriangles[tri + 1];
                    }
                }

                mesh.SetTriangles(triangles, submesh);
            }

            mesh.RecalculateNormals();
            mesh.RecalculateTangents();
            mesh.RecalculateBounds();
        }
    }
}
