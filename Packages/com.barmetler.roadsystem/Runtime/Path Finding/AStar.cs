using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using Barmetler.DictExtensions;
using Barmetler.RoadSystem.Util;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace Barmetler
{
    namespace DictExtensions
    {
        public static class MyExtensions
        {
            /// <summary>
            /// Try to get a value from a Dictionary, but return default if the key is not in the Dictionary.
            /// </summary>
            /// <returns>this[k] if exists, defaultValue otherwise</returns>
            public static V GetWithDefault<K, V>(this Dictionary<K, V> dict, K key, V defaultValue)
            {
                return dict.TryGetValue(key, out var value) ? value : defaultValue;
            }
        }
    }

    [BurstCompile]
    public static class AStar
    {
        public class NodeBase
        {
            public Vector3 position;
        }

        public delegate float Heuristic<NodeType>(NodeType node, NodeType goal) where NodeType : NodeBase;

        private static float DefaultHeuristic<NodeType>(NodeType node, NodeType goal) where NodeType : NodeBase
        {
            return (node.position - goal.position).magnitude;
        }

        /// <summary>
        /// Find the shortest path from one point to another in a di-graph.
        /// </summary>
        /// <typeparam name="NodeType">- Must Extend AStar.NodeBase</typeparam>
        /// <param name="nodes"></param>
        /// <param name="weights">weights[x,y] is edge cost from x to y.</param>
        /// <param name="start"></param>
        /// <param name="goal"></param>
        /// <param name="heuristic">Eulerian distance to goal per default.</param>
        /// <returns>Shortest path from start to goal.</returns>
        public static List<NodeType> FindShortestPath<NodeType>(
            List<NodeType> nodes, TwoDimensionalArray<float> weights, NodeType start, NodeType goal,
            Heuristic<NodeType> heuristic = null, int maxSteps = 10000
        )
            where NodeType : NodeBase
        {
            heuristic ??= DefaultHeuristic;
            float h(NodeType node) => heuristic(node, start);

            var cameFrom = new Dictionary<NodeType, NodeType>();
            var gScore = new Dictionary<NodeType, float>
            {
                [start] = 0
            };
            var fScore = new Dictionary<NodeType, float>
            {
                [start] = h(start)
            };

            var comparer = Comparer<NodeType>.Create((a, b) =>
            {
                var fCompare = fScore.GetWithDefault(a, float.PositiveInfinity)
                    .CompareTo(fScore.GetWithDefault(b, float.PositiveInfinity));
                if (fCompare != 0) return fCompare;
                var gCompare = gScore.GetWithDefault(a, float.PositiveInfinity)
                    .CompareTo(gScore.GetWithDefault(b, float.PositiveInfinity));
                if (gCompare != 0) return gCompare;
                return nodes.IndexOf(a).CompareTo(nodes.IndexOf(b));
            });

            var openSet = new SortedSet<NodeType>(new[] { start }, comparer);

            for (var steps = 0; openSet.Count > 0; ++steps)
            {
                if (steps == maxSteps)
                    _ = 0; // for debugging
                if (steps > maxSteps)
                    throw new Exception("Too many steps!");

                var current = openSet.Min;
                openSet.Remove(current);
                if (current == goal)
                {
                    // Debug.Log("Steps taken: " + steps);
                    return ReconstructPath(cameFrom, current);
                }

                // Pair is neighbor and the distance to that neighbor
                foreach (var neighbor in GetNeighbors(nodes, weights, current))
                {
                    var tentative_gScore = gScore[current] + neighbor.Value;
                    if (tentative_gScore < gScore.GetWithDefault(neighbor.Key, float.PositiveInfinity))
                    {
                        // remove before adding to make sure it's re-sorted
                        openSet.Remove(neighbor.Key);

                        cameFrom[neighbor.Key] = current;
                        gScore[neighbor.Key] = tentative_gScore;
                        fScore[neighbor.Key] = gScore[neighbor.Key] + h(neighbor.Key);

                        openSet.Add(neighbor.Key);
                    }
                }
            }

            throw new Exception("No Path Found!");
        }

        private static List<NodeType> ReconstructPath<NodeType>(Dictionary<NodeType, NodeType> cameFrom,
            NodeType current) where NodeType : NodeBase
        {
            var path = new List<NodeType>();
            while (cameFrom.ContainsKey(current))
            {
                path.Insert(0, current);
                current = cameFrom[current];
            }

            path.Insert(0, current);
            return path;
        }

        private static List<KeyValuePair<NodeType, float>> GetNeighbors<NodeType>(List<NodeType> nodes,
            TwoDimensionalArray<float> weights, NodeType current) where NodeType : NodeBase
        {
            var l = new List<KeyValuePair<NodeType, float>>();
            var index = nodes.IndexOf(current);
            for (var other = 0; other < nodes.Count; ++other)
            {
                if (index == other) continue;
                if (float.IsInfinity(weights[index, other])) continue;
                l.Add(new KeyValuePair<NodeType, float>(nodes[other], weights[index, other]));
            }

            return l;
        }

        public delegate float Heuristic(in float3 node, in float3 goal);

        [BurstCompile]
        private static float DistanceHeuristicFun(in float3 node, in float3 goal) => math.distance(node, goal);

        [BurstCompile]
        private static float DijkstraHeuristicFun(in float3 node, in float3 goal) => 0;

        private static readonly Lazy<FunctionPointer<Heuristic>> DistanceHeuristicLazy =
            new Lazy<FunctionPointer<Heuristic>>(
                () => BurstCompiler.CompileFunctionPointer<Heuristic>(DistanceHeuristicFun));

        private static readonly Lazy<FunctionPointer<Heuristic>> DijkstraHeuristicLazy =
            new Lazy<FunctionPointer<Heuristic>>(
                () => BurstCompiler.CompileFunctionPointer<Heuristic>(DijkstraHeuristicFun));

        public static FunctionPointer<Heuristic> DistanceHeuristic => DistanceHeuristicLazy.Value;

        public static FunctionPointer<Heuristic> DijkstraHeuristic => DijkstraHeuristicLazy.Value;

        public static unsafe int[] FindShortestPath(
            NativeArray<float3> nodes, ExtendedTwoDimensionalNativeArray<float> weights, int start, int goal,
            out int stepsTaken,
            FunctionPointer<Heuristic> heuristic = default)
        {
            if (!heuristic.IsCreated) heuristic = DistanceHeuristic;
            var stepsTakenI = 0;
            var job = new FindShortestPathJob
            {
                HeuristicPtr = heuristic,
                Nodes = nodes,
                Weights = weights,
                Start = start,
                Goal = goal,
                MaxSteps = 10000,
                StepsTaken = &stepsTakenI,
                Path = new NativeList<int>(Allocator.TempJob)
            };
            job.Run();
            var path = job.Path.AsArray().ToArray();
            job.Path.Dispose();
            stepsTaken = stepsTakenI;
            return path;
        }

        [SuppressMessage("ReSharper", "SwapViaDeconstruction")] // tuples don't work with Burst !!!
        private unsafe struct FindShortestPathJob : IJob
        {
            public FunctionPointer<Heuristic> HeuristicPtr;

            [ReadOnly]
            public NativeArray<float3> Nodes;

            [ReadOnly]
            public ExtendedTwoDimensionalNativeArray<float> Weights;

            [ReadOnly]
            public int Start;

            [ReadOnly]
            public int Goal;

            [ReadOnly]
            public int MaxSteps;

            [NativeDisableUnsafePtrRestriction]
            public int* StepsTaken;

            public NativeList<int> Path;

            public void Execute()
            {
                // cameFrom[a] = b, if 'a' was reached from 'b' (a and b are indices into Nodes)
                var cameFrom = new NativeArray<int>(Nodes.Length, Allocator.Temp);
                var gScore = new NativeArray<float>(Nodes.Length, Allocator.Temp);
                var fScore = new NativeArray<float>(Nodes.Length, Allocator.Temp);
                var openSet = new NativeMinHeap(Nodes.Length, Allocator.Temp);

                for (var i = 0; i < Nodes.Length; ++i)
                    gScore[i] = fScore[i] = float.PositiveInfinity;
                gScore[Start] = 0;
                fScore[Start] = HeuristicPtr.Invoke(Nodes[Start], Nodes[Goal]);
                openSet.Insert(Start, fScore[Start]);

                var step = 0;
                for (; step < MaxSteps; ++step)
                {
                    if (openSet.Count == 0) break;

                    var current = openSet.ExtractMin();

                    if (current == Goal) goto reconstructPath;

                    for (var neighbor = 0; neighbor < Nodes.Length; ++neighbor)
                    {
                        if (neighbor == current) continue;
                        if (float.IsInfinity(Weights[current, neighbor]) || Weights[current, neighbor] < 1e-6) continue;
                        var tentativeGScore = gScore[current] + Weights[current, neighbor];
                        if (tentativeGScore >= gScore[neighbor]) continue;
                        cameFrom[neighbor] = current;
                        gScore[neighbor] = tentativeGScore;
                        fScore[neighbor] = gScore[neighbor] + HeuristicPtr.Invoke(Nodes[neighbor], Nodes[Goal]);
                        openSet.InsertOrUpdate(neighbor, fScore[neighbor]);
                    }
                }

                goto cleanup;

                reconstructPath:
                // we use i to prevent infinite loops
                for (int i = 0, current = Goal; i < Nodes.Length && current != Start; ++i, current = cameFrom[current])
                    Path.Add(current);
                Path.Add(Start);
                // Reverse the path
                for (var i = 0; i < Path.Length / 2; ++i)
                {
                    var temp = Path[i];
                    Path[i] = Path[Path.Length - i - 1];
                    Path[Path.Length - i - 1] = temp;
                }

                cleanup:
                *StepsTaken = step;
                cameFrom.Dispose();
                gScore.Dispose();
                fScore.Dispose();
                openSet.Dispose();
            }
        }
    }
}
