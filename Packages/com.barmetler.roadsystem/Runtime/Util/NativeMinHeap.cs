using System;
using System.Diagnostics;
using System.Diagnostics.CodeAnalysis;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;

namespace Barmetler.RoadSystem.Util
{
    /// <summary>
    /// MinHeap, with support for updating values.
    /// </summary>
    [BurstCompatible]
    [DebuggerDisplay("Length = {Count}/{_nodes.Length}")]
    [SuppressMessage("ReSharper", "SwapViaDeconstruction")] // tuples don't work with Burst !!!
    public struct NativeMinHeap
    {
        private struct Node
        {
            public int Index;
            public float Priority;
        }

        private NativeArray<Node> _nodes;
        private NativeArray<int> _indices;

        public int Count { get; private set; }

        public NativeMinHeap(int size, Allocator allocator)
        {
            _nodes = new NativeArray<Node>(size, allocator);
            _indices = new NativeArray<int>(size, allocator);
            Count = 0;
            for (var i = 0; i < _indices.Length; ++i) _indices[i] = -1;
        }

        private static int Parent(int i) => (i - 1) / 2;
        private static int Left(int i) => 2 * i + 1;
        private static int Right(int i) => 2 * i + 2;

        public int Min
        {
            get
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                if (Count == 0) throw new InvalidOperationException("Heap is empty");
#endif
                return _nodes[0].Index;
            }
        }

        [WriteAccessRequired]
        public void Insert(int index, float priority)
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            if (_indices[index] != -1)
                throw new InvalidOperationException("Index already exists in heap");
            if (Count == _nodes.Length)
                throw new InvalidOperationException("Heap is full");
#endif
            _nodes[Count] = new Node
            {
                Index = index,
                Priority = priority
            };
            _indices[index] = Count;
            ++Count;
            SiftUp(Count - 1);
        }

        [WriteAccessRequired]
        public int ExtractMin()
        {
            var result = Min;
            Swap(0, Count - 1);
            _indices[result] = -1;
            --Count;

            SiftDown(0);

            return result;
        }

        [WriteAccessRequired]
        public void Update(int index, float priority)
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            if (_indices[index] == -1)
                throw new InvalidOperationException("Index does not exist in heap");
#endif
            var i = _indices[index];
            var oldPriority = _nodes[i].Priority;
            _nodes[i] = new Node
            {
                Index = index,
                Priority = priority
            };
            if (priority < oldPriority) SiftUp(i);
            else SiftDown(i);
        }

        public bool Contains(int index) => _indices[index] != -1;

        [WriteAccessRequired]
        public void InsertOrUpdate(int index, float priority)
        {
            if (Contains(index)) Update(index, priority);
            else Insert(index, priority);
        }

        [WriteAccessRequired]
        public void Dispose()
        {
            _nodes.Dispose();
            _indices.Dispose();
            Count = 0;
        }

        public void Dispose(JobHandle inputDeps)
        {
            _nodes.Dispose(inputDeps);
            _indices.Dispose(inputDeps);
        }

        public int[] ToArray()
        {
            var copy = new NativeMinHeap(_nodes.Length, Allocator.Temp);
            Copy(in this, ref copy);
            var result = new int[Count];
            for (var i = 0; i < result.Length; ++i) result[i] = copy.ExtractMin();
            copy.Dispose();
            return result;
        }

        [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS")]
        private static void CheckCopyLengths(int sourceLength, int destinationLength)
        {
            if (sourceLength != destinationLength)
                throw new InvalidOperationException("source and destination length must be the same");
        }

        [WriteAccessRequired]
        public void CopyFrom(in NativeMinHeap other) => Copy(in other, ref this);

        public void CopyTo(ref NativeMinHeap other) => Copy(in this, ref other);

        public static void Copy(in NativeMinHeap source, ref NativeMinHeap destination)
        {
            CheckCopyLengths(source._nodes.Length, destination._nodes.Length);
            CheckCopyLengths(source._indices.Length, destination._indices.Length);
            destination.Count = source.Count;
            NativeArray<Node>.Copy(source._nodes, destination._nodes);
            NativeArray<int>.Copy(source._indices, destination._indices);
        }

        private void Swap(int a, int b)
        {
            var ia = _nodes[a].Index;
            var ib = _nodes[b].Index;
            _indices[ia] = b;
            _indices[ib] = a;
            var t = _nodes[a];
            _nodes[a] = _nodes[b];
            _nodes[b] = t;
        }

        private void SiftUp(int i)
        {
            while (i != 0 && _nodes[Parent(i)].Priority > _nodes[i].Priority)
            {
                Swap(i, Parent(i));
                i = Parent(i);
            }
        }

        private void SiftDown(int i)
        {
            while (Left(i) < Count)
            {
                if (Right(i) >= Count)
                {
                    if (_nodes[i].Priority > _nodes[Left(i)].Priority)
                        Swap(i, Left(i));
                    return;
                }

                if (_nodes[i].Priority <= _nodes[Left(i)].Priority &&
                    _nodes[i].Priority <= _nodes[Right(i)].Priority) return;

                if (_nodes[Left(i)].Priority < _nodes[Right(i)].Priority)
                {
                    Swap(i, Left(i));
                    i = Left(i);
                }
                else
                {
                    Swap(i, Right(i));
                    i = Right(i);
                }
            }
        }
    }
}
