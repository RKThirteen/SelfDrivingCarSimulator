using Barmetler.RoadSystem.Util;
using JetBrains.Annotations;
using NUnit.Framework;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;

namespace Tests.Runtime
{
    [BurstCompile]
    public unsafe class NativeTwoDimensionalArrayTests
    {
        [Test]
        public void T010_Index_ShouldPlaceDataCorrectly()
        {
            var arr = new TwoDimensionalNativeArray<int>(8, 12, Allocator.Temp);
            for (int i = 0, y = 0; y < arr.Height; ++y)
            for (var x = 0; x < arr.Width; ++x)
                arr[x, y] = ++i * 3;
            for (int i = 1, y = 0; y < arr.Height; ++y)
            for (var x = 0; x < arr.Width; ++x, ++i)
            {
                Assert.AreEqual(i * 3, arr[x, y]);
                Assert.AreEqual(i * 3, arr[y * 8 + x]);
            }

            var arr2 = arr.ToArray();
            for (int i = 1, y = 0; y < arr.Height; ++y)
            for (var x = 0; x < arr.Width; ++x, ++i)
                Assert.AreEqual(i * 3, arr2[y * 8 + x]);
            arr.Dispose();
        }

        [BurstDiscard]
        private static void SetTrueIfNotBurst([UsedImplicitly] ref bool isNotBurst) => isNotBurst = true;

        private static bool IsBurst()
        {
            var isNotBurst = false;
            SetTrueIfNotBurst(ref isNotBurst);
            return !isNotBurst;
        }

        [BurstCompile]
        private struct CheckIsBurstJob : IJob
        {
            [NativeDisableUnsafePtrRestriction] public bool* IsBurstPtr;

            public void Execute()
            {
                *IsBurstPtr = IsBurst();
            }
        }

        [Test]
        public void T020_IsBurstJob_ShouldReturnTrue()
        {
            bool isBurst;
            var job = new CheckIsBurstJob { IsBurstPtr = &isBurst };
            job.Run();
            Assert.AreEqual(true, isBurst);
        }

        private delegate bool CheckIsBurstFunctionDelegate();

        [BurstCompile]
        private static bool CheckIsBurstFunction()
        {
            return IsBurst();
        }

        [Test]
        public void T030_IsBurstFunction_ShouldReturnTrue()
        {
            var f = BurstCompiler.CompileFunctionPointer<CheckIsBurstFunctionDelegate>(CheckIsBurstFunction).Invoke;
            Assert.AreEqual(true, f());
        }

        [BurstCompile]
        private struct SequentialJob : IJob
        {
            public TwoDimensionalNativeArray<int> Array;
            [NativeDisableUnsafePtrRestriction] public bool* Success;

            public void Execute()
            {
                if (!IsBurst())
                    throw new System.Exception("Not running in burst mode");
                for (int i = 0, y = 0; y < Array.Height; ++y)
                for (var x = 0; x < Array.Width; ++x)
                    Array[x, y] = ++i * 3;
                *Success = true;
            }
        }

        [Test]
        public void T040_BurstSequential_ShouldPlaceDataCorrectly()
        {
            var arr = new TwoDimensionalNativeArray<int>(8, 12, Allocator.TempJob);
            try
            {
                var success = false;
                var job = new SequentialJob
                {
                    Array = arr,
                    Success = &success
                };
                job.Run();
                Assert.AreEqual(true, success);
                for (int i = 1, y = 0; y < arr.Height; ++y)
                for (var x = 0; x < arr.Width; ++x, ++i)
                {
                    Assert.AreEqual(i * 3, arr[x, y]);
                    Assert.AreEqual(i * 3, arr[y * 8 + x]);
                }
            }
            finally
            {
                arr.Dispose();
            }
        }

        [BurstCompile]
        private struct ParallelJob : IJobParallelFor
        {
            public TwoDimensionalNativeArray<int> Array;
            [NativeDisableUnsafePtrRestriction] public bool* Success;

            public void Execute(int index)
            {
                if (!IsBurst())
                    throw new System.Exception("Not running in burst mode");
                var x = index % Array.Width;
                var y = index / Array.Width;
                Array[x, y] = (index + 1) * 3;
                *Success = true;
            }
        }

        [Test]
        public void T050_BurstParallel_ShouldPlaceDataCorrectly()
        {
            var arr = new TwoDimensionalNativeArray<int>(8, 12, Allocator.TempJob);
            try
            {
                var success = false;
                var job = new ParallelJob
                {
                    Array = arr,
                    Success = &success
                };
                job.Schedule(arr.Length, 1).Complete();
                Assert.AreEqual(true, success);
                for (int i = 1, y = 0; y < arr.Height; ++y)
                for (var x = 0; x < arr.Width; ++x, ++i)
                {
                    Assert.AreEqual(i * 3, arr[x, y]);
                    Assert.AreEqual(i * 3, arr[y * 8 + x]);
                }
            }
            finally
            {
                arr.Dispose();
            }
        }

        [Test]
        public void T060_Extended_ShouldPlaceDataCorrectly()
        {
            TwoDimensionalNativeArray<int> inner;
            var arr = new ExtendedTwoDimensionalNativeArray<int>(
                inner = new TwoDimensionalNativeArray<int>(5, 6, Allocator.Temp),
                startX: 2, startY: 1, width: 8, height: 8,
                Allocator.Temp
            );
            try
            {
                for (int i = 1, y = 0; y < arr.Height; ++y)
                for (var x = 0; x < arr.Width; ++x, ++i)
                    arr[x, y] = i * 3;
                for (int i = 1, y = 0; y < arr.Height; ++y)
                for (var x = 0; x < arr.Width; ++x, ++i)
                    Assert.AreEqual(i * 3, arr[x, y]);
            }
            finally
            {
                arr.Dispose();
                inner.Dispose();
            }
        }

        [Test]
        public void T070_ExtendedExistingInner_ShouldReturnDataCorrectly()
        {
            var inner = new TwoDimensionalNativeArray<int>(4, 6, Allocator.Temp);
            for (int i = 1, y = 0; y < inner.Height; ++y)
            for (var x = 0; x < inner.Width; ++x, ++i)
                inner[x, y] = i * 3;
            var outer = new ExtendedTwoDimensionalNativeArray<int>(
                inner,
                startX: 2, startY: 2, width: 8, height: 8,
                Allocator.Temp
            );
            try
            {
                var expected = new[]
                {
                    0, 0, 00, 00, 00, 00, 0, 0,
                    0, 0, 00, 00, 00, 00, 0, 0,
                    0, 0, 03, 06, 09, 12, 0, 0,
                    0, 0, 15, 18, 21, 24, 0, 0,
                    0, 0, 27, 30, 33, 36, 0, 0,
                    0, 0, 39, 42, 45, 48, 0, 0,
                    0, 0, 51, 54, 57, 60, 0, 0,
                    0, 0, 63, 66, 69, 72, 0, 0
                };
                var actual = outer.ToArray();

                Assert.AreEqual(expected.Length, actual.Length);
                CollectionAssert.AreEqual(expected, actual);
            }
            finally
            {
                outer.Dispose();
                inner.Dispose();
            }
        }
    }
}