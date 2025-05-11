using Barmetler.RoadSystem.Util;
using NUnit.Framework;
using Unity.Collections;

namespace Tests.Runtime
{
    public class NativeMinHeapTests
    {
        [Test]
        public void T000_Runs() => Assert.Pass();

        [Test]
        public void T010_Insert_ShouldPlaceDataCorrectly()
        {
            var heap = new NativeMinHeap(20, Allocator.Temp);
            try
            {
                var data = new[]
                {
                    (8, 10f), (5, 7f), (11, 2f), (13, 14f),
                    (6, 14.5f), (7, 3f), (19, 13f), (9, 11f),
                    (1, 28f), (3, 6f), (16, 44f), (17, 49f),
                    (2, 45f), (15, 43f), (10, 7.5f), (0, 9f)
                };
                foreach (var (index, priority) in data) heap.Insert(index, priority);
                var expected = new[] { 11, 7, 3, 5, 10, 0, 8, 9, 19, 13, 6, 1, 15, 16, 2, 17 };
                var actual = heap.ToArray();
                CollectionAssert.AreEqual(expected, actual);
            }
            finally
            {
                heap.Dispose();
            }
        }

        [Test]
        public void T020_Update_ShouldKeepOrdering()
        {
            var heap = new NativeMinHeap(20, Allocator.Temp);
            try
            {
                var data = new[]
                {
                    (8, 10f), (5, 7f), (11, 2f), (13, 14f),
                    (6, 14.5f), (7, 3f), (19, 13f), (9, 11f),
                    (1, 28f), (3, 6f), (16, 44f), (17, 49f),
                    (2, 45f), (15, 43f), (10, 7.5f), (0, 9f)
                };
                foreach (var (index, priority) in data) heap.Insert(index, priority);
                heap.Update(10, 23f);
                heap.Update(0, 1f);
                heap.Update(17, 0.5f);
                heap.Update(11, 32f);
                var expected = new[] { 17, 0, 7, 3, 5, 8, 9, 19, 13, 6, 10, 1, 11, 15, 16, 2 };
                var actual = heap.ToArray();
                CollectionAssert.AreEqual(expected, actual);
            }
            finally
            {
                heap.Dispose();
            }
        }
    }
}