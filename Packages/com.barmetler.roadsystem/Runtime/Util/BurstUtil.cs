using JetBrains.Annotations;
using Unity.Burst;

namespace Barmetler.RoadSystem.Util
{
    internal static class BurstUtil
    {
        [BurstDiscard]
        private static void SetTrueIfNotBurst([UsedImplicitly] ref bool isNotBurst) => isNotBurst = true;

        internal static bool IsBurst()
        {
            var isNotBurst = false;
            SetTrueIfNotBurst(ref isNotBurst);
            return !isNotBurst;
        }
    }
}
