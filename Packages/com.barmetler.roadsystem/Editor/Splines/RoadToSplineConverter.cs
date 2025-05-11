using UnityEditor;
using UnityEngine;
#if COM_UNITY_SPLINES
using static Unity.Mathematics.math;
#endif

namespace Barmetler.RoadSystem.Splines
{
    public static class RoadToSplineEditor
    {
#if COM_UNITY_SPLINES
        /// <summary>
        /// Select any object or objects in the scene. All roads found in the selection will be converted to splines,
        /// including any children. For example, you could select the RoadSystem object.
        /// </summary>
        [MenuItem("Tools/RoadSystem/Create or Update Spline(s) from Road")]
        public static void ConvertRoadToSpline()
        {
            var targets = Selection.GetFiltered<Road>(SelectionMode.Deep | SelectionMode.Editable);

            Undo.SetCurrentGroupName($"Convert {targets.Length} Road(s) to Spline(s)");

            foreach (var road in targets)
            {
                var splineChild = road.transform.Find("Spline");
                if (!splineChild)
                {
                    splineChild = new GameObject("Spline").transform;
                    splineChild.SetParent(road.transform);
                    splineChild.localPosition = Vector3.zero;
                    splineChild.localRotation = Quaternion.identity;
                    splineChild.localScale = Vector3.one;
                    Undo.RegisterCreatedObjectUndo(splineChild.gameObject, "Create Spline");
                }

                if (splineChild.gameObject.TryGetComponent(out UnityEngine.Splines.SplineContainer splineContainer))
                    Undo.RecordObject(splineContainer, "Convert Road to Spline");
                else
                    splineContainer = Undo.AddComponent<UnityEngine.Splines.SplineContainer>(splineChild.gameObject);

                if (splineContainer.Splines.Count < 1)
                    splineContainer.Splines = new UnityEngine.Splines.Spline[1];
                splineContainer.Spline ??= new UnityEngine.Splines.Spline();
                splineContainer.Spline.Clear();

                for (var i = 0; i < road.NumSegments + 1; i++)
                {
                    var knot = new UnityEngine.Splines.BezierKnot(road[i * 3]);
                    var forward = i < road.NumSegments
                        ? normalize(road[i * 3 + 1] - road[i * 3])
                        : i > 0
                            ? normalize(road[i * 3] - road[i * 3 - 1])
                            : road.transform.forward;
                    float inLength = 0, outLength = 0;
                    if (i > 0)
                        inLength = length(road[i * 3] - road[i * 3 - 1]);
                    if (i < road.NumSegments)
                        outLength = length(road[i * 3 + 1] - road[i * 3]);
                    if (i == 0)
                        inLength = outLength;
                    if (i == road.NumSegments)
                        outLength = inLength;
                    if (i == 0 && road.NumSegments == 0)
                        inLength = outLength = 1;

                    knot.TangentIn = float3(0, 0, -inLength);
                    knot.TangentOut = float3(0, 0, outLength);
                    knot.Rotation = Quaternion.LookRotation(forward, road.GetNormal(i));

                    splineContainer.Spline.Add(knot);
                }

                splineContainer.Spline.SetTangentMode(UnityEngine.Splines.TangentMode.Continuous);
            }

            Debug.Log($"Converted {targets.Length} Road(s) to Spline(s)");
        }
#else
        [MenuItem("Tools/RoadSystem/Create or Update Spline(s) from Road (requires com.unity.splines)")]
        public static void ConvertRoadToSpline()
        {
            Debug.Log("asd");
        }

        [MenuItem("Tools/RoadSystem/Create or Update Spline(s) from Road (requires com.unity.splines)", true)]
        public static bool ConvertRoadToSplineValidate() => false;
#endif
    }
}
