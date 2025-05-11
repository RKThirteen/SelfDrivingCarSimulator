using System.Linq;
using System.Text;
using Barmetler.RoadSystem.Util;
using UnityEditor;
using UnityEngine;

namespace Barmetler.RoadSystem
{
    [CustomEditor(typeof(RoadSystem))]
    public class RoadSystemEditor : Editor
    {
        private RoadSystem _roadSystem;

        private void OnSceneGUI()
        {
            Draw();
        }

        private void Draw()
        {
            if (!_roadSystem.ShowDebugInfo) return;
            var edges = _roadSystem.GetGraphEdges();
            Handles.color = Color.blue;
            var style = new GUIStyle
            {
                normal =
                {
                    textColor = Color.magenta
                }
            };
            foreach (var road in _roadSystem.Roads)
            {
                if (!road) continue;
                foreach (var segment in Enumerable.Range(0, road.NumSegments))
                {
                    var points = road.GetPointsInSegment(segment).Select(e => road.transform.TransformPoint(e))
                        .ToArray();
                    Handles.DrawBezier(points[0], points[3], points[1], points[2], Color.green, null, 2f);
                }
            }

            foreach (var edge in edges)
            {
                Handles.DrawLine(edge.start, edge.end, 2f);
                if (_roadSystem.ShowEdgeWeights)
                    Handles.Label((edge.start + edge.end) / 2, "Cost: " + edge.cost, style);
            }
        }

        private int _presetSelectedIndex;
        private string _meshGenerationTime;

        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();

            GUILayout.Space(10);
            GUILayout.Label("Bulk Road Editing", EditorStyles.boldLabel);
            using (new GUILayout.VerticalScope("", "box"))
            {
                using (new GUILayout.HorizontalScope())
                {
                    var options = MeshConversion.MeshOrientation.Presets.Keys.ToList();
                    _presetSelectedIndex =
                        EditorGUILayout.Popup("Source Orientation Preset", _presetSelectedIndex, options.ToArray());
                    if (GUILayout.Button("Set All Roads"))
                    {
                        var generators = _roadSystem.GetComponentsInChildren<RoadMeshGenerator>();
                        var group = Undo.GetCurrentGroup();
                        // ReSharper disable once CoVariantArrayConversion
                        Undo.RecordObjects(generators, "Change Source Orientation Preset on all Roads");
                        foreach (var g in generators)
                            g.settings.SourceOrientation.Preset = options[_presetSelectedIndex];
                        Undo.CollapseUndoOperations(group);
                    }
                }

                using (new EditorGUILayout.HorizontalScope())
                {
                    if (GUILayout.Button("Construct Graph"))
                    {
                        var t = Time.realtimeSinceStartup;
                        _roadSystem.ConstructGraph();
                        _meshGenerationTime = $"Constructed graph in {(Time.realtimeSinceStartup - t) * 1000:0.000}ms.";
                        EditorUtility.SetDirty(_roadSystem);
                        SceneView.RepaintAll();
                    }

                    if (GUILayout.Button("Rebuild All Roads"))
                    {
                        var generators = _roadSystem.GetComponentsInChildren<RoadMeshGenerator>();
                        var t = Time.realtimeSinceStartup;
                        foreach (var g in generators)
                            g.GenerateRoadMesh();
                        _meshGenerationTime =
                            $"Generated {generators.Length} road meshes in {(Time.realtimeSinceStartup - t) * 1000:0.000}ms.";
                    }
                }

                if (GUILayout.Button("Construct Graph + Rebuild All Roads", GUILayout.Height(32)))
                {
                    var sb = new StringBuilder();
                    var t = Time.realtimeSinceStartup;
                    _roadSystem.ConstructGraph();
                    sb.AppendLine($"Constructed graph in {(Time.realtimeSinceStartup - t) * 1000:0.000}ms.");
                    var generators = _roadSystem.GetComponentsInChildren<RoadMeshGenerator>();
                    t = Time.realtimeSinceStartup;
                    foreach (var g in generators)
                        g.GenerateRoadMesh();
                    sb.Append(
                        $"Generated {generators.Length} road meshes in {(Time.realtimeSinceStartup - t) * 1000:0.000}ms.");
                    _meshGenerationTime = sb.ToString();

                    EditorUtility.SetDirty(_roadSystem);
                    SceneView.RepaintAll();
                }

                if (!string.IsNullOrEmpty(_meshGenerationTime))
                    EditorGUILayout.HelpBox(_meshGenerationTime, MessageType.Info);
            }
        }


        private void OnEnable()
        {
            _roadSystem = (RoadSystem)target;
        }
    }
}
