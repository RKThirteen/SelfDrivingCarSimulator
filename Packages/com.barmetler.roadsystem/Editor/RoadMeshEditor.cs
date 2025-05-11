using System.Linq;
using Barmetler.RoadSystem.Util;
using UnityEditor;
using UnityEngine;

namespace Barmetler.RoadSystem
{
    [CustomEditor(typeof(RoadMeshGenerator))]
    public class RoadMeshEditor : Editor
    {
        private RoadMeshGenerator _roadMeshGenerator;

        public override void OnInspectorGUI()
        {
            EditorGUI.BeginChangeCheck();
            base.OnInspectorGUI();
            if (EditorGUI.EndChangeCheck() && _roadMeshGenerator.AutoGenerate)
                _roadMeshGenerator.GenerateRoadMesh();

            GUILayout.Space(10);
            var rect = EditorGUILayout.GetControlRect(false, 1);
            rect.height = 1;
            EditorGUI.DrawRect(rect, new Color(0.5f, 0.5f, 0.5f, 1));
            GUILayout.Space(10);

            var preset = _roadMeshGenerator.settings.SourceOrientation.Preset;
            var options = MeshConversion.MeshOrientation.Presets.Keys.Append("CUSTOM").ToList();
            var selected = options.IndexOf(preset);
            if (selected != options.Count - 1) options.RemoveAt(options.Count - 1);
            var index = EditorGUILayout.Popup(
                new GUIContent(
                    "Source Orientation Preset",
                    "Coordinate space of the model.\n" +
                    "\n" +
                    "If right-handed, like in blender, face orientation is automatically swapped, so that it looks correct."
                ),
                selected, options.ToArray()
            );
            if (index != selected)
            {
                Undo.RecordObject(_roadMeshGenerator, "Change Source Orientation Preset");
                _roadMeshGenerator.settings.SourceOrientation.Preset = options[index];
                if (_roadMeshGenerator.AutoGenerate) _roadMeshGenerator.GenerateRoadMesh();
            }

            GUILayout.BeginHorizontal();
            GUILayout.Label(
                new GUIContent(
                    "Auto Generate",
                    "Automatically generate road mesh when something changes."
                ),
                GUILayout.Width(EditorGUIUtility.labelWidth)
            );
            var autoGenerate = GUILayout.Toggle(_roadMeshGenerator.AutoGenerate, "");
            if (autoGenerate != _roadMeshGenerator.AutoGenerate)
            {
                Undo.RecordObject(_roadMeshGenerator, "Toggle Auto Generate");
                _roadMeshGenerator.AutoGenerate = autoGenerate;
            }

            GUILayout.EndHorizontal();

            GUILayout.Space(10);
            if (GUILayout.Button(new GUIContent("Generate Mesh", ""), GUILayout.Height(50)))
            {
                Undo.RecordObject(_roadMeshGenerator.gameObject, "Generate Mesh");
                _roadMeshGenerator.GenerateRoadMesh();
            }
        }

        private void OnEnable()
        {
            _roadMeshGenerator = (RoadMeshGenerator)target;
        }
    }
}
