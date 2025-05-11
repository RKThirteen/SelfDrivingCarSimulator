using System;
using Barmetler.RoadSystem.Settings;
using UnityEditor;
using UnityEngine;

namespace Barmetler.RoadSystem
{
    [CustomEditor(typeof(RoadSystemNavigator))]
    public class RoadSystemNavigatorEditor : Editor
    {
        private RoadSystemNavigator _navigator;
        private RoadSystemSettings _settings;

        private void OnSceneGUI()
        {
            if (!Application.isPlaying && _navigator.transform.hasChanged)
            {
                UpdateNavigator();
                _navigator.transform.hasChanged = false;
            }

            Draw();
        }

        private void UpdateNavigator()
        {
            if (!_settings.AutoCalculateNavigator) return;

            try
            {
                _navigator.CalculateWayPointsSync();
            }
            catch (Exception e)
            {
                Debug.LogError(e);
            }

            SceneView.RepaintAll();
        }

        private void Draw()
        {
            if (!_settings.DrawNavigatorDebug) return;

            var points = _navigator.CurrentPoints;

            Vector3 position;
            var lastPos = _navigator.transform.position;
            Handles.color = Color.yellow;
            foreach (var point in points)
            {
                position = point.position;
                Handles.DrawLine(lastPos, position);
                lastPos = position;
            }

            position = _navigator.Goal;
            Handles.DrawLine(lastPos, position);

            {
                var d1 = _navigator.GetMinDistance(out _, out var p1, out _);
                var d2 = _navigator.GetMinDistance(out _, out _, out var p2, out _);
                var p = d1 < d2 ? p1 : p2;
                Handles.SphereHandleCap(0, p, Quaternion.identity, 0.5f, EventType.Repaint);
            }

            if (_settings.DrawNavigatorDebugPoints)
            {
                foreach (var point in points)
                {
                    Handles.SphereHandleCap(0, point.position, Quaternion.identity, 0.2f, EventType.Repaint);
                }
            }
        }

        public override void OnInspectorGUI()
        {
            EditorGUI.BeginChangeCheck();

            base.OnInspectorGUI();

            var drawDebug = GUILayout.Toggle(_settings.DrawNavigatorDebug, "Draw Navigator Debug Info");
            if (drawDebug != _settings.DrawNavigatorDebug)
            {
                Undo.RecordObject(_settings, "Toggle Draw Navigator Debug Info");
                _settings.DrawNavigatorDebug = drawDebug;
            }

            var drawDebugPoints = GUILayout.Toggle(_settings.DrawNavigatorDebugPoints, "Draw Navigator Debug Points");
            if (drawDebugPoints != _settings.DrawNavigatorDebugPoints)
            {
                Undo.RecordObject(_settings, "Toggle Draw Navigator Debug Points");
                _settings.DrawNavigatorDebugPoints = drawDebugPoints;
            }

            var autoCalculate = GUILayout.Toggle(_settings.AutoCalculateNavigator, "Auto Calculate Navigator");
            if (autoCalculate != _settings.AutoCalculateNavigator)
            {
                Undo.RecordObject(_settings, "Toggle Auto Calculate Navigator");
                _settings.AutoCalculateNavigator = autoCalculate;
            }

            if (GUILayout.Button("Calculate WayPoints"))
            {
                _navigator.CalculateWayPointsSync();
                SceneView.RepaintAll();
            }

            if (EditorGUI.EndChangeCheck())
            {
                UpdateNavigator();
            }
        }

        private void OnEnable()
        {
            _navigator = (RoadSystemNavigator)target;
            _settings = RoadSystemSettings.Instance;
            UpdateNavigator();
        }
    }
}
