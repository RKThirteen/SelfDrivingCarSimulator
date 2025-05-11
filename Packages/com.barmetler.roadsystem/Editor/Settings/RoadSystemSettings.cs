using System;
using System.IO;
using UnityEditor;
using UnityEngine;

namespace Barmetler.RoadSystem.Settings
{
    [CreateAssetMenu(fileName = "RoadSystemSettings", menuName = "Barmetler/RoadSystemSettings")]
    public class RoadSystemSettings : ScriptableObject
    {
        [Serializable]
        private class RoadSettings
        {
            [Tooltip("Draw bounding boxes around bezier segments?")]
            public bool drawBoundingBoxes;

            [Tooltip(
                "When extending the road, whether to place it at the intersection of the mouse with the scene's geometry.")]
            public bool useRayCast = true;

            [Tooltip(
                "If useRayCast is enabled, should the new road segment copy the surface normal of the intersection?")]
            public bool copyHitNormal;

            [Tooltip("The Prefab to use when creating a new road.")]
            public GameObject newRoadPrefab;
        }

        [Serializable]
        private class IntersectionSettings
        {
            [Tooltip("The Prefab to use when creating a new intersection.")]
            public GameObject newIntersectionPrefab;
        }

        [SerializeField]
        private RoadSettings roadSettings = new RoadSettings();

        [SerializeField]
        private IntersectionSettings intersectionSettings = new IntersectionSettings();

        [SerializeField]
        private bool drawNavigatorDebug;

        [SerializeField]
        private bool drawNavigatorDebugPoints;

        [SerializeField]
        private bool autoCalculateNavigator;

        public bool DrawBoundingBoxes => roadSettings.drawBoundingBoxes;
        public bool UseRayCast => roadSettings.useRayCast;
        public bool CopyHitNormal => roadSettings.copyHitNormal;

        public GameObject NewRoadPrefab
        {
            get => roadSettings.newRoadPrefab;
            set
            {
                roadSettings.newRoadPrefab = value;
                EditorUtility.SetDirty(this);
            }
        }

        public GameObject NewIntersectionPrefab
        {
            get => intersectionSettings.newIntersectionPrefab;
            set
            {
                intersectionSettings.newIntersectionPrefab = value;
                EditorUtility.SetDirty(this);
            }
        }

        public bool DrawNavigatorDebug
        {
            get => drawNavigatorDebug;
            set
            {
                drawNavigatorDebug = value;
                EditorUtility.SetDirty(this);
            }
        }

        public bool DrawNavigatorDebugPoints
        {
            get => drawNavigatorDebugPoints;
            set
            {
                drawNavigatorDebugPoints = value;
                EditorUtility.SetDirty(this);
            }
        }

        public bool AutoCalculateNavigator
        {
            get => autoCalculateNavigator;
            set
            {
                autoCalculateNavigator = value;
                EditorUtility.SetDirty(this);
            }
        }

        private const string SettingsPath = "Assets/Settings/Editor/RoadSystemSettings.asset";

        private static RoadSystemSettings _instance;

        public static RoadSystemSettings Instance
        {
            get
            {
                if (_instance == null)
                    _instance = AssetDatabase.LoadAssetAtPath<RoadSystemSettings>(SettingsPath);
                if (_instance != null) return _instance;
                _instance = CreateInstance<RoadSystemSettings>();
                Directory.CreateDirectory("Assets/Settings/Editor");
                AssetDatabase.CreateAsset(_instance, SettingsPath);
                AssetDatabase.SaveAssets();
                return _instance;
            }
        }

        internal static SerializedObject SerializedInstance => new SerializedObject(Instance);
    }
}
