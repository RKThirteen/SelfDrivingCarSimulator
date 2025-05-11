using UnityEditor;
using UnityEngine;

namespace Barmetler.RoadSystem.Settings
{
    public class NewRoadWizard : ScriptableWizard
    {
        [MenuItem("Tools/RoadSystem/Create Road Wizard", priority = 3)]
        public static void CreateWizard()
        {
            DisplayWizard<NewRoadWizard>("Create Road", "Create", "Apply");
        }

        private GameObject _road;

        private void OnEnable()
        {
            minSize = new Vector2(350, 200);
            helpString =
                "Select a prefab for the new road! You can also set that prefab in [Project Settings/MB RoadSystem]";
            _road = RoadSystemSettings.Instance.NewRoadPrefab;
        }

        protected override bool DrawWizardGUI()
        {
            EditorGUI.BeginChangeCheck();

            EditorGUILayout.BeginHorizontal();
            GUILayout.Label("Prefab", GUILayout.Width(EditorGUIUtility.labelWidth));
            _road = EditorGUILayout.ObjectField(_road, typeof(GameObject), false) as GameObject;
            EditorGUILayout.EndHorizontal();

            return EditorGUI.EndChangeCheck();
        }

        private void OnWizardCreate()
        {
            OnWizardOtherButton();
            RoadMenu.CreateRoad();
        }

        private void OnWizardOtherButton()
        {
            RoadSystemSettings.Instance.NewRoadPrefab = _road;
        }
    }
}
