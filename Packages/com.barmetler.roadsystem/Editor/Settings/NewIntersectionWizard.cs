using UnityEditor;
using UnityEngine;

namespace Barmetler.RoadSystem.Settings
{
    public class NewIntersectionWizard : ScriptableWizard
    {
        [MenuItem("Tools/RoadSystem/Create Intersection Wizard", priority = 3)]
        public static void CreateWizard()
        {
            DisplayWizard<NewIntersectionWizard>("Create Intersection", "Create", "Apply");
        }

        private GameObject _intersection;

        private void OnEnable()
        {
            minSize = new Vector2(350, 200);
            helpString =
                "Select a prefab for the new intersection! You can also set that prefab in [Project Settings/MB RoadSystem]";
            _intersection = RoadSystemSettings.Instance.NewIntersectionPrefab;
        }

        protected override bool DrawWizardGUI()
        {
            EditorGUI.BeginChangeCheck();

            EditorGUILayout.BeginHorizontal();
            GUILayout.Label("Prefab", GUILayout.Width(EditorGUIUtility.labelWidth));
            _intersection = EditorGUILayout.ObjectField(_intersection, typeof(GameObject), false) as GameObject;
            EditorGUILayout.EndHorizontal();

            return EditorGUI.EndChangeCheck();
        }

        private void OnWizardCreate()
        {
            OnWizardOtherButton();
            RoadMenu.CreateIntersection();
        }

        private void OnWizardOtherButton()
        {
            RoadSystemSettings.Instance.NewIntersectionPrefab = _intersection;
        }
    }
}
