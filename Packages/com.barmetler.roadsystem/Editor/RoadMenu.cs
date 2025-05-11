using Barmetler.RoadSystem.Settings;
using UnityEditor;
using UnityEditor.EditorTools;
using UnityEngine;

namespace Barmetler.RoadSystem
{
    public class RoadMenu : MonoBehaviour
    {
        private static RoadEditor ActiveEditor => RoadEditor.GetEditor(Selection.activeGameObject);

        #region Validation

        public static bool MenuRoadIsSelected() =>
            ActiveEditor;

        [MenuItem("Tools/RoadSystem/Remove Point [backspace]", validate = true)]
        public static bool MenuPointIsSelected() =>
            ActiveEditor is { } editor &&
            editor.SelectedAnchorPoint != -1;

        public static bool MenuEndPointIsSelected() =>
            ActiveEditor is { } editor &&
            editor.IsEndPoint(editor.SelectedAnchorPoint);

        [MenuItem("Tools/RoadSystem/Unlink Point", validate = true)]
        public static bool MenuEndPointIsSelectedAndConnected() =>
            ActiveEditor is { } editor &&
            editor.IsEndPoint(editor.SelectedAnchorPoint, YesNoMaybe.YES);

        [MenuItem("Tools/RoadSystem/Extrude", validate = true)]
        public static bool MenuEndPointIsSelectedAndNotConnected() =>
            ActiveEditor is { } editor &&
            editor.IsEndPoint(editor.SelectedAnchorPoint, YesNoMaybe.NO);

        #endregion Validation

        #region Menus

        [MenuItem("Tools/RoadSystem/Create Road System", priority = 1)]
        public static void CreateRoadSystem()
        {
            var selected = Selection.activeGameObject;
            Transform parent;

            if (!selected)
                parent = null;
            else if (selected.GetComponent<Road>())
                parent = selected.transform.parent;
            else if (selected.GetComponentInParent<Intersection>())
                parent = selected.GetComponentInParent<Intersection>().transform.parent;
            else if (selected.GetComponentInParent<RoadSystem>())
                parent = selected.GetComponentInParent<RoadSystem>().transform.parent;
            else
                parent = selected.transform;

            var newObject = new GameObject("RoadSystem");
            newObject.AddComponent<RoadSystem>();

            GameObjectUtility.SetParentAndAlign(newObject, parent ? parent.gameObject : null);

            Undo.RegisterCreatedObjectUndo(newObject, "Create new Road System");

            Selection.activeGameObject = newObject;
        }

        /// <summary>
        /// If road is selected, connect intersection to closest end           <br/>
        /// If intersection is selected, create intersection on the same level <br/>
        /// If anchorPoint is selected, create intersection at level above     <br/>
        /// If RoadSystem is selected, create intersection under it            <br/>
        /// <br/>
        /// Use Prefab defined in RoadSystemSettings
        /// </summary>
        [MenuItem("Tools/RoadSystem/Create Intersection", priority = 2)]
        public static void CreateIntersection()
        {
            var selected = Selection.activeGameObject;
            if (RoadLinkTool.ActiveInstance && RoadLinkTool.Selection)
                selected = RoadLinkTool.Selection;

            Transform parent;
            GameObject newObject;
            Intersection intersection = null;

            if (RoadSystemSettings.Instance.NewIntersectionPrefab)
            {
                newObject = Instantiate(RoadSystemSettings.Instance.NewIntersectionPrefab);
                intersection = newObject.GetComponent<Intersection>();
            }
            else
            {
                newObject = new GameObject("Intersection");
            }

            if (!intersection)
                newObject.AddComponent<Intersection>();

            if (!selected)
                parent = null;
            else if (selected.GetComponent<Road>())
                parent = selected.transform.parent;
            else if (selected.GetComponentInParent<Intersection>())
                parent = selected.GetComponentInParent<Intersection>().transform.parent;
            else
                parent = selected.transform;

            if (selected?.GetComponent<Road>() is { } road)
            {
                var isStart = RoadLinkTool.ActiveInstance
                    ? (RoadLinkTool.ActivePoint as RoadLinkTool.RoadPoint)?.isStart ?? false
                    : RoadEditor.GetEditor(selected).SelectedAnchorPoint <= road.NumSegments / 2;
                if (!(isStart ? road.start : road.end) &&
                    newObject.GetComponentInChildren<RoadAnchor>() is { } anchor)
                {
                    newObject.transform.parent = parent;
                    var position = road.transform.TransformPoint(isStart ? road[0] : road[-1]);
                    var normal =
                        road.transform.TransformDirection(
                            isStart ? road.GetNormal(0) : road.GetNormal(road.NumSegments));
                    var forward =
                        road.transform.TransformDirection(isStart
                            ? (road[1] - road[0]).normalized
                            : (road[-2] - road[-1]).normalized);
                    var orientation = Quaternion.LookRotation(forward, normal);
                    var relative = newObject.transform.localToWorldMatrix * anchor.transform.worldToLocalMatrix;
                    var targetOrientation = Quaternion.LookRotation(relative.GetColumn(2), relative.GetColumn(1)) *
                                            orientation;
                    newObject.transform.rotation = targetOrientation;
                    newObject.transform.position =
                        newObject.transform.TransformPoint(anchor.transform.InverseTransformPoint(position));
                    if (isStart) road.start = anchor;
                    else road.end = anchor;
                    road.RefreshEndPoints();
                    if (RoadLinkTool.ActiveInstance)
                    {
                        RoadLinkTool.Select(isStart ? road.start : road.end);
                    }
                }
                else
                {
                    GameObjectUtility.SetParentAndAlign(newObject, parent ? parent.gameObject : null);
                }
            }
            else
            {
                GameObjectUtility.SetParentAndAlign(newObject, parent ? parent.gameObject : null);
            }

            Undo.RegisterCreatedObjectUndo(newObject, "Create new Intersection");

            if (!RoadLinkTool.ActiveInstance)
                Selection.activeGameObject = newObject;
        }

        /// <summary>
        /// If road is selected, create new road on the same level                  <br/>
        /// If intersection is selected, create road on the same level              <br/>
        /// If anchorPoint is selected, and it is free, create road connected to it <br/>
        /// If RoadSystem is selected, create road under it                         <br/>
        /// <br/>
        /// Use Prefab defined in RoadSystemSettings
        /// </summary>
        [MenuItem("Tools/RoadSystem/Create Road", priority = 2)]
        public static void CreateRoad()
        {
            var selected = Selection.activeGameObject;
            if (RoadLinkTool.ActiveInstance && RoadLinkTool.Selection)
                selected = RoadLinkTool.Selection;

            Transform parent;
            GameObject newObject;
            Road road = null;

            if (RoadSystemSettings.Instance.NewRoadPrefab)
            {
                newObject = Instantiate(RoadSystemSettings.Instance.NewRoadPrefab);
                // ReSharper disable once AssignmentInConditionalExpression
                if (road = newObject.GetComponent<Road>())
                    road.start = road.end = null;
            }
            else
            {
                newObject = new GameObject("Road");
            }

            if (!road)
                road = newObject.AddComponent<Road>();

            if (!selected)
                parent = null;
            else if (selected.GetComponent<Road>())
                parent = selected.transform.parent;
            else if (selected.GetComponentInParent<Intersection>())
                parent = selected.GetComponentInParent<Intersection>().transform.parent;
            else
                parent = selected.transform;

            GameObjectUtility.SetParentAndAlign(newObject, parent ? parent.gameObject : null);

            var selectionContext = ScriptableObject.CreateInstance<RoadEditor.RoadSelectionContext>();
            selectionContext.Road = road;

            if (selected && selected.GetComponent<RoadAnchor>() is { } anchor && !anchor.GetConnectedRoad())
            {
                road.start = anchor;
                road.RefreshEndPoints();
                var n = (road[1] - road[0]).normalized;
                foreach (var i in new[] { 1, 3, 2 })
                    road.MovePoint(i, road[0] + i * n);
                road.MoveNormal(1, road.GetNormal(0));
                selectionContext.EndSelected = true;
            }

            Undo.RegisterCreatedObjectUndo(newObject, "Create new Road");

            if (newObject.GetComponent<RoadMeshGenerator>() is { } roadMeshGenerator)
                roadMeshGenerator.GenerateRoadMesh();

            if (RoadLinkTool.ActiveInstance)
            {
                RoadLinkTool.Select(road, isStart: selectionContext.EndSelected != true);
            }
            else
            {
                Selection.SetActiveObjectWithContext(newObject, selectionContext);
            }
        }

        [MenuItem("Tools/RoadSystem/Extrude %#e")]
        public static void MenuExtrude()
        {
            if (MenuEndPointIsSelectedAndNotConnected())
            {
                ActiveEditor.ExtrudeSelected();
            }
            else
            {
                Debug.LogError("No road endpoint selected!");
            }
        }

        [MenuItem("Tools/RoadSystem/Remove Point [backspace]")]
        public static void MenuRemove()
        {
            if (MenuPointIsSelected())
            {
                ActiveEditor.RemoveSelected();
            }
            else
            {
                Debug.LogError("No Road selected!");
            }
        }

        [MenuItem("Tools/RoadSystem/Unlink Point %u")]
        public static void MenuUnlink()
        {
            if (RoadLinkTool.ActiveInstance)
            {
                RoadLinkTool.UnlinkSelected();
            }
            else if (MenuEndPointIsSelectedAndConnected())
            {
                ActiveEditor.UnlinkSelected();
            }
            else
            {
                Debug.LogError("No connected Endpoint selected!");
            }
        }

        [MenuItem("Tools/RoadSystem/Link Points %l")]
        public static void MenuLink()
        {
            if (MenuEndPointIsSelected())
            {
                ToolManager.SetActiveTool<RoadLinkTool>();
            }

            ToolManager.SetActiveTool<RoadLinkTool>();
        }

        #endregion Menus
    }
}
