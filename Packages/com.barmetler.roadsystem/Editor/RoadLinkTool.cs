using System;
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEditor.EditorTools;
using UnityEngine;

namespace Barmetler.RoadSystem
{
    [EditorTool("RoadSystem/Road Link Tool")]
    public class RoadLinkTool : EditorTool
    {
        private GUIContent _iconContent;

        public override GUIContent toolbarIcon => _iconContent;

        public static RoadLinkTool ActiveInstance { get; private set; }

        public override void OnActivated()
        {
            _iconContent ??= new GUIContent(EditorGUIUtility.IconContent("Linked@2x"))
            {
                text = "Road Link Tool",
                tooltip = "Used to link and unlink roads from anchor points.",
            };

            ActiveInstance = this;
            Undo.undoRedoPerformed += OnUndoRedo;
            UnityEditor.Selection.activeObject = null;
        }

        public override void OnWillBeDeactivated()
        {
            switch (ActivePoint)
            {
                case AnchorPoint { anchor: { } anchor } pt when anchor.GetConnectedRoad() is { } road:
                {
                    var context = CreateInstance<RoadEditor.RoadSelectionContext>();
                    context.Road = road;
                    context.EndSelected = anchor == road.end;
                    UnityEditor.Selection.SetActiveObjectWithContext(road, context);
                    break;
                }
                case RoadPoint { road: { } road, isStart: var isStart }:
                {
                    var context = CreateInstance<RoadEditor.RoadSelectionContext>();
                    context.Road = road;
                    context.EndSelected = !isStart;
                    UnityEditor.Selection.SetActiveObjectWithContext(road, context);
                    break;
                }
                default:
                    UnityEditor.Selection.activeObject = ActivePoint?.gameObject;
                    break;
            }

            ActiveInstance = null;
            Undo.undoRedoPerformed -= OnUndoRedo;
        }

        private enum ToolState
        {
            SELECTING,
            LINKING,
            UNLINKING
        }

        private ToolState _toolState = ToolState.SELECTING;

        public interface IPoint : IEquatable<IPoint>
        {
            Vector3 position { get; }
            Quaternion rotation { get; }
            GameObject gameObject { get; }
            bool IsConnected { get; }
        }

        public sealed class RoadPoint : IPoint
        {
            public Road road;
            public bool isStart;

            public Vector3 position =>
                road.transform.TransformPoint(isStart ? road[0] : road[-1]);

            public Quaternion rotation =>
                RoadUtilities.GetRotationAtWorldSpace(road, isStart ? 0 : -1) *
                (isStart ? Quaternion.AngleAxis(180, Vector3.up) : Quaternion.identity);

            public GameObject gameObject => road ? road.gameObject : null;

            public bool IsConnected => isStart ? road.start : road.end;

            public bool Equals(IPoint other)
            {
                return (other is RoadPoint otherRoad) && road == otherRoad.road && isStart == otherRoad.isStart;
            }
        }

        public sealed class AnchorPoint : IPoint
        {
            public RoadAnchor anchor;

            public Vector3 position =>
                anchor.transform.position;

            public Quaternion rotation =>
                anchor.transform.rotation;

            public GameObject gameObject => anchor ? anchor.gameObject : null;

            public bool IsConnected => anchor.GetConnectedRoad();

            public bool Equals(IPoint other)
            {
                return other is AnchorPoint otherAnchor && anchor == otherAnchor.anchor;
            }
        }

        public static IPoint ActivePoint { get; private set; }

        public static GameObject Selection => ActivePoint?.gameObject;

        public static void Select(Road road, bool isStart)
        {
            if (isStart ? road.start : road.end)
                ActivePoint = new AnchorPoint { anchor = isStart ? road.start : road.end };
            else
                ActivePoint = new RoadPoint { road = road, isStart = isStart };
        }

        public static void Select(RoadAnchor anchor)
        {
            if (anchor)
                ActivePoint = new AnchorPoint { anchor = anchor };
            else
                ActivePoint = null;
        }

        public override void OnToolGUI(EditorWindow window)
        {
            var e = Event.current;

            if (ActivePoint != null && !ActivePoint.gameObject) ActivePoint = null;

            if (ActivePoint != null && !ActivePoint.IsConnected && e.shift)
                _toolState = ToolState.LINKING;
            else if (e.control && !e.shift)
                _toolState = ToolState.UNLINKING;
            else
                _toolState = ToolState.SELECTING;

            var buttons = new List<IPoint>();

            foreach (var intersection in FindObjectsOfType<Intersection>())
            {
                foreach (var anchor in intersection.AnchorPoints)
                {
                    buttons.Add(new AnchorPoint { anchor = anchor });
                }
            }

            foreach (var road in FindObjectsOfType<Road>())
            {
                if (!road.start)
                    buttons.Add(new RoadPoint { road = road, isStart = true });
                if (!road.end)
                    buttons.Add(new RoadPoint { road = road, isStart = false });
            }

            var activeIsRoad = ActivePoint is RoadPoint;

            buttons = buttons
                .Where(Filter)
                .OrderByDescending(e =>
                    Vector3.Dot(Camera.current.transform.forward, e.position - Camera.current.transform.position))
                .ToList();

            const float size = 1.5f;
            foreach (var point in buttons)
            {
                var position = point.position - point.rotation * (Vector3.forward * size / 2);
                Handles.color = Color.red + 0.7f * Color.white;
                switch (point)
                {
                    case RoadPoint _:
                        Handles.color = Color.cyan;
                        break;
                    case AnchorPoint anchor1 when !anchor1.anchor.GetConnectedRoad():
                        Handles.color = Color.blue;
                        break;
                    case AnchorPoint anchor2 when anchor2.anchor.GetConnectedRoad():
                        position = point.position;
                        break;
                }

                if (_toolState == ToolState.UNLINKING)
                    Handles.color = Color.red * .5f + Color.yellow * .5f;

                if (Handles.Button(position, point.rotation, size, size * 1.5f, Handles.CubeHandleCap))
                {
                    switch (_toolState)
                    {
                        case ToolState.SELECTING:
                            ActivePoint = point;
                            break;

                        case ToolState.LINKING:
                            Link(ActivePoint, point, e.control);
                            break;

                        case ToolState.UNLINKING:
                            Unlink(point);
                            break;
                    }
                }
            }

            switch (_toolState)
            {
                case ToolState.SELECTING:
                case ToolState.LINKING:
                    if (ActivePoint != null)
                    {
                        var position = ActivePoint.position - ActivePoint.rotation * (Vector3.forward * size / 2);
                        if (ActivePoint is AnchorPoint anchor2 && anchor2.anchor.GetConnectedRoad())
                            position = ActivePoint.position;
                        Handles.color = Color.black;
                        Handles.CubeHandleCap(0, position, ActivePoint.rotation, -1.1f * size, EventType.Repaint);
                        Handles.color = Color.red;
                        Handles.CubeHandleCap(0, position, ActivePoint.rotation, size, EventType.Repaint);
                    }

                    break;
            }

            PrintToolTip();
            return;

            bool Filter(IPoint point)
            {
                switch (_toolState)
                {
                    case ToolState.SELECTING:
                        if (point.Equals(ActivePoint))
                            return false;
                        break;

                    case ToolState.LINKING:
                        if (point.Equals(ActivePoint))
                            return false;
                        if (activeIsRoad)
                        {
                            if (point is RoadPoint)
                                return false;
                            if (point is AnchorPoint anchorPoint && anchorPoint.anchor.GetConnectedRoad())
                                return false;
                        }
                        else
                        {
                            if (point is AnchorPoint) return false;
                            if ((ActivePoint as AnchorPoint)?.anchor.GetConnectedRoad())
                                return false;
                        }

                        break;

                    case ToolState.UNLINKING:
                        if (point is RoadPoint)
                            return false;
                        if (!(point as AnchorPoint)?.anchor.GetConnectedRoad())
                            return false;
                        break;
                }

                var viewPos = Camera.current.WorldToViewportPoint(point.position);
                if (Mathf.Abs(viewPos.x - 0.5f) * 2 > 1f) return false;
                if (Mathf.Abs(viewPos.y - 0.5f) * 2 > 1f) return false;
                if (viewPos.z < Camera.current.nearClipPlane + 0.5f) return false;

                return true;
            }
        }

        private void PrintToolTip()
        {
            string text = null;
            switch (_toolState)
            {
                case ToolState.LINKING:
                    text = $"Click to link ({(Event.current.control ? "extend road" : "move endpoint")})";
                    break;

                case ToolState.UNLINKING:
                    text = "Click to unlink";
                    break;
            }

            if (text != null)
            {
                var ray = HandleUtility.GUIPointToWorldRay(Event.current.mousePosition);
                var pos = ray.GetPoint(10) + Camera.current.transform.right * 0.2f;
                Handles.Label(pos, text);
                HandleUtility.Repaint();
            }
        }

        private static void Link(IPoint a, IPoint b, bool extend)
        {
            if (a is RoadPoint && b is AnchorPoint)
            {
                Link(b, a, extend);
                return;
            }

            if (a is AnchorPoint anchor && b is RoadPoint road)
            {
                Undo.SetCurrentGroupName("Link Road");
                var group = Undo.GetCurrentGroup();
                Undo.RecordObject(road.road, "Link Road - road");
                Undo.RecordObject(road.road.GetComponent<MeshFilter>(), "Link Road - mesh");
                Undo.RecordObject(anchor.anchor, "Link Road - anchor");
                if (extend)
                {
                    road.road.AppendSegment(road.road.transform.InverseTransformPoint(anchor.position), road.isStart);
                }

                anchor.anchor.SetRoad(road.road, road.isStart);
                road.road.RefreshEndPoints();
                ActivePoint = anchor;
                Undo.CollapseUndoOperations(group);
            }
            else
            {
                Debug.LogWarning("A road point and an anchor point need to be selected!");
            }
        }

        public static void UnlinkSelected()
        {
            Unlink(ActivePoint);
        }

        private static void Unlink(IPoint point)
        {
            if (ActiveInstance)
            {
                if (point is AnchorPoint anchorPoint && anchorPoint.anchor.GetConnectedRoad())
                {
                    Undo.SetCurrentGroupName("UnLink Road");
                    var group = Undo.GetCurrentGroup();
                    Undo.RecordObject(anchorPoint.anchor.GetConnectedRoad(), "UnLink Road - road");
                    Undo.RecordObject(anchorPoint.anchor.GetConnectedRoad().GetComponent<MeshFilter>(),
                        "UnLink Road - mesh");
                    Undo.RecordObject(anchorPoint.anchor, "UnLink Road - anchor");
                    anchorPoint.anchor.Disconnect();
                    Undo.CollapseUndoOperations(group);
                }
                else
                {
                    Debug.LogWarning("No connected Point selected!");
                }
            }
            else
            {
                Debug.LogWarning("Road Link Tool not active!");
            }
        }

        private static void OnUndoRedo()
        {
            if (ActivePoint is RoadPoint { IsConnected: true } roadPoint)
            {
                ActivePoint = new AnchorPoint
                    { anchor = roadPoint.isStart ? roadPoint.road.start : roadPoint.road.end };
            }
        }
    }
}
