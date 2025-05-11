using System;
using System.Collections.Generic;
using System.Linq;
using Barmetler.RoadSystem.Settings;
using UnityEditor;
using UnityEngine;
using Object = UnityEngine.Object;

#pragma warning disable CS0618

namespace Barmetler.RoadSystem
{
    [CustomEditor(typeof(Road))]
    public class RoadEditor : Editor
    {
        private static readonly HashSet<RoadEditor> ActiveEditors = new HashSet<RoadEditor>();

        public static RoadEditor GetEditor(GameObject gameObject) =>
            ActiveEditors.FirstOrDefault(e => ((Road)e.target).gameObject == gameObject);

        private Road _road;
        private RoadSystemSettings _settings;
        private SerializedObject _settingsSerialized;
        private Tool _lastTool;

        private int _selectedAnchorPoint = -1;

        public int SelectedAnchorPoint => Tools.current == Tool.Custom ? -1 : _selectedAnchorPoint;

        private bool _rightMouseDown;

        private void OnUndoRedo()
        {
            _road.OnCurveChanged(true);
        }

        private void OnEnable()
        {
            ActiveEditors.Add(this);
            _road = (Road)target;
            _settings = RoadSystemSettings.Instance;
            _settingsSerialized = new SerializedObject(_settings);
            _road.RefreshEndPoints();
            UpdateToolVisibility();
            Undo.undoRedoPerformed += OnUndoRedo;
            if (Selection.activeContext is RoadSelectionContext { EndSelected: { } endSelected })
                _selectedAnchorPoint = endSelected ? _road.NumPoints - 1 : 0;
        }

        private void OnDisable()
        {
            Tools.hidden = false;
            Undo.undoRedoPerformed -= OnUndoRedo;
            ActiveEditors.Remove(this);
            RoadLinkTool.Select(_road, _selectedAnchorPoint <= _road.NumPoints / 2);
        }

        private void OnSceneGUI()
        {
            _rightMouseDown = Event.current.type switch
            {
                EventType.MouseDown when Event.current.button == 1 => true,
                EventType.MouseUp when Event.current.button == 1 => false,
                _ => _rightMouseDown
            };

            var controlID = GUIUtility.GetControlID(FocusType.Passive);

            if (_selectedAnchorPoint >= _road.NumPoints) _selectedAnchorPoint = _road.NumPoints - 1;

            if (_road.transform.hasChanged)
            {
                _road.RefreshEndPoints();
                _road.transform.hasChanged = false;
            }

            UpdateToolVisibility();

            DrawInfo(controlID);
            GUIControlPoints(controlID);
            GUIAddOrRemovePoints(controlID);

            GUIDrawWindow();
        }

        private void UpdateToolVisibility()
        {
            switch (Tools.current)
            {
                case Tool.Move:
                case Tool.Rotate:
                case Tool.Scale:
                    Tools.hidden = _selectedAnchorPoint != -1;
                    break;
                case Tool.Rect:
                    Tools.hidden = true;
                    break;
                case Tool.Custom:
                    // Enable tools if you switch to custom tool, but from then on let the custom tool manage tool visibility
                    if (_lastTool != Tool.Custom)
                        Tools.hidden = false;
                    break;
                default:
                    Tools.hidden = false;
                    break;
            }

            _lastTool = Tools.current;
        }

        private void DrawInfo(int controlID)
        {
            if (_settings.DrawBoundingBoxes)
            {
                RSHandleUtility.DrawBoundingBoxes(_road);
            }

            var points = _road.GetEvenlySpacedPoints(1, 1).Select(e => e.ToWorldSpace(_road.transform)).ToArray();
            var lastPos = Vector3.zero;
            Handles.color = Color.green * 0.8f;
            for (var i = 0; i < points.Length; ++i)
            {
                var p = points[i];
                if (i > 0)
                    Handles.DrawLine(lastPos, p.position);
                lastPos = p.position;
            }

            foreach (var p in points)
            {
                Handles.color = Color.blue * 0.8f;
                Handles.SphereHandleCap(0, p.position, Quaternion.identity, 0.2f, EventType.Repaint);
                Handles.color = Color.yellow * 0.8f;
                Handles.DrawLine(p.position, p.position + p.normal);
                // Handles.color = Color.red * 0.8f;
                // Handles.DrawLine(p.position, p.position + p.forward);
            }
        }

        private readonly Dictionary<int, Quaternion> _initialRotations = new Dictionary<int, Quaternion>();

        private void GUIControlPoints(int controlID)
        {
            var e = Event.current;
            var hasModifiers = (e.alt || e.shift || e.control);

            if (e.type == EventType.KeyDown && e.keyCode == KeyCode.Escape)
                _selectedAnchorPoint = -1;

            switch (Tools.current)
            {
                case Tool.Move:
                case Tool.Rotate:
                case Tool.Scale:
                    // Draw selection Handles
                    var points = Enumerable
                        .Range(0, _road.NumSegments + 1)
                        .Select(e => (index: e * 3, pos: _road.transform.TransformPoint(_road[e * 3])))
                        .OrderByDescending(e =>
                            Vector3.Dot(Camera.current.transform.forward, e.pos - Camera.current.transform.position))
                        .ToList();

                    Handles.color = hasModifiers ? Color.grey : Color.red * 0.8f;
                    foreach (var p in points.Where(p => _selectedAnchorPoint != p.index))
                    {
                        if (hasModifiers)
                            Handles.SphereHandleCap(0, p.pos, Quaternion.identity,
                                0.2f * HandleUtility.GetHandleSize(p.pos), EventType.Repaint);
                        else if (Handles.Button(p.pos, Quaternion.identity,
                                     0.3f * HandleUtility.GetHandleSize(p.pos),
                                     0.3f * HandleUtility.GetHandleSize(p.pos),
                                     Handles.SphereHandleCap))
                            _selectedAnchorPoint = p.index;
                    }

                    break;
            }

            var pos = Vector3.zero;
            var forward = Vector3.forward;
            var rot = Quaternion.identity;
            if (_selectedAnchorPoint != -1)
            {
                pos = _road.transform.TransformPoint(_road[_selectedAnchorPoint]);
                rot = RoadUtilities.GetRotationAtWorldSpace(_road, _selectedAnchorPoint, out forward, out _);
            }

            if (e.control) return;
            switch (Tools.current)
            {
                case Tool.Move:
                    if (_selectedAnchorPoint != -1)
                    {
                        var newPos = Handles.PositionHandle(pos,
                            Tools.pivotRotation == PivotRotation.Local ? rot : Quaternion.identity);
                        if (newPos != pos)
                        {
                            Undo.RecordObject(_road, "Move Control Point");
                            _road.MovePoint(_selectedAnchorPoint, _road.transform.InverseTransformPoint(newPos));
                        }
                    }

                    break;

                case Tool.Rotate:
                    if (_selectedAnchorPoint != -1)
                    {
                        var hc = GUIUtility.hotControl;
                        var newRot =
                            Handles.RotationHandle(
                                Tools.pivotRotation == PivotRotation.Local ? rot : Quaternion.identity, pos);
                        if (hc != GUIUtility.hotControl)
                        {
                            _initialRotations[GUIUtility.hotControl] = rot;
                        }

                        if ((Tools.pivotRotation == PivotRotation.Global && newRot != Quaternion.identity) ||
                            (Tools.pivotRotation == PivotRotation.Local && newRot != rot))
                        {
                            if (Tools.pivotRotation == PivotRotation.Global)
                            {
                                if (GUIUtility.hotControl == 1317) // 1317
                                    newRot *= rot;
                                else
                                    newRot *= _initialRotations[GUIUtility.hotControl];
                            }

                            Undo.RecordObject(_road, "Rotate Control Point");
                            RoadUtilities.SetRotationAtWorldSpace(_road, _selectedAnchorPoint, newRot);
                        }
                    }

                    break;

                case Tool.Scale:
                    if (_selectedAnchorPoint != -1)
                    {
                        Handles.color = hasModifiers ? Color.grey : Color.white * 0.7f;
                        Handles.SphereHandleCap(0, pos, Quaternion.identity,
                            (hasModifiers ? 0.2f : 0.3f) * HandleUtility.GetHandleSize(pos), EventType.Repaint);
                        Handles.color = Color.red + Color.white * 0.4f;
                        for (var i = -1; i <= 1; i += 2)
                        {
                            var j = _selectedAnchorPoint + i;
                            if (j < 0 || j >= _road.NumPoints) continue;
                            var hPos = _road.transform.TransformPoint(_road[j]);

                            Handles.color = hasModifiers ? Color.grey : Color.red;
                            Handles.DrawLine(pos, hPos);

                            Handles.color = hasModifiers ? Color.grey : Color.red + Color.white * 0.4f;
                            if (hasModifiers)
                            {
                                Handles.SphereHandleCap(0, hPos, Quaternion.identity,
                                    0.2f * HandleUtility.GetHandleSize(pos), EventType.Repaint);
                            }
                            else
                            {
                                var nPos = Handles.FreeMoveHandle(hPos, Quaternion.identity,
                                    0.3f * HandleUtility.GetHandleSize(pos), Vector3.zero, Handles.SphereHandleCap);

                                if (hPos != nPos)
                                {
                                    Undo.RecordObject(_road, "Scale Control Point");
                                    var dot = Vector3.Dot(forward, nPos - pos);
                                    if (i == -1) dot = Mathf.Min(dot, -0.1f);
                                    else dot = Mathf.Max(dot, 0.1f);
                                    _road.MovePoint(j, _road.transform.InverseTransformPoint(pos + forward * dot));
                                }
                            }
                        }
                    }

                    break;

                case Tool.Rect:

                    Handles.color = Color.black;
                    for (var i = 0; i < _road.NumPoints; i += 3)
                    {
                        var p = _road.transform.TransformPoint(_road[i]);

                        if (i > 0)
                        {
                            var p2 = _road.transform.TransformPoint(_road[i - 1]);
                            Handles.DrawLine(p, p2);
                        }

                        if (i < _road.NumPoints - 1)
                        {
                            var p2 = _road.transform.TransformPoint(_road[i + 1]);
                            Handles.DrawLine(p, p2);
                        }
                    }

                    var points = Enumerable
                        .Range(0, _road.NumPoints)
                        .Select(e => (index: e, pos: _road.transform.TransformPoint(_road[e])))
                        .OrderByDescending(e =>
                            Vector3.Dot(Camera.current.transform.forward, e.pos - Camera.current.transform.position))
                        .ToList();

                    foreach (var p in points)
                    {
                        var c = ((p.index + 1) / 3 * 3) == _selectedAnchorPoint
                            ? (0.7f * Color.cyan + 0.3f * Color.black)
                            : Color.red;
                        Handles.color = e.alt ? Color.grey : (p.index % 3 == 0 ? c : (c + Color.white * 0.4f));
                        if (e.alt || e.shift || e.control)
                        {
                            Handles.SphereHandleCap(0, p.pos, Quaternion.identity,
                                0.2f * HandleUtility.GetHandleSize(p.pos), EventType.Repaint);
                        }
                        else
                        {
                            var newPos = Handles.FreeMoveHandle($"Handle-{p.index}".GetHashCode(), p.pos,
                                Quaternion.identity,
                                (p.index % 3 == 0 ? 0.3f : 0.25f) * HandleUtility.GetHandleSize(p.pos),
                                Vector3.zero, Handles.SphereHandleCap);

                            if (p.pos != newPos)
                            {
                                _selectedAnchorPoint = (p.index + 1) / 3 * 3;
                                Undo.RecordObject(_road, "Move Control Point");
                                _road.MovePoint(p.index, _road.transform.InverseTransformPoint(newPos));
                            }
                        }
                    }

                    break;
            }
        }

        private readonly Dictionary<KeyCode, bool> _wasDown = new Dictionary<KeyCode, bool>();

        private bool WasDown(KeyCode keyCode)
        {
            if (_wasDown.TryGetValue(keyCode, out var down))
                return down;

            return _wasDown[keyCode] = false;
        }

        private void GUIAddOrRemovePoints(int controlID)
        {
            var e = Event.current;

            if (_rightMouseDown) return;
            switch (Tools.current)
            {
                case Tool.Move:
                case Tool.Rotate:
                case Tool.Scale:
                case Tool.Rect:
                    break;
                default:
                    return;
            }

            // ===============================
            // =========[ Extrusion ]=========
            // ===============================

            if (e.type == EventType.KeyDown)
            {
                var keyCode = e.keyCode;
                if (e.control && !e.alt && !e.shift && keyCode == KeyCode.E && !WasDown(KeyCode.E))
                {
                    if (Extrude(ref _selectedAnchorPoint))
                        e.Use();
                }
                else if (!e.control && !e.alt && !e.shift && keyCode == KeyCode.Backspace &&
                         !WasDown(KeyCode.Backspace))
                {
                    if (RemoveSelected())
                        e.Use();
                }

                _wasDown[keyCode] = true;
            }
            else if (e.type == EventType.KeyUp)
            {
                _wasDown[e.keyCode] = false;
            }

            // ===============================
            // =====[ Segment Insertion ]=====
            // ===============================

            if (e.shift && !e.alt && !e.control)
            {
                var minDist = float.PositiveInfinity;
                var segmentIndex = -1;
                var segment = new Vector3[] { };
                var segmentNormals = new Vector3[] { };

                for (var seg = 0; seg < _road.NumSegments; ++seg)
                {
                    var orientedPoints = Bezier.GetEvenlySpacedPoints(
                        _road.GetPointsInSegment(seg),
                        new List<Vector3> { _road.GetNormal(seg), _road.GetNormal(seg + 1) }, 1
                    ).Select(e => e.ToWorldSpace(_road.transform)).ToArray();
                    if (orientedPoints.Length == 0) continue;
                    var v = orientedPoints.Select(e => e.position).ToArray();
                    var d = v.Length == 1
                        ? HandleUtility.DistanceToCircle(v[0], 0)
                        : HandleUtility.DistanceToPolyLine(v);
                    if (!(d < minDist)) continue;
                    minDist = d;
                    segmentIndex = seg;
                    segment = v;
                    segmentNormals = orientedPoints.Select(e => e.normal).ToArray();
                }

                if (segmentIndex != -1)
                {
                    var hoverPos = ClosestPointToPolyLine(segment, out var polyIndex, out var polyT);

                    var s = _road
                        .GetPointsInSegment(segmentIndex)
                        .Select(t => _road.transform.TransformPoint(t))
                        .ToArray();
                    // get t value of the closest point on the segment
                    var t = Bezier.InverseCubic(s[0], s[1], s[2], s[3], hoverPos);
                    var normal = Vector3.Lerp(
                        segmentNormals[polyIndex],
                        segmentNormals[Mathf.Min(polyIndex + 1, segmentNormals.Length - 1)], polyT
                    ).normalized;

                    var tooClose = t < 1e-6 || t > 1 - 1e-6;

                    Handles.color = tooClose ? Color.grey : Color.white;
                    Handles.SphereHandleCap(0, hoverPos, Quaternion.identity,
                        0.2f * HandleUtility.GetHandleSize(hoverPos), EventType.Repaint);
                    Handles.color = Color.red;
                    Handles.DrawLine(hoverPos, hoverPos + normal * (0.5f * HandleUtility.GetHandleSize(hoverPos)));
                    if (tooClose)
                        Handles.Label(hoverPos, "Too Close!", new GUIStyle
                        {
                            normal =
                            {
                                textColor = Color.red,
                                background = Texture2D.whiteTexture,
                            }
                        });
                    else if (e.type == EventType.MouseDown && e.button == 0)
                    {
                        Undo.RecordObject(_road, "Insert Segment");
                        _road.InsertSegment(segmentIndex, t, normal);
                        _selectedAnchorPoint = segmentIndex * 3 + 3;
                    }
                }

                // If you click off the road, or the road itself, the selection system will deselect it.
                if (e.type == EventType.Used)
                {
                    Selection.activeObject = _road;
                }

                if (Event.current.type == EventType.MouseMove) SceneView.RepaintAll();
            }

            // ===============================
            // =====[ Segment Extension ]=====
            // ===============================

            /*
             * Behavior:
             * - You can't extend from an endpoint that is connected to an anchor.
             * - If an end point is selected, the extension will be made from there.
             * - Otherwise, the closest endpoint will be chosen.
             * - Selection of coordinates and normal:
             *   1. If shift is held, the mouse ray is intersected with a plane at the end point.
             *      The plane's up vector depends on Tools.pivotRotation, it is essentially the green arrow of the translation-gizmo.
             *      The intersection point has to be within 500 units of the camera.
             *   2. If the mouse position intersects with a collider within a distance of 500 units, that point is chosen.
             *   3. Otherwise, the point at the same depth as the end point is used.
             */

            if (e.control && !e.alt)
            {
                var minDist = float.PositiveInfinity;
                var selectedEndpoint = (isValid: false, isStart: false, position: Vector3.zero, index: (int)0);
                foreach (var isStart in new[] { true, false })
                {
                    if ((isStart && _road.start) || (!isStart && _road.end)) continue;
                    var position = _road.transform.TransformPoint(_road[isStart ? 0 : -1]);
                    var d = 0.0f;
                    if ((isStart && _selectedAnchorPoint == 0) ||
                        (!isStart && _selectedAnchorPoint == _road.NumPoints - 1))
                        d = -1;
                    else
                        d = HandleUtility.DistanceToPolyLine(position, position);

                    if (d < minDist)
                    {
                        minDist = d;
                        selectedEndpoint.isValid = true;
                        selectedEndpoint.isStart = isStart;
                        selectedEndpoint.position = position;
                        selectedEndpoint.index = isStart ? 0 : _road.NumPoints - 1;
                    }
                }

                if (selectedEndpoint.isValid)
                {
                    Handles.color = Color.red;

                    Handles.SphereHandleCap(
                        0,
                        selectedEndpoint.position, Quaternion.identity,
                        0.3f * HandleUtility.GetHandleSize(selectedEndpoint.position),
                        EventType.Repaint);


                    var depth = Vector3.Dot(Camera.current.transform.forward,
                        selectedEndpoint.position - Camera.current.transform.position);
                    var ray = HandleUtility.GUIPointToWorldRay(e.mousePosition);
                    Vector3 position;

                    RoadUtilities.GetRotationAtWorldSpace(_road, selectedEndpoint.index, out var forward, out var up);
                    var d = selectedEndpoint.position.y;
                    if (Tools.pivotRotation == PivotRotation.Local)
                        d = Vector3.Dot(up, selectedEndpoint.position);
                    else
                    {
                        forward = Vector3.forward;
                        up = Vector3.up;
                    }

                    if (e.shift && new Plane(-up, d).Raycast(ray, out var enter) && enter < 500)
                    {
                        position = ray.origin + ray.direction * enter;

                        var right = Vector3.Cross(up, forward);
                        var c = Handles.color;
                        Handles.color = new Color(1, 0.3f, 0.3f, 0.5f);
                        RSHandleUtility.DrawGridCircles(selectedEndpoint.position, right, forward, 5, new[]
                        {
                            (selectedEndpoint.position, 20.0f),
                            (position, 20.0f),
                        });
                        Handles.color = c;
                    }
                    else if (!e.shift && _settings.UseRayCast && Physics.Raycast(ray, out var rayHit, 500))
                    {
                        position = rayHit.point;
                        if (_settings.CopyHitNormal)
                            up = rayHit.normal;
                    }
                    else
                    {
                        var direction = ray.direction / Vector3.Dot(Camera.current.transform.forward, ray.direction);
                        position = Camera.current.transform.position + depth * direction;
                    }

                    Handles.DrawLine(selectedEndpoint.position, position);

                    Handles.DrawLine(position,
                        position + 0.3f * HandleUtility.GetHandleSize(selectedEndpoint.position) * up);
                    Handles.ArrowHandleCap(0,
                        position + 0.3f * HandleUtility.GetHandleSize(selectedEndpoint.position) * up,
                        Quaternion.LookRotation(up), 1 * HandleUtility.GetHandleSize(selectedEndpoint.position),
                        EventType.Repaint);

                    Handles.SphereHandleCap(
                        0,
                        position, Quaternion.identity,
                        0.3f * HandleUtility.GetHandleSize(selectedEndpoint.position),
                        EventType.Repaint);

                    if (e.type == EventType.MouseDown && e.button == 0)
                    {
                        Undo.RecordObject(_road, "Add Segment");
                        _road.AppendSegment(_road.transform.InverseTransformPoint(position), selectedEndpoint.isStart,
                            _road.transform.InverseTransformDirection(up));
                        if (_selectedAnchorPoint > 0) _selectedAnchorPoint = _road.NumPoints - 1;
                    }
                }

                // If you click off the road, or the road itself, the selection system will deselect it.
                if (e.type == EventType.Used)
                {
                    Selection.activeObject = _road;
                }

                if (Event.current.type == EventType.MouseMove) SceneView.RepaintAll();
            }
        }

        #region Actions

        public bool SelectedIsEndpoint(YesNoMaybe shouldBeConnectedToAnchor = YesNoMaybe.MAYBE)
        {
            return IsEndPoint(_selectedAnchorPoint, shouldBeConnectedToAnchor);
        }

        public bool IsEndPoint(int i, YesNoMaybe shouldBeConnectedToAnchor = YesNoMaybe.MAYBE)
        {
            var isEndPoint = (i == 0 || i == _road.NumPoints - 1);
            if (!isEndPoint) return false;

            bool isConnected = (i == 0 ? _road.start : _road.end);

            switch (shouldBeConnectedToAnchor)
            {
                case YesNoMaybe.YES:
                    return isConnected;
                case YesNoMaybe.NO:
                    return !isConnected;
                default:
                    return true;
            }
        }

        public bool UnlinkSelected()
        {
            return Unlink(_selectedAnchorPoint);
        }

        /// <summary>
        /// Unlinks A point from an Anchor.
        /// </summary>
        /// <param name="i">- control point index</param>
        /// <returns>Should use Event?</returns>
        public bool Unlink(int i)
        {
            if (IsEndPoint(_selectedAnchorPoint))
            {
                if (!(i == 0 ? _road.start : _road.end))
                {
                    Debug.LogWarning("Endpoint is not connected to anything!");
                    return true;
                }

                Undo.RecordObject(_road, "Unlink Point from Anchor");
                if (i == 0)
                    _road.start.Disconnect();
                else
                    _road.end.Disconnect();

                return true;
            }

            return false;
        }

        public bool RemoveSelected()
        {
            return Remove(ref _selectedAnchorPoint);
        }

        /// <summary>
        /// Removes a point from the Road.
        /// </summary>
        /// <param name="i">- control point index</param>
        /// <returns>Should use Event?</returns>
        public bool Remove(ref int i)
        {
            if (i != -1)
            {
                if (_road.NumSegments == 1)
                {
                    Debug.LogWarning("Can't delete last segment!");
                    return true;
                }

                Undo.RecordObject(_road, "Delete Point");
                if (i == 0 && _road.start)
                    _road.start.Disconnect();
                else if (i == _road.NumPoints - 1 && _road.end)
                    _road.end.Disconnect();

                _road.DeleteAnchor(i);
                return true;
            }

            return false;
        }

        public bool ExtrudeSelected()
        {
            return Extrude(ref _selectedAnchorPoint);
        }

        /// <summary>
        /// Extends the end of the Road.
        /// </summary>
        /// <param name="i">- control point index</param>
        /// <returns>Should use Event?</returns>
        public bool Extrude(ref int i)
        {
            if (i == 0 || i == _road.NumPoints - 1)
            {
                if ((i == 0 && _road.start == null) || (i != 0 && _road.end == null))
                {
                    Undo.RecordObject(_road, "Extrude");
                    var endIndex = i;
                    var controlIndex = i == 0 ? 1 : -2;
                    _road.AppendSegment(_road[endIndex] - (_road[controlIndex] - _road[endIndex]).normalized * 2,
                        i == 0);
                    if (i != 0) i = _road.NumPoints - 1;
                    return true;
                }
            }

            return false;
        }

        #endregion Actions

        private static Rect _windowRect = new Rect(10000, 10000, 300, 300);

        private void GUIDrawWindow()
        {
            // only enable when a point can be selected
            switch (Tools.current)
            {
                case Tool.Move:
                case Tool.Rotate:
                case Tool.Scale:
                case Tool.Rect:
                    break;
                case Tool.View:
                case Tool.Transform:
                case Tool.Custom:
                case Tool.None:
                default: return;
            }

            _windowRect.x = Mathf.Clamp(_windowRect.x, 0,
                SceneView.lastActiveSceneView.camera.pixelWidth - _windowRect.width);
            _windowRect.y = Mathf.Clamp(_windowRect.y, 0,
                SceneView.lastActiveSceneView.camera.pixelHeight - _windowRect.height);

            Handles.BeginGUI();
            _windowRect = GUILayout.Window(0, _windowRect, windowID =>
            {
                if (_selectedAnchorPoint != -1)
                {
                    var oldVec = _road.transform.TransformPoint(_road[_selectedAnchorPoint]);
                    var newVec = EditorGUILayout.Vector3Field("Position", oldVec);
                    if (newVec != oldVec)
                    {
                        Undo.RecordObject(_road, "Move Point");
                        _road.MovePoint(_selectedAnchorPoint, _road.transform.InverseTransformPoint(newVec));
                    }

                    oldVec = RoadUtilities.GetRotationAtWorldSpace(_road, _selectedAnchorPoint).eulerAngles;
                    newVec = EditorGUILayout.Vector3Field("Rotation", oldVec);
                    if (newVec != oldVec)
                    {
                        Undo.RecordObject(_road, "Rotate Point");
                        RoadUtilities.SetRotationAtWorldSpace(_road, _selectedAnchorPoint, Quaternion.Euler(newVec));
                    }

                    if (_selectedAnchorPoint > 0)
                    {
                        oldVec = _road.transform.TransformPoint(_road[_selectedAnchorPoint - 1]);
                        newVec = EditorGUILayout.Vector3Field("Handle Position 1", oldVec);
                        if (newVec != oldVec)
                        {
                            Undo.RecordObject(_road, "Move Control Point");
                            _road.MovePoint(_selectedAnchorPoint - 1, _road.transform.InverseTransformPoint(newVec));
                        }
                    }

                    if (_selectedAnchorPoint < _road.NumPoints - 1)
                    {
                        oldVec = _road.transform.TransformPoint(_road[_selectedAnchorPoint + 1]);
                        newVec = EditorGUILayout.Vector3Field("Handle Position 2", oldVec);
                        if (newVec != oldVec)
                        {
                            Undo.RecordObject(_road, "Move Control Point");
                            _road.MovePoint(_selectedAnchorPoint + 1, _road.transform.InverseTransformPoint(newVec));
                        }
                    }

                    GUILayout.Label("Roll Angle", GUILayout.Width(EditorGUIUtility.labelWidth));
                    var oldFloat = _road.GetAngle(_selectedAnchorPoint / 3);
                    var newFloat = EditorGUILayout.DelayedFloatField(oldFloat);
                    // ReSharper disable once CompareOfFloatsByEqualityOperator
                    if (newFloat != oldFloat) // comparison is used for change detection, so it's not a mistake
                    {
                        Undo.RecordObject(_road, "Change Angle");
                        _road.MoveAngle(_selectedAnchorPoint / 3, newFloat);
                    }
                }

                GUI.DragWindow();
            }, _selectedAnchorPoint != -1 ? $"Point {_selectedAnchorPoint / 3}" : "No Point Selected");
            Handles.EndGUI();
        }

        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();
            EditorGUI.BeginChangeCheck();

            GUILayout.Space(10);
            var rect = EditorGUILayout.GetControlRect(false, 1);
            rect.height = 1;
            EditorGUI.DrawRect(rect, new Color(0.5f, 0.5f, 0.5f, 1));
            GUILayout.Space(10);

            EditorGUILayout.PropertyField(_settingsSerialized.FindProperty("roadSettings"));
            _settingsSerialized.ApplyModifiedProperties();

            BoolField("Auto Set Control Points", _road.AutoSetControlPoints, v => _road.AutoSetControlPoints = v, _road,
                false);
            if (GUILayout.Button("Set Control Points"))
            {
                Undo.RecordObject(_road, "Set Control Points");
                _road.AutoSetAllControlPoints();
            }

            GUILayout.EndHorizontal();

            GUILayout.Space(10);
            if (GUILayout.Button("Reset Road", GUILayout.Height(50)))
            {
                Undo.RecordObject(_road, "Reset Road");
                _road.Clear();
                _road.RefreshEndPoints();
                _selectedAnchorPoint = -1;
                SceneView.RepaintAll();
            }

            if (EditorGUI.EndChangeCheck())
            {
                SceneView.RepaintAll();
            }
        }

        private void BoolField(string label, bool value, Action<bool> setter, Object obj,
            bool endHorizontal = true)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(label, GUILayout.Width(EditorGUIUtility.labelWidth));
            var newValue = GUILayout.Toggle(value, "");
            if (newValue != value)
            {
                Undo.RecordObject(obj, $"Toggle {label}");
                setter(newValue);
            }

            if (endHorizontal) GUILayout.EndHorizontal();
        }

        // copied from HandleUtility, but with added out values
        private static Vector3 ClosestPointToPolyLine(Vector3[] vertices, out int index, out float t)
        {
            var num1 = HandleUtility.DistanceToLine(vertices[0], vertices[1]);
            var index1 = 0;
            for (var index2 = 2; index2 < vertices.Length; ++index2)
            {
                var line = HandleUtility.DistanceToLine(vertices[index2 - 1], vertices[index2]);
                if ((double)line < (double)num1)
                {
                    num1 = line;
                    index1 = index2 - 1;
                }
            }

            var vertex1 = vertices[index1];
            var vertex2 = vertices[index1 + 1];
            var rhs = Event.current.mousePosition - HandleUtility.WorldToGUIPoint(vertex1);
            var lhs = HandleUtility.WorldToGUIPoint(vertex2) - HandleUtility.WorldToGUIPoint(vertex1);
            var magnitude = lhs.magnitude;
            var num2 = Vector3.Dot((Vector3)lhs, (Vector3)rhs);
            if ((double)magnitude > 9.999999974752427E-07)
                num2 /= magnitude * magnitude;
            t = Mathf.Clamp01(num2);
            index = index1;
            return Vector3.Lerp(vertex1, vertex2, t);
        }

        public class RoadSelectionContext : ScriptableObject
        {
            public Road Road;
            public bool? EndSelected;
        }
    }
}
