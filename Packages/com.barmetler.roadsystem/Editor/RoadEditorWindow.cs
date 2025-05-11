using System;
using System.Collections.Generic;
using Barmetler.RoadSystem.Settings;
using Barmetler.RoadSystem.Util;
using UnityEditor;
using UnityEngine;

namespace Barmetler.RoadSystem
{
    public class RoadEditorWindow : EditorWindow
    {
        [MenuItem("Tools/RoadSystem/Show Editor")]
        [MenuItem("Window/Road System Editor")]
        private static void ShowWindow()
        {
            GetWindow(typeof(RoadEditorWindow));
        }

        private struct Button
        {
            public enum ESymbol
            {
                NONE,
                PLUS,
                MINUS,
                LINK
            }

            public string Name; // unused for now, but can be used as identification.
            public string DisplayName;
            public string ToolTip;
            public ESymbol Symbol;
            public Action OnClick;
            public Action OnClickAlt;
            public Func<bool> IsEnabled;
            public Texture Icon;
        }

        private List<Button> _actions = new List<Button>();

        private const float ButtonSize = 48;
        private const float ButtonGap = 4;

        private void OnEnable()
        {
            _actions = new List<Button>
            {
                new Button
                {
                    Name = "new_road_system",
                    DisplayName = "New Road System",
                    ToolTip = "Create a new Road System",
                    Symbol = Button.ESymbol.PLUS,
                    OnClick = RoadMenu.CreateRoadSystem,
                },
                new Button
                {
                    Name = "new_intersection",
                    DisplayName = "New Intersection",
                    ToolTip = "Create a new Intersection",
                    Symbol = Button.ESymbol.PLUS,
                    OnClick = RoadMenu.CreateIntersection,
                    OnClickAlt = NewIntersectionWizard.CreateWizard,
                    Icon = EditorGUIUtility.Load(
                        "Packages/com.barmetler.roadsystem/Assets/Resources/Icons/Intersection.png") as Texture,
                },
                new Button
                {
                    Name = "new_road",
                    DisplayName = "New Road",
                    ToolTip = "Create a new Road",
                    Symbol = Button.ESymbol.PLUS,
                    OnClick = RoadMenu.CreateRoad,
                    OnClickAlt = NewRoadWizard.CreateWizard,
                    Icon =
                        EditorGUIUtility.Load("Packages/com.barmetler.roadsystem/Assets/Resources/Icons/Road.png") as
                            Texture,
                },
                new Button
                {
                    Name = "remove_point",
                    DisplayName = "Remove Point",
                    ToolTip = "Remove selected point from the Road [Backspace]",
                    Symbol = Button.ESymbol.MINUS,
                    OnClick = RoadMenu.MenuRemove,
                    IsEnabled = RoadMenu.MenuPointIsSelected,
                    Icon = EditorGUIUtility.Load(
                        "Packages/com.barmetler.roadsystem/Assets/Resources/Icons/RemovePoint.png") as Texture,
                },
                new Button
                {
                    Name = "extrude",
                    DisplayName = "Extrude",
                    ToolTip = "Extrude Selected Endpoint [Ctrl+E]",
                    Symbol = Button.ESymbol.PLUS,
                    OnClick = RoadMenu.MenuExtrude,
                    IsEnabled = RoadMenu.MenuEndPointIsSelectedAndNotConnected,
                    Icon =
                        EditorGUIUtility.Load("Packages/com.barmetler.roadsystem/Assets/Resources/Icons/Extrude.png") as
                            Texture,
                },
                new Button
                {
                    Name = "link",
                    DisplayName = "Link Points",
                    ToolTip =
                        "Enables the linking Tool.\n\n- Click a point to select\n- Shift-Click another to link\n- Shift-Ctrl-Click to link AND extend the road\n   (instead of moving the endpoint)\n- Ctrl-Click to disconnect.",
                    Symbol = Button.ESymbol.LINK,
                    OnClick = RoadMenu.MenuLink,
                    Icon =
                        EditorGUIUtility.Load("Packages/com.barmetler.roadsystem/Assets/Resources/Icons/Road.png") as
                            Texture,
                }
            };
            titleContent = new GUIContent("Road System Editor");
        }

        private void OnInspectorUpdate()
        {
            Repaint();
        }

        private void OnGUI()
        {
            var popupStyle = new GUIStyle();
            popupStyle.padding = new RectOffset(2, 2, 2, 2);
            popupStyle.alignment = TextAnchor.UpperRight;
            var symbolStyle = new GUIStyle(popupStyle);
            symbolStyle.alignment = TextAnchor.LowerRight;
            var popupIcon = EditorGUIUtility.IconContent("_Popup");
            var plusIcon = EditorGUIUtility.IconContent("Toolbar Plus");
            var minusIcon = EditorGUIUtility.IconContent("Toolbar Minus");
            var linkIcon = EditorGUIUtility.IconContent("Linked");

            GUIStyle buttonStyle = new GUIStyle(GUI.skin.button);
            buttonStyle.fontSize = 24;
            buttonStyle.fontStyle = FontStyle.Bold;

            int rowWidth = Mathf.Max(1,
                (int)((EditorGUIUtility.currentViewWidth - ButtonGap) / (ButtonSize + ButtonGap)));

            int x = 0;
            foreach (var action in _actions)
            {
                if (x == 0)
                    GUILayout.BeginHorizontal();

                var toolTip = action.ToolTip;
                if (action.OnClickAlt != null)
                    toolTip += "\n\n(ALT-Click for more Settings)";

                var content = action.Icon
                    ? new GUIContent(action.Icon, $"[{action.DisplayName}]: {toolTip}")
                    : new GUIContent(GetInitials(action.DisplayName), toolTip);

                GUI.enabled = action.OnClick != null && (action.IsEnabled?.Invoke() ?? true);
                if (GUILayout.Button(content, buttonStyle,
                        GUILayout.Width(50), GUILayout.Height(50)))
                {
                    if (action.OnClickAlt != null && Event.current.alt)
                        action.OnClickAlt();
                    else
                        action.OnClick?.Invoke();
                }

                var rect = GUILayoutUtility.GetLastRect();

                switch (action.Symbol)
                {
                    case Button.ESymbol.PLUS:
                        GUI.Label(rect, plusIcon, symbolStyle);
                        break;
                    case Button.ESymbol.MINUS:
                        GUI.Label(rect, minusIcon, symbolStyle);
                        break;
                    case Button.ESymbol.LINK:
                        GUI.Label(rect, linkIcon, symbolStyle);
                        break;
                }

                if (action.OnClickAlt != null)
                {
                    GUI.Label(rect, popupIcon, popupStyle);
                }

                if (x == rowWidth - 1)
                    GUILayout.EndHorizontal();
                x = (x + 1) % rowWidth;
            }

            GUI.enabled = true;
            if (x != 0)
                GUILayout.EndHorizontal();
        }

        private static string GetInitials(string str)
        {
            if (str == null) return "";
            str = str.ToLower();
            if (str.StartsWith("new"))
                str = str.Substring(3);
            return StringUtility.GetInitials(str);
        }
    }
}
