using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEngine;

namespace Barmetler.RoadSystem
{
    public static class RSHandleUtility
    {
        public static void DrawGridCircles(Vector3 origin, Vector3 right, Vector3 forward, float lineGap,
            IEnumerable<(Vector3 position, float radius)> centers)
        {
            foreach (var center in centers.Select(center =>
                     {
                         var relative = center.position - origin;
                         var u = Vector3.Dot(right, relative);
                         var v = Vector3.Dot(forward, relative);
                         return (uv: new Vector2(u, v), center.radius);
                     }))
            {
                for (var v = Mathf.CeilToInt((center.uv.y - center.radius) / lineGap) * lineGap;
                     v <= center.uv.y + center.radius;
                     v += lineGap)
                {
                    var pos = origin + forward * v + right * center.uv.x;
                    var width = Mathf.Sqrt(Mathf.Pow(center.radius, 2) - Mathf.Pow(v - center.uv.y, 2));
                    Handles.DrawLine(pos - right * width, pos + right * width);
                }

                for (var u = Mathf.CeilToInt((center.uv.x - center.radius) / lineGap) * lineGap;
                     u <= center.uv.x + center.radius;
                     u += lineGap)
                {
                    var pos = origin + forward * center.uv.y + right * u;
                    var width = Mathf.Sqrt(Mathf.Pow(center.radius, 2) - Mathf.Pow(u - center.uv.x, 2));
                    Handles.DrawLine(pos - forward * width, pos + forward * width);
                }
            }
        }

        public static void DrawBoundingBoxes(Road road)
        {
            var previousMatrix = Handles.matrix;
            Handles.matrix = road.transform.localToWorldMatrix;
            Handles.color = Color.grey * 0.8f;
            Handles.DrawWireCube(road.BoundingBox.center, road.BoundingBox.size);
            Handles.color = Color.white * 0.8f;
            foreach (var bounds in road.BoundingBoxes)
                Handles.DrawWireCube(bounds.center, bounds.size);
            Handles.matrix = previousMatrix;
        }
    }
}
