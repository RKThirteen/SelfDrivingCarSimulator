using System.Linq;
using UnityEngine;

namespace Barmetler.RoadSystem.Util
{
    /// <summary>
    /// Updates a LineRenderer based on a RoadSystemNavigator
    /// </summary>
    [ExecuteAlways, RequireComponent(typeof(LineRenderer))]
    public class NavigationLineUpdater : MonoBehaviour
    {
        [SerializeField]
        private RoadSystemNavigator navigator;

        [SerializeField]
        private float Tolerance = 0.1f;

        [SerializeField]
        private float LineWidth = 2;

        [SerializeField, HideInInspector]
        private LineRenderer lineRenderer;

        private AsyncUpdater<Vector3[]> _pathPoints;

        private void OnValidate()
        {
            lineRenderer = GetComponent<LineRenderer>();
        }

        private void Awake()
        {
            OnValidate();
        }

        // Update is called once per frame
        private void Update()
        {
            _pathPoints ??= new AsyncUpdater<Vector3[]>(this, UpdateData, new Vector3[] { }, 1f / 144);
            _pathPoints.Update();
            var points = _pathPoints.GetData();
            lineRenderer.positionCount = points.Length;
            lineRenderer.SetPositions(points);
            lineRenderer.widthMultiplier = LineWidth;
        }

        private Vector3[] UpdateData()
        {
            if (!navigator) return new Vector3[] { };
            var points = navigator.CurrentPoints.Select(e => e.position).ToList();

            LineUtility.Simplify(points.ToList(), Tolerance, points);

            return points.Select(e =>
                Vector3.Scale(e, Vector3.forward + Vector3.right) + Vector3.up * 100
            ).ToArray();
        }
    }
}
