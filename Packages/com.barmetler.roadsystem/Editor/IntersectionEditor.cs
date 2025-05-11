using System.Collections.Generic;
using System.Linq;
using UnityEditor;

namespace Barmetler.RoadSystem
{
    [CustomEditor(typeof(Intersection))]
    public class IntersectionEditor : Editor
    {
        private Intersection _intersection;
        private List<Road> _affectedRoads;

        private void OnEnable()
        {
            _intersection = (Intersection)target;
            _intersection.Invalidate();
            _affectedRoads = _intersection.AnchorPoints.Select(e => e.GetConnectedRoad()).Where(e => e).ToList();
            Undo.undoRedoPerformed += OnUndoRedo;
        }

        private void OnDisable()
        {
            Undo.undoRedoPerformed -= OnUndoRedo;
        }

        public void OnUndoRedo()
        {
            _affectedRoads.ForEach(e => e.OnCurveChanged(true));
        }

        private void OnSceneGUI()
        {
            if (_intersection.transform.hasChanged)
            {
                _intersection.transform.hasChanged = false;
                _intersection.Invalidate();
            }
        }
    }
}
