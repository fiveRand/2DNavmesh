using System.Collections;
using System.Collections.Generic;
using UnityEngine;
# if UNITY_EDITOR
using UnityEditor;
#endif

namespace DelaunayTriangulation
{

    public class Map : MonoBehaviour
    {
        public List<Vector2> points = new List<Vector2>();

        public Vector2 gridSize = new Vector2(50, 50);
        public float radius = 4;
        Mesh mesh;
        private void OnDrawGizmosSelected()
        {
            Gizmos.color = Color.white;
            Gizmos.DrawWireMesh(mesh);
            foreach (var point in points)
            {
                Gizmos.DrawWireSphere(point, 0.5f);
            }
        }
        public void GenerateDelaunay()
        {
            mesh = BoyerWatson.GenerateDelaunayMesh(points);
        }

        public void GetRandomPosition()
        {
            points = PoissonDiscSamping.GeneratePoint(transform.position, gridSize, radius);
        }
    }


#if UNITY_EDITOR

    [CustomEditor(typeof(Map))]
    public class MapEditor : Editor
    {

        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();

            var script = (Map)target;

            if (GUILayout.Button("Triangulate"))
            {
                script.GenerateDelaunay();
            }

            if (GUILayout.Button("GetRandomPosition"))
            {
                script.GetRandomPosition();
            }
        }
    }
#endif
}
