using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace DelaunayTriangulation
{
    public class Delaunay
    {

        float EPSILON = 0.00001f;
        public Mesh GenerateDelaunayMesh(List<Vector2> points)
        {
            var triangles = BoyerWatson(points);
            var mesh = Triangle2Mesh(triangles);
            return mesh;
        }


        public Mesh GenerateDelaunayMesh(Vector2[] points)
        {
            List<Vector2> listPoints = new List<Vector2>(points);
            var triangles = BoyerWatson(listPoints);
            var mesh = Triangle2Mesh(triangles);
            return mesh;
        }


        /// <summary>
        /// https://en.wikipedia.org/wiki/Bowyer%E2%80%93Watson_algorithm
        /// </summary>
        /// <param name="startPoints"></param>
        /// <returns></returns>
        HashSet<Triangle> BoyerWatson(List<Vector2> startPoints)
        {
            // 1. 점들의 위치가 0 ~ 1 사이로 두게 만드는데, 그 이유는 roundoff-error 라고 위치가
            // 지정된 숫자보다 넘어가면 오버플로/언더플로가 일어나기에 그렇다
            var bound = SetBoundary(startPoints);
            var normalizedPoint = NormalizePoints(startPoints, bound);
            var triangulationData = new HalfEdgeData(bound);
            // 2. 모든 점들을 포함하는 슈퍼-삼각형을 만든다.
            Triangle superTriangle = new Triangle(new Vector2(-100, -100), new Vector2(0, 100), new Vector2(100, -100));
            triangulationData.AddTriangle(superTriangle);

            for (int i = 0; i < normalizedPoint.Count; i++)
            {
                // 3. 점을 하나씩 넣어 델루니 법칙을 적용한다
                OnInsertPoint(normalizedPoint[i], triangulationData);
            }
            // 4. 슈퍼-삼각형과 연관된 걸 삭제한다
            RemoveSuperTriangle(superTriangle, triangulationData);
            // 5. 비정규화하여 본래의 위치로 되돌린다
            UnNormalizePoints(triangulationData, bound);

            HashSet<Triangle> results = triangulationData.HalfEdge2Triangle();
            return results;
        }

        void OnInsertPoint(Vector2 point, HalfEdgeData triangulationData)
        {
            HashSet<HalfEdgeFace> badTriangles = new HashSet<HalfEdgeFace>();

            foreach (var triangle in triangulationData.faces)
            {
                if (triangle.isPointInsideCircumCircle(point))
                {
                    badTriangles.Add(triangle);
                }
            }
            List<HalfEdge> NotSharedEdges = isTriangleEdgeSharedByOther(badTriangles);

            foreach (var triangle in badTriangles)
            {
                triangulationData.DeleteTriangleFace(triangle, false);

            }
            List<HalfEdge> edges = new List<HalfEdge>(NotSharedEdges.Count * 3);
            foreach (var halfedge in NotSharedEdges)
            {
                var t = triangulationData.AddTriangle(halfedge, point);

                edges.Add(t.edge);
                edges.Add(t.edge.nextEdge);
                edges.Add(t.edge.prevEdge);
            }
            // 새로운 halfedge에 twin 추가
            for (int i = 0; i < edges.Count; i++)
            {
                var edge = edges[i];
                if (edge.oppositeEdge != null)
                {
                    continue;
                }

                var curEdgeVertex = edge.v;
                var nextEdgeVertex = edge.nextEdge.v;

                for (int j = 0; j < edges.Count; j++)
                {
                    var otherEdge = edges[j];

                    if (edge == otherEdge || otherEdge.oppositeEdge != null)
                    {
                        continue;
                    }
                    if (isSame(curEdgeVertex.position, otherEdge.nextEdge.v.position) && isSame(nextEdgeVertex.position, otherEdge.v.position))
                    {
                        edge.oppositeEdge = otherEdge;
                        otherEdge.oppositeEdge = edge;
                        break;
                    }

                }
            }
        }
        void RemoveSuperTriangle(Triangle superTriangle, HalfEdgeData data)
        {
            HashSet<HalfEdgeFace> trianglesToDelete = new HashSet<HalfEdgeFace>();

            foreach (var v in data.vertices)
            {
                if (trianglesToDelete.Contains(v.edge.face))
                {
                    continue;
                }

                var vPos = v.position;


                if (isSame(vPos, superTriangle.v1) || isSame(vPos, superTriangle.v2) || isSame(vPos, superTriangle.v3))
                {
                    trianglesToDelete.Add(v.edge.face);
                }
            }

            foreach (var face in trianglesToDelete)
            {
                data.DeleteTriangleFace(face, true);
            }
        }
        List<HalfEdge> isTriangleEdgeSharedByOther(HashSet<HalfEdgeFace> badTriangles)
        {
            List<HalfEdge> result = new List<HalfEdge>(badTriangles.Count * 3);

            foreach (var triangle in badTriangles)
            {
                var e1 = triangle.edge;
                var e2 = triangle.edge.nextEdge;
                var e3 = triangle.edge.prevEdge;


                if (!isEdgeSharedByOtherTriangles(e1, badTriangles))
                {
                    result.Add(e1);
                }
                if (!isEdgeSharedByOtherTriangles(e2, badTriangles))
                {
                    result.Add(e2);
                }
                if (!isEdgeSharedByOtherTriangles(e3, badTriangles))
                {
                    result.Add(e3);
                }
            }
            return result;
        }
        bool isEdgeSharedByOtherTriangles(HalfEdge edge, HashSet<HalfEdgeFace> badTriangles)
        {
            return edge.oppositeEdge != null && badTriangles.Contains(edge.oppositeEdge.face);
        }
        List<Vector2> NormalizePoints(List<Vector2> points, Bounds bound)
        {
            List<Vector2> result = new List<Vector2>(points.Count);
            float dMax = CalculateDMax(bound);

            for (int i = 0; i < points.Count; i++)
            {
                var p = points[i];

                float x = (p.x - bound.min.x) / dMax;
                float y = (p.y - bound.min.y) / dMax;

                result.Add(new Vector2(x, y));
            }
            return result;
        }

        void UnNormalizePoints(HalfEdgeData data, Bounds bound)
        {
            float dMax = CalculateDMax(bound);
            foreach (var v in data.vertices)
            {
                float x = (v.position.x * dMax) + bound.min.x;
                float y = (v.position.y * dMax) + bound.min.y;
                v.position = new Vector2(x, y);
            }
            foreach (var face in data.faces)
            {
                face.RecalculateCircums();
            }
        }

        float CalculateDMax(Bounds bound)
        {
            float dX = bound.max.x - bound.min.x;
            float dY = bound.max.y - bound.min.y;
            float dMax = Mathf.Max(dX, dY);
            return dMax;
        }

        Bounds SetBoundary(List<Vector2> points)
        {
            Vector2 newMin = new Vector2(float.MaxValue, float.MaxValue);
            Vector2 newMax = new Vector2(float.MinValue, float.MinValue);

            for (int i = 0; i < points.Count; ++i)
            {
                if (points[i].x > newMax.x)
                {
                    newMax.x = points[i].x;
                }

                if (points[i].y > newMax.y)
                {
                    newMax.y = points[i].y;
                }

                if (points[i].x < newMin.x)
                {
                    newMin.x = points[i].x;
                }

                if (points[i].y < newMin.y)
                {
                    newMin.y = points[i].y;
                }
            }

            Vector2 size = new Vector2(Mathf.Abs(newMax.x - newMin.x), Mathf.Abs(newMax.y - newMin.y));

            return new Bounds(newMin + (size * 0.5f), size);
        }

        Mesh Triangle2Mesh(HashSet<Triangle> triangles)
        {
            if (triangles == null)
            {
                return null;
            }

            Vector3[] triVertices = new Vector3[triangles.Count * 3];
            int[] triOrder = new int[triangles.Count * 3];
            int i = 0;
            foreach (var tri in triangles)
            {
                int triIndex = i * 3;
                int i1 = triIndex;
                int i2 = triIndex + 1;
                int i3 = triIndex + 2;

                // Debug.Log($"v1 : {tri.v1} , v2 : {tri.v2} , v3 : {tri.v3}");

                triVertices[i1] = tri.v1;
                triVertices[i2] = tri.v2;
                triVertices[i3] = tri.v3;

                triOrder[i1] = i1;
                triOrder[i2] = i2;
                triOrder[i3] = i3;
                i++;

            }

            Mesh mesh = new Mesh();

            mesh.vertices = triVertices;
            mesh.normals = triVertices;
            mesh.triangles = triOrder;

            return mesh;
        }


        bool isSame(Vector2 aVec, Vector2 bVec, float toleranceRange = 0.00001f)
        {
            return Vector3.SqrMagnitude(aVec - bVec) < toleranceRange;
        }
    }
}
