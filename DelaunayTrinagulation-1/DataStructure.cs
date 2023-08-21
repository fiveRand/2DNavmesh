using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace DelaunayTriangulation
{
    public struct Triangle
    {
        public Vector2 v1;
        public Vector2 v2;
        public Vector2 v3;

        public Triangle(Vector2 v1, Vector2 v2, Vector2 v3)
        {
            this.v1 = v1;
            this.v2 = v2;
            this.v3 = v3;

            SetOrientation();
        }
        /// <summary>
        /// Switch clockwise = counter-clockwise 
        /// </summary>
        public void ChangeOrientation()
        {
            Vector2 temp = v1;
            v1 = v2;
            v2 = temp;
        }

        public float MinX()
        {
            return Mathf.Min(v1.x, Mathf.Min(v2.x, v3.x));
        }
        public float MaxX()
        {
            return Mathf.Max(v1.x, Mathf.Max(v2.x, v3.x));
        }
        public float MinY()
        {
            return Mathf.Min(v1.y, Mathf.Min(v2.y, v3.y));
        }
        public float MaxY()
        {
            return Mathf.Max(v1.y, Mathf.Max(v2.y, v3.y));
        }

        public void SetOrientation()
        {
            if (!MathUtility.isTriangleOrientedClockWise(v1, v2, v3))
            {
                ChangeOrientation();
            }
        }


    }

    public class HalfEdge
    {
        //The vertex it points to
        public HalfEdgeVertex v;

        //The face it belongs to
        public HalfEdgeFace face;

        //The next half-edge inside the face (ordered clockwise)
        //The document says counter-clockwise but clockwise is easier because that's how Unity is displaying triangles
        public HalfEdge nextEdge;

        //The opposite half-edge belonging to the neighbor
        public HalfEdge oppositeEdge;

        //(optionally) the previous halfedge in the face
        //If we assume the face is closed, then we could identify this edge by walking forward
        //until we reach it
        public HalfEdge prevEdge;



        public HalfEdge(HalfEdgeVertex v)
        {
            this.v = v;
        }

    }

    public class HalfEdgeFace
    {
        public HalfEdge edge;
        public Vector2 centerPosition;
        public float circumRadius;
        public Vector2 circumCenter;

        public HalfEdgeFace(HalfEdge edge)
        {
            this.edge = edge;

            Vector2 a = edge.v.position;
            Vector2 b = edge.nextEdge.v.position;
            Vector2 c = edge.prevEdge.v.position;

            LinearEquation lineAB = new LinearEquation(a, b);
            LinearEquation lineBC = new LinearEquation(b, c);
            var perpendicularAB = lineAB.PerpendicularLineAt(Vector2.Lerp(a, b, .5f));
            var perpendicularBC = lineBC.PerpendicularLineAt(Vector2.Lerp(b, c, .5f));

            this.circumCenter = GetCrossingPoint(perpendicularAB, perpendicularBC);

            this.circumRadius = Vector2.Distance(circumCenter, a);
        }

        public HalfEdgeFace(Vector3 a, Vector3 b, Vector3 c)
        {

            LinearEquation lineAB = new LinearEquation(a, b);
            LinearEquation lineBC = new LinearEquation(b, c);
            var perpendicularAB = lineAB.PerpendicularLineAt(Vector2.Lerp(a, b, .5f));
            var perpendicularBC = lineBC.PerpendicularLineAt(Vector2.Lerp(b, c, .5f));

            this.circumCenter = GetCrossingPoint(perpendicularAB, perpendicularBC);
            this.circumRadius = Vector2.Distance(circumCenter, a);

            centerPosition = (a + b + c) / 3;
        }


        public bool isPointInsideCircumCircle(Vector2 point)
        {

            float dist = Vector2.Distance(point, circumCenter);
            return dist < circumRadius;
        }

        public void RecalculateCircums()
        {
            Vector2 a = edge.v.position;
            Vector2 b = edge.nextEdge.v.position;
            Vector2 c = edge.prevEdge.v.position;

            LinearEquation lineAB = new LinearEquation(a, b);
            LinearEquation lineBC = new LinearEquation(b, c);
            var perpendicularAB = lineAB.PerpendicularLineAt(Vector2.Lerp(a, b, .5f));
            var perpendicularBC = lineBC.PerpendicularLineAt(Vector2.Lerp(b, c, .5f));

            circumCenter = GetCrossingPoint(perpendicularAB, perpendicularBC);
            circumRadius = Vector2.Distance(circumCenter, a);


            centerPosition = (a + b + c) / 3;
        }


        static Vector2 GetCrossingPoint(LinearEquation line1, LinearEquation line2)
        {
            float A1 = line1.a;
            float A2 = line2.a;
            float B1 = line1.b;
            float B2 = line2.b;
            float C1 = line1.c;
            float C2 = line2.c;

            //Cramer's rule
            float Determinant = A1 * B2 - A2 * B1;
            float DeterminantX = C1 * B2 - C2 * B1;
            float DeterminantY = A1 * C2 - A2 * C1;

            float x = DeterminantX / Determinant;
            float y = DeterminantY / Determinant;

            return new Vector2(x, y);
        }

        public bool IsInside(Vector3 p)
        {
            Vector2 p1 = edge.v.position;
            Vector2 p2 = edge.nextEdge.v.position;
            Vector2 p3 = edge.prevEdge.v.position;

            bool isWithinTriangle = false;

            //Based on Barycentric coordinates
            float denominator = ((p2.y - p3.y) * (p1.x - p3.x) + (p3.x - p2.x) * (p1.y - p3.y));

            float a = ((p2.y - p3.y) * (p.x - p3.x) + (p3.x - p2.x) * (p.y - p3.y)) / denominator;
            float b = ((p3.y - p1.y) * (p.x - p3.x) + (p1.x - p3.x) * (p.y - p3.y)) / denominator;
            float c = 1 - a - b;

            //The point is within the triangle or on the border if 0 <= a <= 1 and 0 <= b <= 1 and 0 <= c <= 1
            //if (a >= 0f && a <= 1f && b >= 0f && b <= 1f && c >= 0f && c <= 1f)
            //{
            //    isWithinTriangle = true;
            //}

            //The point is within the triangle
            if (a > 0f && a < 1f && b > 0f && b < 1f && c > 0f && c < 1f)
            {
                isWithinTriangle = true;
            }

            return isWithinTriangle;
        }

        [System.Serializable]
        public struct LinearEquation
        {
            public float a;
            public float b;
            public float c;

            public LinearEquation(float a, float b, float c)
            {
                this.a = a;
                this.b = b;
                this.c = c;
            }
            //Ax+By=C
            public LinearEquation(Vector2 pointA, Vector2 pointB)
            {
                float deltaX = pointB.x - pointA.x;
                float deltaY = pointB.y - pointA.y;
                a = deltaY; //y2-y1
                b = -deltaX; //x1-x2
                c = a * pointA.x + b * pointA.y;
            }

            public LinearEquation PerpendicularLineAt(Vector3 point)
            {

                float c = -b * point.x + a * point.y;
                LinearEquation newLine = new LinearEquation(-b, a, c);


                return newLine;
            }
        }
    }

    public class HalfEdgeVertex
    {
        public Vector2 position;

        // 이 edge의 시작점은 이 vertex다.
        public HalfEdge edge;

        public HalfEdgeVertex(Vector2 position_)
        {
            position = position_;
        }
    }

    public class HalfEdgeData
    {
        public HashSet<HalfEdgeVertex> vertices;

        public HashSet<HalfEdgeFace> faces;

        public HashSet<HalfEdge> edges;
        public Bounds bound;

        public HalfEdgeData(Bounds bound)
        {
            this.vertices = new HashSet<HalfEdgeVertex>();
            this.faces = new HashSet<HalfEdgeFace>();
            this.edges = new HashSet<HalfEdge>();
            this.bound = bound;
        }

        public void AddTriangle(Triangle tri)
        {
            HalfEdgeVertex v1 = new HalfEdgeVertex(tri.v1);
            HalfEdgeVertex v2 = new HalfEdgeVertex(tri.v2);
            HalfEdgeVertex v3 = new HalfEdgeVertex(tri.v3);

            HalfEdge he1 = new HalfEdge(v1);
            HalfEdge he2 = new HalfEdge(v2);
            HalfEdge he3 = new HalfEdge(v3);

            he1.nextEdge = he2;
            he2.nextEdge = he3;
            he3.nextEdge = he1;

            he1.prevEdge = he3;
            he2.prevEdge = he1;
            he3.prevEdge = he2;

            v1.edge = he2;
            v2.edge = he3;
            v3.edge = he1;

            HalfEdgeFace face = new HalfEdgeFace(he1);

            // he1.face = he2.face = he3.face = face;
            he1.face = face;
            he2.face = face;
            he3.face = face;

            edges.Add(he1);
            edges.Add(he2);
            edges.Add(he3);

            faces.Add(face);

            vertices.Add(v1);
            vertices.Add(v2);
            vertices.Add(v3);


        }

        public HalfEdgeFace AddTriangle(HalfEdge eOld, Vector2 point)
        {
            var vOrigin = new HalfEdgeVertex(eOld.v.position);
            var vNext = new HalfEdgeVertex(eOld.nextEdge.v.position);
            var vPrev = new HalfEdgeVertex(point);

            HalfEdge e1 = new HalfEdge(vOrigin);
            HalfEdge e2 = new HalfEdge(vNext);
            HalfEdge e3 = new HalfEdge(vPrev);
            // create Connection
            // Update new Edge connection.
            // But if opposite edge needs a new reference to this edge if it's not a border
            e1.oppositeEdge = eOld.oppositeEdge;
            if (e1.oppositeEdge != null)
            {
                eOld.oppositeEdge.oppositeEdge = e1;
            }

            // Creating connection each other
            e1.nextEdge = e2;
            e1.prevEdge = e3;

            e2.nextEdge = e3;
            e2.prevEdge = e1;

            e3.nextEdge = e1;
            e3.prevEdge = e2;
            // Update face
            // add Edge to vertex

            HalfEdgeFace f = new HalfEdgeFace(e1);
            e1.face = f;
            e2.face = f;
            e3.face = f;

            vPrev.edge = e1;
            vNext.edge = e3;
            vOrigin.edge = e2;

            faces.Add(f);

            edges.Add(e1);
            edges.Add(e2);
            edges.Add(e3);

            vertices.Add(vOrigin);
            vertices.Add(vNext);
            vertices.Add(vPrev);
            return f;
        }
        public void DeleteTriangleFace(HalfEdgeFace face, bool shouldSetOppositeNull)
        {
            var e1 = face.edge;
            var e2 = e1.nextEdge;
            var e3 = e1.prevEdge;

            if (shouldSetOppositeNull)
            {
                if (e1.oppositeEdge != null)
                {
                    e1.oppositeEdge.oppositeEdge = null;
                }
                if (e2.oppositeEdge != null)
                {
                    e2.oppositeEdge.oppositeEdge = null;

                }
                if (e3.oppositeEdge != null)
                {
                    e3.oppositeEdge.oppositeEdge = null;
                }
            }

            faces.Remove(face);

            edges.Remove(e1);
            edges.Remove(e2);
            edges.Remove(e3);

            vertices.Remove(e1.v);
            vertices.Remove(e2.v);
            vertices.Remove(e3.v);
        }

        public HashSet<Triangle> HalfEdge2Triangle()
        {
            HashSet<Triangle> triangles = new HashSet<Triangle>();

            foreach (var face in faces)
            {
                var v1 = face.edge.v.position;
                var v2 = face.edge.nextEdge.v.position;
                var v3 = face.edge.prevEdge.v.position;

                Triangle tri = new Triangle(v1, v2, v3);
                triangles.Add(tri);
            }
            return triangles;
        }
    }

    public class Normalizer2
    {
        float dMax;
        AABB2 boundingBox;
        public Normalizer2(List<Vector2> points)
        {
            this.boundingBox = new AABB2(points);
            this.dMax = CalculateDMax(boundingBox);
        }
        public float CalculateDMax(AABB2 boundingBox)
        {
            float dX = boundingBox.max.x - boundingBox.min.x;
            float dY = boundingBox.max.y - boundingBox.min.y;
            float dMax = Mathf.Max(dX, dY);
            return dMax;
        }

        public Vector2 Normalize(Vector2 point)
        {
            float x = (point.x - boundingBox.min.x) / dMax;
            float y = (point.y - boundingBox.min.y) / dMax;

            return new Vector2(x, y);
        }

        public Vector2 UnNormalize(Vector2 point)
        {
            float x = (point.x * dMax) + boundingBox.min.x;
            float y = (point.y * dMax) + boundingBox.min.y;

            return new Vector2(x, y);
        }

        public List<Vector2> Normalize(List<Vector2> points)
        {
            List<Vector2> result = new List<Vector2>();
            foreach (var p in points)
            {
                result.Add(Normalize(p));
            }
            return result;
        }


        public void UnNormalize(ref HalfEdgeData data)
        {
            foreach (var v in data.vertices)
            {
                Vector2 unNormalize = UnNormalize(v.position);
                v.position = unNormalize;
            }
        }
    }

    public struct AABB2
    {
        public Vector2 min;
        public Vector2 max;

        public AABB2(Vector2 min, Vector2 max)
        {
            this.min = min;
            this.max = max;
        }

        public AABB2(List<Vector2> points)
        {

            float minX = float.MaxValue;
            float minY = float.MaxValue;
            float maxX = float.MinValue;
            float maxY = float.MinValue;
            for (int i = 0; i < points.Count; i++)
            {
                var point = points[i];
                if (point.x < minX)
                {
                    minX = point.x;
                }
                else if (point.x > maxX)
                {
                    maxX = point.x;
                }

                if (point.y < minY)
                {
                    minY = point.y;
                }
                else if (point.y > maxY)
                {
                    maxY = point.y;
                }
            }
            this.min = new Vector2(minX, minY);
            this.max = new Vector2(maxX, maxY);
        }
    }
}
