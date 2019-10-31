using UnityEngine;
using System.Collections.Generic;
using System;

namespace Pathfinding {
	using Pathfinding.Util;

	static class AstarSplines {
		public static Vector3 CatmullRom (Vector3 previous, Vector3 start, Vector3 end, Vector3 next, float elapsedTime) {
			

			float percentComplete = elapsedTime;
			float percentCompleteSquared = percentComplete * percentComplete;
			float percentCompleteCubed = percentCompleteSquared * percentComplete;

			return
				previous * (-0.5F*percentCompleteCubed +
							percentCompleteSquared -
							0.5F*percentComplete) +

				start *
				(1.5F*percentCompleteCubed +
				 -2.5F*percentCompleteSquared + 1.0F) +

				end *
				(-1.5F*percentCompleteCubed +
				 2.0F*percentCompleteSquared +
				 0.5F*percentComplete) +

				next *
				(0.5F*percentCompleteCubed -
				 0.5F*percentCompleteSquared);
		}

		public static Vector3 CubicBezier (Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t) {
			t = Mathf.Clamp01(t);
			float t2 = 1-t;
			return t2*t2*t2 * p0 + 3 * t2*t2 * t * p1 + 3 * t2 * t*t * p2 + t*t*t * p3;
		}

		public static Vector3 CubicBezierDerivative (Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t) {
			t = Mathf.Clamp01(t);
			float t2 = 1-t;
			return 3*t2*t2*(p1-p0) + 6*t2*t*(p2 - p1) + 3*t*t*(p3 - p2);
		}

		public static Vector3 CubicBezierSecondDerivative (Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t) {
			t = Mathf.Clamp01(t);
			float t2 = 1-t;
			return 6*t2*(p2 - 2*p1 + p0) + 6*t*(p3 - 2*p2 + p1);
		}
	}

	
	public static class VectorMath {
		/// <summary>
		/// Complex number multiplication.
		/// Returns: a * b
		///
		/// Used to rotate vectors in an efficient way.
		///
		/// See: https://en.wikipedia.org/wiki/Complex_number<see cref="Multiplication_and_division"/>
		/// </summary>
		public static Vector2 ComplexMultiply (Vector2 a, Vector2 b) {
			return new Vector2(a.x * b.x - a.y * b.y, a.x * b.y + a.y * b.x);
		}

		
		public static Vector2 ComplexMultiplyConjugate (Vector2 a, Vector2 b) {
			return new Vector2(a.x * b.x + a.y * b.y, a.y * b.x - a.x * b.y);
		}

		
		public static Vector3 ClosestPointOnLine (Vector3 lineStart, Vector3 lineEnd, Vector3 point) {
			Vector3 lineDirection = Vector3.Normalize(lineEnd - lineStart);
			float dot = Vector3.Dot(point - lineStart, lineDirection);

			return lineStart + (dot*lineDirection);
		}

		
		public static float ClosestPointOnLineFactor (Vector3 lineStart, Vector3 lineEnd, Vector3 point) {
			var dir = lineEnd - lineStart;
			float sqrMagn = dir.sqrMagnitude;

			if (sqrMagn <= 0.000001) return 0;

			return Vector3.Dot(point - lineStart, dir) / sqrMagn;
		}

		
		public static float ClosestPointOnLineFactor (Int3 lineStart, Int3 lineEnd, Int3 point) {
			var lineDirection = lineEnd - lineStart;
			float magn = lineDirection.sqrMagnitude;

			float closestPoint = Int3.Dot((point - lineStart), lineDirection);

			if (magn != 0) closestPoint /= magn;

			return closestPoint;
		}

		
		public static float ClosestPointOnLineFactor (Int2 lineStart, Int2 lineEnd, Int2 point) {
			var lineDirection = lineEnd - lineStart;
			double magn = lineDirection.sqrMagnitudeLong;

			double closestPoint = Int2.DotLong(point - lineStart, lineDirection);

			if (magn != 0) closestPoint /= magn;

			return (float)closestPoint;
		}

		
		public static Vector3 ClosestPointOnSegment (Vector3 lineStart, Vector3 lineEnd, Vector3 point) {
			var dir = lineEnd - lineStart;
			float sqrMagn = dir.sqrMagnitude;

			if (sqrMagn <= 0.000001) return lineStart;

			float factor = Vector3.Dot(point - lineStart, dir) / sqrMagn;
			return lineStart + Mathf.Clamp01(factor)*dir;
		}

		
		public static Vector3 ClosestPointOnSegmentXZ (Vector3 lineStart, Vector3 lineEnd, Vector3 point) {
			lineStart.y = point.y;
			lineEnd.y = point.y;
			Vector3 fullDirection = lineEnd-lineStart;
			Vector3 fullDirection2 = fullDirection;
			fullDirection2.y = 0;
			float magn = fullDirection2.magnitude;
			Vector3 lineDirection = magn > float.Epsilon ? fullDirection2/magn : Vector3.zero;

			float closestPoint = Vector3.Dot((point-lineStart), lineDirection);
			return lineStart+(Mathf.Clamp(closestPoint, 0.0f, fullDirection2.magnitude)*lineDirection);
		}

		
		public static float SqrDistancePointSegmentApproximate (int x, int z, int px, int pz, int qx, int qz) {
			float pqx = (float)(qx - px);
			float pqz = (float)(qz - pz);
			float dx = (float)(x - px);
			float dz = (float)(z - pz);
			float d = pqx*pqx + pqz*pqz;
			float t = pqx*dx + pqz*dz;

			if (d > 0)
				t /= d;
			if (t < 0)
				t = 0;
			else if (t > 1)
				t = 1;

			dx = px + t*pqx - x;
			dz = pz + t*pqz - z;

			return dx*dx + dz*dz;
		}

		
		public static float SqrDistancePointSegmentApproximate (Int3 a, Int3 b, Int3 p) {
			float pqx = (float)(b.x - a.x);
			float pqz = (float)(b.z - a.z);
			float dx = (float)(p.x - a.x);
			float dz = (float)(p.z - a.z);
			float d = pqx*pqx + pqz*pqz;
			float t = pqx*dx + pqz*dz;

			if (d > 0)
				t /= d;
			if (t < 0)
				t = 0;
			else if (t > 1)
				t = 1;

			dx = a.x + t*pqx - p.x;
			dz = a.z + t*pqz - p.z;

			return dx*dx + dz*dz;
		}

		
		public static float SqrDistancePointSegment (Vector3 a, Vector3 b, Vector3 p) {
			var nearest = ClosestPointOnSegment(a, b, p);

			return (nearest-p).sqrMagnitude;
		}

		
		public static float SqrDistanceSegmentSegment (Vector3 s1, Vector3 e1, Vector3 s2, Vector3 e2) {
			Vector3 u = e1 - s1;
			Vector3 v = e2 - s2;
			Vector3 w = s1 - s2;
			float a = Vector3.Dot(u, u);           // always >= 0
			float b = Vector3.Dot(u, v);
			float c = Vector3.Dot(v, v);           // always >= 0
			float d = Vector3.Dot(u, w);
			float e = Vector3.Dot(v, w);
			float D = a*c - b*b;           // always >= 0
			float sc, sN, sD = D;          // sc = sN / sD, default sD = D >= 0
			float tc, tN, tD = D;          // tc = tN / tD, default tD = D >= 0

			// compute the line parameters of the two closest points
			if (D < 0.000001f) { // the lines are almost parallel
				sN = 0.0f;         // force using point P0 on segment S1
				sD = 1.0f;         // to prevent possible division by 0.0 later
				tN = e;
				tD = c;
			} else {               // get the closest points on the infinite lines
				sN = (b*e - c*d);
				tN = (a*e - b*d);
				if (sN < 0.0f) {        // sc < 0 => the s=0 edge is visible
					sN = 0.0f;
					tN = e;
					tD = c;
				} else if (sN > sD) { // sc > 1  => the s=1 edge is visible
					sN = sD;
					tN = e + b;
					tD = c;
				}
			}

			if (tN < 0.0f) {            // tc < 0 => the t=0 edge is visible
				tN = 0.0f;
				// recompute sc for this edge
				if (-d < 0.0f)
					sN = 0.0f;
				else if (-d > a)
					sN = sD;
				else {
					sN = -d;
					sD = a;
				}
			} else if (tN > tD) {    // tc > 1  => the t=1 edge is visible
				tN = tD;
				// recompute sc for this edge
				if ((-d + b) < 0.0f)
					sN = 0;
				else if ((-d + b) > a)
					sN = sD;
				else {
					sN = (-d +  b);
					sD = a;
				}
			}
			// finally do the division to get sc and tc
			sc = (Math.Abs(sN) < 0.000001f ? 0.0f : sN / sD);
			tc = (Math.Abs(tN) < 0.000001f ? 0.0f : tN / tD);

			// get the difference of the two closest points
			Vector3 dP = w + (sc * u) - (tc * v);  // =  S1(sc) - S2(tc)

			return dP.sqrMagnitude;   // return the closest distance
		}

		public static float SqrDistanceXZ (Vector3 a, Vector3 b) {
			var delta = a-b;

			return delta.x*delta.x+delta.z*delta.z;
		}

		
		public static long SignedTriangleAreaTimes2XZ (Int3 a, Int3 b, Int3 c) {
			return (long)(b.x - a.x) * (long)(c.z - a.z) - (long)(c.x - a.x) * (long)(b.z - a.z);
		}

		
		public static float SignedTriangleAreaTimes2XZ (Vector3 a, Vector3 b, Vector3 c) {
			return (b.x - a.x) * (c.z - a.z) - (c.x - a.x) * (b.z - a.z);
		}

		
		public static bool RightXZ (Vector3 a, Vector3 b, Vector3 p) {
			return (b.x - a.x) * (p.z - a.z) - (p.x - a.x) * (b.z - a.z) < -float.Epsilon;
		}

		
		public static bool RightXZ (Int3 a, Int3 b, Int3 p) {
			return (long)(b.x - a.x) * (long)(p.z - a.z) - (long)(p.x - a.x) * (long)(b.z - a.z) < 0;
		}

		
		public static Side SideXZ (Int3 a, Int3 b, Int3 p) {
			var s = (long)(b.x - a.x) * (long)(p.z - a.z) - (long)(p.x - a.x) * (long)(b.z - a.z);

			return s > 0 ? Side.Left : (s < 0 ? Side.Right : Side.Colinear);
		}

		
		public static bool RightOrColinear (Vector2 a, Vector2 b, Vector2 p) {
			return (b.x - a.x) * (p.y - a.y) - (p.x - a.x) * (b.y - a.y) <= 0;
		}

		
		public static bool RightOrColinear (Int2 a, Int2 b, Int2 p) {
			return (long)(b.x - a.x) * (long)(p.y - a.y) - (long)(p.x - a.x) * (long)(b.y - a.y) <= 0;
		}

		
		public static bool RightOrColinearXZ (Vector3 a, Vector3 b, Vector3 p) {
			return (b.x - a.x) * (p.z - a.z) - (p.x - a.x) * (b.z - a.z) <= 0;
		}

		
		public static bool RightOrColinearXZ (Int3 a, Int3 b, Int3 p) {
			return (long)(b.x - a.x) * (long)(p.z - a.z) - (long)(p.x - a.x) * (long)(b.z - a.z) <= 0;
		}

		
		public static bool IsClockwiseMarginXZ (Vector3 a, Vector3 b, Vector3 c) {
			return (b.x-a.x)*(c.z-a.z)-(c.x-a.x)*(b.z-a.z) <= float.Epsilon;
		}

		
		public static bool IsClockwiseXZ (Vector3 a, Vector3 b, Vector3 c) {
			return (b.x-a.x)*(c.z-a.z)-(c.x-a.x)*(b.z-a.z) < 0;
		}

		
		public static bool IsClockwiseXZ (Int3 a, Int3 b, Int3 c) {
			return RightXZ(a, b, c);
		}

		
		public static bool IsClockwiseOrColinearXZ (Int3 a, Int3 b, Int3 c) {
			return RightOrColinearXZ(a, b, c);
		}

		
		public static bool IsClockwiseOrColinear (Int2 a, Int2 b, Int2 c) {
			return RightOrColinear(a, b, c);
		}

		
		public static bool IsColinear (Vector3 a, Vector3 b, Vector3 c) {
			var lhs = b - a;
			var rhs = c - a;
			// Take the cross product of lhs and rhs
			// The magnitude of the cross product will be zero if the points a,b,c are colinear
			float x = lhs.y * rhs.z - lhs.z * rhs.y;
			float y = lhs.z * rhs.x - lhs.x * rhs.z;
			float z = lhs.x * rhs.y - lhs.y * rhs.x;
			float v = x*x + y*y + z*z;

			// Epsilon not chosen with much thought, just that float.Epsilon was a bit too small.
			return v <= 0.0000001f;
		}

		public static bool IsColinear (Vector2 a, Vector2 b, Vector2 c) {
			float v = (b.x-a.x)*(c.y-a.y)-(c.x-a.x)*(b.y-a.y);

			// Epsilon not chosen with much thought, just that float.Epsilon was a bit too small.
			return v <= 0.0000001f && v >= -0.0000001f;
		}

		public static bool IsColinearXZ (Int3 a, Int3 b, Int3 c) {
			return (long)(b.x - a.x) * (long)(c.z - a.z) - (long)(c.x - a.x) * (long)(b.z - a.z) == 0;
		}

		public static bool IsColinearXZ (Vector3 a, Vector3 b, Vector3 c) {
			float v = (b.x-a.x)*(c.z-a.z)-(c.x-a.x)*(b.z-a.z);

			// Epsilon not chosen with much thought, just that float.Epsilon was a bit too small.
			return v <= 0.0000001f && v >= -0.0000001f;
		}

		public static bool IsColinearAlmostXZ (Int3 a, Int3 b, Int3 c) {
			long v = (long)(b.x - a.x) * (long)(c.z - a.z) - (long)(c.x - a.x) * (long)(b.z - a.z);

			return v > -1 && v < 1;
		}

		
		public static bool SegmentsIntersect (Int2 start1, Int2 end1, Int2 start2, Int2 end2) {
			return RightOrColinear(start1, end1, start2) != RightOrColinear(start1, end1, end2) && RightOrColinear(start2, end2, start1) != RightOrColinear(start2, end2, end1);
		}

		
		public static bool SegmentsIntersectXZ (Int3 start1, Int3 end1, Int3 start2, Int3 end2) {
			return RightOrColinearXZ(start1, end1, start2) != RightOrColinearXZ(start1, end1, end2) && RightOrColinearXZ(start2, end2, start1) != RightOrColinearXZ(start2, end2, end1);
		}

		
		public static bool SegmentsIntersectXZ (Vector3 start1, Vector3 end1, Vector3 start2, Vector3 end2) {
			Vector3 dir1 = end1-start1;
			Vector3 dir2 = end2-start2;

			float den = dir2.z*dir1.x - dir2.x * dir1.z;

			if (den == 0) {
				return false;
			}

			float nom = dir2.x*(start1.z-start2.z)- dir2.z*(start1.x-start2.x);
			float nom2 = dir1.x*(start1.z-start2.z) - dir1.z * (start1.x - start2.x);
			float u = nom/den;
			float u2 = nom2/den;

			if (u < 0F || u > 1F || u2 < 0F || u2 > 1F) {
				return false;
			}

			return true;
		}

		
		public static Vector3 LineDirIntersectionPointXZ (Vector3 start1, Vector3 dir1, Vector3 start2, Vector3 dir2) {
			float den = dir2.z*dir1.x - dir2.x * dir1.z;

			if (den == 0) {
				return start1;
			}

			float nom = dir2.x*(start1.z-start2.z)- dir2.z*(start1.x-start2.x);
			float u = nom/den;

			return start1 + dir1*u;
		}

		
		public static Vector3 LineDirIntersectionPointXZ (Vector3 start1, Vector3 dir1, Vector3 start2, Vector3 dir2, out bool intersects) {
			float den = dir2.z*dir1.x - dir2.x * dir1.z;

			if (den == 0) {
				intersects = false;
				return start1;
			}

			float nom = dir2.x*(start1.z-start2.z)- dir2.z*(start1.x-start2.x);
			float u = nom/den;

			intersects = true;
			return start1 + dir1*u;
		}

		
		public static bool RaySegmentIntersectXZ (Int3 start1, Int3 end1, Int3 start2, Int3 end2) {
			Int3 dir1 = end1-start1;
			Int3 dir2 = end2-start2;

			long den = dir2.z*dir1.x - dir2.x * dir1.z;

			if (den == 0) {
				return false;
			}

			long nom = dir2.x*(start1.z-start2.z)- dir2.z*(start1.x-start2.x);
			long nom2 = dir1.x*(start1.z-start2.z) - dir1.z * (start1.x - start2.x);

			//factor1 < 0
			// If both have the same sign, then nom/den < 0 and thus the segment cuts the ray before the ray starts
			if (!(nom < 0 ^ den < 0)) {
				return false;
			}

			//factor2 < 0
			if (!(nom2 < 0 ^ den < 0)) {
				return false;
			}

			if ((den >= 0 && nom2 > den) || (den < 0 && nom2 <= den)) {
				return false;
			}

			return true;
		}

		
		public static bool LineIntersectionFactorXZ (Int3 start1, Int3 end1, Int3 start2, Int3 end2, out float factor1, out float factor2) {
			Int3 dir1 = end1-start1;
			Int3 dir2 = end2-start2;

			long den = dir2.z*dir1.x - dir2.x * dir1.z;

			if (den == 0) {
				factor1 = 0;
				factor2 = 0;
				return false;
			}

			long nom = dir2.x*(start1.z-start2.z)- dir2.z*(start1.x-start2.x);
			long nom2 = dir1.x*(start1.z-start2.z) - dir1.z * (start1.x - start2.x);

			factor1 = (float)nom/den;
			factor2 = (float)nom2/den;

			return true;
		}

		
		public static bool LineIntersectionFactorXZ (Vector3 start1, Vector3 end1, Vector3 start2, Vector3 end2, out float factor1, out float factor2) {
			Vector3 dir1 = end1-start1;
			Vector3 dir2 = end2-start2;

			float den = dir2.z*dir1.x - dir2.x * dir1.z;

			if (den <= 0.00001f && den >= -0.00001f) {
				factor1 = 0;
				factor2 = 0;
				return false;
			}

			float nom = dir2.x*(start1.z-start2.z)- dir2.z*(start1.x-start2.x);
			float nom2 = dir1.x*(start1.z-start2.z) - dir1.z * (start1.x - start2.x);

			float u = nom/den;
			float u2 = nom2/den;

			factor1 = u;
			factor2 = u2;

			return true;
		}

		
		public static float LineRayIntersectionFactorXZ (Int3 start1, Int3 end1, Int3 start2, Int3 end2) {
			Int3 dir1 = end1-start1;
			Int3 dir2 = end2-start2;

			int den = dir2.z*dir1.x - dir2.x * dir1.z;

			if (den == 0) {
				return float.NaN;
			}

			int nom = dir2.x*(start1.z-start2.z)- dir2.z*(start1.x-start2.x);
			int nom2 = dir1.x*(start1.z-start2.z) - dir1.z * (start1.x - start2.x);

			if ((float)nom2/den < 0) {
				return float.NaN;
			}
			return (float)nom/den;
		}

		
		public static float LineIntersectionFactorXZ (Vector3 start1, Vector3 end1, Vector3 start2, Vector3 end2) {
			Vector3 dir1 = end1-start1;
			Vector3 dir2 = end2-start2;

			float den = dir2.z*dir1.x - dir2.x * dir1.z;

			if (den == 0) {
				return -1;
			}

			float nom = dir2.x*(start1.z-start2.z)- dir2.z*(start1.x-start2.x);
			float u = nom/den;

			return u;
		}

		public static Vector3 LineIntersectionPointXZ (Vector3 start1, Vector3 end1, Vector3 start2, Vector3 end2) {
			bool s;

			return LineIntersectionPointXZ(start1, end1, start2, end2, out s);
		}

		public static Vector3 LineIntersectionPointXZ (Vector3 start1, Vector3 end1, Vector3 start2, Vector3 end2, out bool intersects) {
			Vector3 dir1 = end1-start1;
			Vector3 dir2 = end2-start2;

			float den = dir2.z*dir1.x - dir2.x * dir1.z;

			if (den == 0) {
				intersects = false;
				return start1;
			}

			float nom = dir2.x*(start1.z-start2.z)- dir2.z*(start1.x-start2.x);

			float u = nom/den;

			intersects = true;
			return start1 + dir1*u;
		}

		public static Vector2 LineIntersectionPoint (Vector2 start1, Vector2 end1, Vector2 start2, Vector2 end2) {
			bool s;

			return LineIntersectionPoint(start1, end1, start2, end2, out s);
		}

		public static Vector2 LineIntersectionPoint (Vector2 start1, Vector2 end1, Vector2 start2, Vector2 end2, out bool intersects) {
			Vector2 dir1 = end1-start1;
			Vector2 dir2 = end2-start2;

			float den = dir2.y*dir1.x - dir2.x * dir1.y;

			if (den == 0) {
				intersects = false;
				return start1;
			}

			float nom = dir2.x*(start1.y-start2.y)- dir2.y*(start1.x-start2.x);

			float u = nom/den;

			intersects = true;
			return start1 + dir1*u;
		}

		
		public static Vector3 SegmentIntersectionPointXZ (Vector3 start1, Vector3 end1, Vector3 start2, Vector3 end2, out bool intersects) {
			Vector3 dir1 = end1-start1;
			Vector3 dir2 = end2-start2;

			float den = dir2.z * dir1.x - dir2.x * dir1.z;

			if (den == 0) {
				intersects = false;
				return start1;
			}

			float nom = dir2.x*(start1.z-start2.z)- dir2.z*(start1.x-start2.x);
			float nom2 = dir1.x*(start1.z-start2.z) - dir1.z*(start1.x-start2.x);
			float u = nom/den;
			float u2 = nom2/den;

			if (u < 0F || u > 1F || u2 < 0F || u2 > 1F) {
				intersects = false;
				return start1;
			}

			intersects = true;
			return start1 + dir1*u;
		}

		
		public static bool SegmentIntersectsBounds (Bounds bounds, Vector3 a, Vector3 b) {
			// Put segment in box space
			a -= bounds.center;
			b -= bounds.center;

			// Get line midpoint and extent
			var LMid = (a + b) * 0.5F;
			var L = (a - LMid);
			var LExt = new Vector3(Math.Abs(L.x), Math.Abs(L.y), Math.Abs(L.z));

			Vector3 extent = bounds.extents;

			// Use Separating Axis Test
			// Separation vector from box center to segment center is LMid, since the line is in box space
			if (Math.Abs(LMid.x) > extent.x + LExt.x) return false;
			if (Math.Abs(LMid.y) > extent.y + LExt.y) return false;
			if (Math.Abs(LMid.z) > extent.z + LExt.z) return false;
			// Crossproducts of line and each axis
			if (Math.Abs(LMid.y * L.z - LMid.z * L.y) > (extent.y * LExt.z + extent.z * LExt.y)) return false;
			if (Math.Abs(LMid.x * L.z - LMid.z * L.x) > (extent.x * LExt.z + extent.z * LExt.x)) return false;
			if (Math.Abs(LMid.x * L.y - LMid.y * L.x) > (extent.x * LExt.y + extent.y * LExt.x)) return false;
			// No separating axis, the line intersects
			return true;
		}

		
		public static float LineCircleIntersectionFactor (Vector3 circleCenter, Vector3 linePoint1, Vector3 linePoint2, float radius) {
			float segmentLength;
			var normalizedDirection = Normalize(linePoint2 - linePoint1, out segmentLength);
			var dirToStart = linePoint1 - circleCenter;

			var dot = Vector3.Dot(dirToStart, normalizedDirection);
			var discriminant = dot * dot - (dirToStart.sqrMagnitude - radius*radius);

			if (discriminant < 0) {
				// No intersection, pick closest point on segment
				discriminant = 0;
			}

			var t = -dot + Mathf.Sqrt(discriminant);
			return segmentLength > 0.00001f ? t / segmentLength : 1f;
		}

		
		public static bool ReversesFaceOrientations (Matrix4x4 matrix) {
			var dX = matrix.MultiplyVector(new Vector3(1, 0, 0));
			var dY = matrix.MultiplyVector(new Vector3(0, 1, 0));
			var dZ = matrix.MultiplyVector(new Vector3(0, 0, 1));

			// Calculate the signed volume of the parallelepiped
			var volume = Vector3.Dot(Vector3.Cross(dX, dY), dZ);

			return volume < 0;
		}

		
		public static bool ReversesFaceOrientationsXZ (Matrix4x4 matrix) {
			var dX = matrix.MultiplyVector(new Vector3(1, 0, 0));
			var dZ = matrix.MultiplyVector(new Vector3(0, 0, 1));

			// Take the cross product of the vectors projected onto the XZ plane
			var cross = (dX.x*dZ.z - dZ.x*dX.z);

			return cross < 0;
		}

		
		public static Vector3 Normalize (Vector3 v, out float magnitude) {
			magnitude = v.magnitude;
			if (magnitude > 1E-05f) {
				return v / magnitude;
			} else {
				return Vector3.zero;
			}
		}

		
		public static Vector2 Normalize (Vector2 v, out float magnitude) {
			magnitude = v.magnitude;
			if (magnitude > 1E-05f) {
				return v / magnitude;
			} else {
				return Vector2.zero;
			}
		}

		/* Clamp magnitude along the X and Z axes.
		 * The y component will not be changed.
		 */
		public static Vector3 ClampMagnitudeXZ (Vector3 v, float maxMagnitude) {
			float squaredMagnitudeXZ = v.x*v.x + v.z*v.z;

			if (squaredMagnitudeXZ > maxMagnitude*maxMagnitude && maxMagnitude > 0) {
				var factor = maxMagnitude / Mathf.Sqrt(squaredMagnitudeXZ);
				v.x *= factor;
				v.z *= factor;
			}
			return v;
		}

		/* Magnitude in the XZ plane */
		public static float MagnitudeXZ (Vector3 v) {
			return Mathf.Sqrt(v.x*v.x + v.z*v.z);
		}
	}

	
	public static class AstarMath {
		public static float MapTo (float startMin, float startMax, float targetMin, float targetMax, float value) {
			return Mathf.Lerp(targetMin, targetMax, Mathf.InverseLerp(startMin, startMax, value));
		}

		public static string FormatBytesBinary (int bytes) {
			double sign = bytes >= 0 ? 1D : -1D;

			bytes = Mathf.Abs(bytes);

			if (bytes < 1024) {
				return (bytes*sign)+" bytes";
			} else if (bytes < 1024*1024) {
				return ((bytes/1024D)*sign).ToString("0.0") + " KiB";
			} else if (bytes < 1024*1024*1024) {
				return ((bytes/(1024D*1024D))*sign).ToString("0.0") +" MiB";
			}
			return ((bytes/(1024D*1024D*1024D))*sign).ToString("0.0") +" GiB";
		}

		
		static int Bit (int a, int b) {
			return (a >> b) & 1;
		}

		
		public static Color IntToColor (int i, float a) {
			int r = Bit(i, 2) + Bit(i, 3) * 2 + 1;
			int g = Bit(i, 1) + Bit(i, 4) * 2 + 1;
			int b = Bit(i, 0) + Bit(i, 5) * 2 + 1;

			return new Color(r*0.25F, g*0.25F, b*0.25F, a);
		}

		
		public static Color HSVToRGB (float h, float s, float v) {
			float r = 0, g = 0, b = 0;

			float Chroma = s * v;
			float Hdash = h / 60.0f;
			float X = Chroma * (1.0f - System.Math.Abs((Hdash % 2.0f) - 1.0f));

			if (Hdash < 1.0f) {
				r = Chroma;
				g = X;
			} else if (Hdash < 2.0f) {
				r = X;
				g = Chroma;
			} else if (Hdash < 3.0f) {
				g = Chroma;
				b = X;
			} else if (Hdash < 4.0f) {
				g = X;
				b = Chroma;
			} else if (Hdash < 5.0f) {
				r = X;
				b = Chroma;
			} else if (Hdash < 6.0f) {
				r = Chroma;
				b = X;
			}

			float Min = v - Chroma;

			r += Min;
			g += Min;
			b += Min;

			return new Color(r, g, b);
		}
	}

	
	public static class Polygon {
		
		public static bool ContainsPointXZ (Vector3 a, Vector3 b, Vector3 c, Vector3 p) {
			return VectorMath.IsClockwiseMarginXZ(a, b, p) && VectorMath.IsClockwiseMarginXZ(b, c, p) && VectorMath.IsClockwiseMarginXZ(c, a, p);
		}

		
		public static bool ContainsPointXZ (Int3 a, Int3 b, Int3 c, Int3 p) {
			return VectorMath.IsClockwiseOrColinearXZ(a, b, p) && VectorMath.IsClockwiseOrColinearXZ(b, c, p) && VectorMath.IsClockwiseOrColinearXZ(c, a, p);
		}

		
		public static bool ContainsPoint (Int2 a, Int2 b, Int2 c, Int2 p) {
			return VectorMath.IsClockwiseOrColinear(a, b, p) && VectorMath.IsClockwiseOrColinear(b, c, p) && VectorMath.IsClockwiseOrColinear(c, a, p);
		}

		
		public static bool ContainsPoint (Vector2[] polyPoints, Vector2 p) {
			int j = polyPoints.Length-1;
			bool inside = false;

			for (int i = 0; i < polyPoints.Length; j = i++) {
				if (((polyPoints[i].y <= p.y && p.y < polyPoints[j].y) || (polyPoints[j].y <= p.y && p.y < polyPoints[i].y)) &&
					(p.x < (polyPoints[j].x - polyPoints[i].x) * (p.y - polyPoints[i].y) / (polyPoints[j].y - polyPoints[i].y) + polyPoints[i].x))
					inside = !inside;
			}
			return inside;
		}

		
		public static bool ContainsPointXZ (Vector3[] polyPoints, Vector3 p) {
			int j = polyPoints.Length-1;
			bool inside = false;

			for (int i = 0; i < polyPoints.Length; j = i++) {
				if (((polyPoints[i].z <= p.z && p.z < polyPoints[j].z) || (polyPoints[j].z <= p.z && p.z < polyPoints[i].z)) &&
					(p.x < (polyPoints[j].x - polyPoints[i].x) * (p.z - polyPoints[i].z) / (polyPoints[j].z - polyPoints[i].z) + polyPoints[i].x))
					inside = !inside;
			}
			return inside;
		}

		
		public static int SampleYCoordinateInTriangle (Int3 p1, Int3 p2, Int3 p3, Int3 p) {
			double det = ((double)(p2.z - p3.z)) * (p1.x - p3.x) + ((double)(p3.x - p2.x)) * (p1.z - p3.z);

			double lambda1 = ((((double)(p2.z - p3.z)) * (p.x - p3.x) + ((double)(p3.x - p2.x)) * (p.z - p3.z)) / det);
			double lambda2 = ((((double)(p3.z - p1.z)) * (p.x - p3.x) + ((double)(p1.x - p3.x)) * (p.z - p3.z)) / det);

			return (int)Math.Round(lambda1 * p1.y + lambda2 * p2.y + (1 - lambda1 - lambda2) * p3.y);
		}

		
		public static Vector3[] ConvexHullXZ (Vector3[] points) {
			if (points.Length == 0) return new Vector3[0];

			var hull = Pathfinding.Util.ListPool<Vector3>.Claim();

			int pointOnHull = 0;
			for (int i = 1; i < points.Length; i++) if (points[i].x < points[pointOnHull].x) pointOnHull = i;

			int startpoint = pointOnHull;
			int counter = 0;

			do {
				hull.Add(points[pointOnHull]);
				int endpoint = 0;
				for (int i = 0; i < points.Length; i++) if (endpoint == pointOnHull || !VectorMath.RightOrColinearXZ(points[pointOnHull], points[endpoint], points[i])) endpoint = i;

				pointOnHull = endpoint;

				counter++;
				if (counter > 10000) {
					Debug.LogWarning("Infinite Loop in Convex Hull Calculation");
					break;
				}
			} while (pointOnHull != startpoint);

			var result = hull.ToArray();

			// Return to pool
			Pathfinding.Util.ListPool<Vector3>.Release(hull);
			return result;
		}

		
		public static Vector2 ClosestPointOnTriangle (Vector2 a, Vector2 b, Vector2 c, Vector2 p) {
			// Check if p is in vertex region outside A
			var ab = b - a;
			var ac = c - a;
			var ap = p - a;

			var d1 = Vector2.Dot(ab, ap);
			var d2 = Vector2.Dot(ac, ap);

			// Barycentric coordinates (1,0,0)
			if (d1 <= 0 && d2 <= 0) {
				return a;
			}

			// Check if p is in vertex region outside B
			var bp = p - b;
			var d3 = Vector2.Dot(ab, bp);
			var d4 = Vector2.Dot(ac, bp);

			// Barycentric coordinates (0,1,0)
			if (d3 >= 0 && d4 <= d3) {
				return b;
			}

			// Check if p is in edge region outside AB, if so return a projection of p onto AB
			if (d1 >= 0 && d3 <= 0) {
				var vc = d1 * d4 - d3 * d2;
				if (vc <= 0) {
					// Barycentric coordinates (1-v, v, 0)
					var v = d1 / (d1 - d3);
					return a + ab*v;
				}
			}

			// Check if p is in vertex region outside C
			var cp = p - c;
			var d5 = Vector2.Dot(ab, cp);
			var d6 = Vector2.Dot(ac, cp);

			// Barycentric coordinates (0,0,1)
			if (d6 >= 0 && d5 <= d6) {
				return c;
			}

			// Check if p is in edge region of AC, if so return a projection of p onto AC
			if (d2 >= 0 && d6 <= 0) {
				var vb = d5 * d2 - d1 * d6;
				if (vb <= 0) {
					// Barycentric coordinates (1-v, 0, v)
					var v = d2 / (d2 - d6);
					return a + ac*v;
				}
			}

			// Check if p is in edge region of BC, if so return projection of p onto BC
			if ((d4 - d3) >= 0 && (d5 - d6) >= 0) {
				var va = d3 * d6 - d5 * d4;
				if (va <= 0) {
					var v = (d4 - d3) / ((d4 - d3) + (d5 - d6));
					return b + (c - b) * v;
				}
			}

			return p;
		}

		
		public static Vector3 ClosestPointOnTriangleXZ (Vector3 a, Vector3 b, Vector3 c, Vector3 p) {
			// Check if p is in vertex region outside A
			var ab = new Vector2(b.x - a.x, b.z - a.z);
			var ac = new Vector2(c.x - a.x, c.z - a.z);
			var ap = new Vector2(p.x - a.x, p.z - a.z);

			var d1 = Vector2.Dot(ab, ap);
			var d2 = Vector2.Dot(ac, ap);

			// Barycentric coordinates (1,0,0)
			if (d1 <= 0 && d2 <= 0) {
				return a;
			}

			// Check if p is in vertex region outside B
			var bp = new Vector2(p.x - b.x, p.z - b.z);
			var d3 = Vector2.Dot(ab, bp);
			var d4 = Vector2.Dot(ac, bp);

			// Barycentric coordinates (0,1,0)
			if (d3 >= 0 && d4 <= d3) {
				return b;
			}

			// Check if p is in edge region outside AB, if so return a projection of p onto AB
			var vc = d1 * d4 - d3 * d2;
			if (d1 >= 0 && d3 <= 0 && vc <= 0) {
				// Barycentric coordinates (1-v, v, 0)
				var v = d1 / (d1 - d3);
				return (1-v)*a + v*b;
			}

			// Check if p is in vertex region outside C
			var cp = new Vector2(p.x - c.x, p.z - c.z);
			var d5 = Vector2.Dot(ab, cp);
			var d6 = Vector2.Dot(ac, cp);

			// Barycentric coordinates (0,0,1)
			if (d6 >= 0 && d5 <= d6) {
				return c;
			}

			// Check if p is in edge region of AC, if so return a projection of p onto AC
			var vb = d5 * d2 - d1 * d6;
			if (d2 >= 0 && d6 <= 0 && vb <= 0) {
				// Barycentric coordinates (1-v, 0, v)
				var v = d2 / (d2 - d6);
				return (1-v)*a + v*c;
			}

			// Check if p is in edge region of BC, if so return projection of p onto BC
			var va = d3 * d6 - d5 * d4;
			if ((d4 - d3) >= 0 && (d5 - d6) >= 0 && va <= 0) {
				var v = (d4 - d3) / ((d4 - d3) + (d5 - d6));
				return b + (c - b) * v;
			} else {
				// P is inside the face region. Compute the point using its barycentric coordinates (u, v, w)
				// Note that the x and z coordinates will be exactly the same as P's x and z coordinates
				var denom = 1f / (va + vb + vc);
				var v = vb * denom;
				var w = vc * denom;

				return new Vector3(p.x, (1 - v - w)*a.y + v*b.y + w*c.y, p.z);
			}
		}

		
		public static Vector3 ClosestPointOnTriangle (Vector3 a, Vector3 b, Vector3 c, Vector3 p) {
			// Check if p is in vertex region outside A
			var ab = b - a;
			var ac = c - a;
			var ap = p - a;

			var d1 = Vector3.Dot(ab, ap);
			var d2 = Vector3.Dot(ac, ap);

			// Barycentric coordinates (1,0,0)
			if (d1 <= 0 && d2 <= 0)
				return a;

			// Check if p is in vertex region outside B
			var bp = p - b;
			var d3 = Vector3.Dot(ab, bp);
			var d4 = Vector3.Dot(ac, bp);

			// Barycentric coordinates (0,1,0)
			if (d3 >= 0 && d4 <= d3)
				return b;

			// Check if p is in edge region outside AB, if so return a projection of p onto AB
			var vc = d1 * d4 - d3 * d2;
			if (d1 >= 0 && d3 <= 0 && vc <= 0) {
				// Barycentric coordinates (1-v, v, 0)
				var v = d1 / (d1 - d3);
				return a + ab * v;
			}

			// Check if p is in vertex region outside C
			var cp = p - c;
			var d5 = Vector3.Dot(ab, cp);
			var d6 = Vector3.Dot(ac, cp);

			// Barycentric coordinates (0,0,1)
			if (d6 >= 0 && d5 <= d6)
				return c;

			// Check if p is in edge region of AC, if so return a projection of p onto AC
			var vb = d5 * d2 - d1 * d6;
			if (d2 >= 0 && d6 <= 0 && vb <= 0) {
				// Barycentric coordinates (1-v, 0, v)
				var v = d2 / (d2 - d6);
				return a + ac * v;
			}

			// Check if p is in edge region of BC, if so return projection of p onto BC
			var va = d3 * d6 - d5 * d4;
			if ((d4 - d3) >= 0 && (d5 - d6) >= 0 && va <= 0) {
				var v = (d4 - d3) / ((d4 - d3) + (d5 - d6));
				return b + (c - b) * v;
			} else {
				// P is inside the face region. Compute the point using its barycentric coordinates (u, v, w)
				var denom = 1f / (va + vb + vc);
				var v = vb * denom;
				var w = vc * denom;

				// This is equal to: u*a + v*b + w*c, u = va*denom = 1 - v - w;
				return a + ab * v + ac * w;
			}
		}

		/// <summary>Cached dictionary to avoid excessive allocations</summary>
		static readonly Dictionary<Int3, int> cached_Int3_int_dict = new Dictionary<Int3, int>();

		/// <summary>
		/// Compress the mesh by removing duplicate vertices.
		///
		/// Vertices that differ by only 1 along the y coordinate will also be merged together.
		/// Warning: This function is not threadsafe. It uses some cached structures to reduce allocations.
		/// </summary>
		/// <param name="vertices">Vertices of the input mesh</param>
		/// <param name="triangles">Triangles of the input mesh</param>
		/// <param name="outVertices">Vertices of the output mesh.</param>
		/// <param name="outTriangles">Triangles of the output mesh.</param>
		public static void CompressMesh (List<Int3> vertices, List<int> triangles, out Int3[] outVertices, out int[] outTriangles) {
			Dictionary<Int3, int> firstVerts = cached_Int3_int_dict;
			firstVerts.Clear();

			// Use cached array to reduce memory allocations
			int[] compressedPointers = ArrayPool<int>.Claim(vertices.Count);

			// Map positions to the first index they were encountered at
			int count = 0;
			for (int i = 0; i < vertices.Count; i++) {
				// Check if the vertex position has already been added
				// Also check one position up and one down because rounding errors can cause vertices
				// that should end up in the same position to be offset 1 unit from each other
				// TODO: Check along X and Z axes as well?
				int ind;
				if (!firstVerts.TryGetValue(vertices[i], out ind) && !firstVerts.TryGetValue(vertices[i] + new Int3(0, 1, 0), out ind) && !firstVerts.TryGetValue(vertices[i] + new Int3(0, -1, 0), out ind)) {
					firstVerts.Add(vertices[i], count);
					compressedPointers[i] = count;
					vertices[count] = vertices[i];
					count++;
				} else {
					compressedPointers[i] = ind;
				}
			}

			// Create the triangle array or reuse the existing buffer
			outTriangles = new int[triangles.Count];

			// Remap the triangles to the new compressed indices
			for (int i = 0; i < outTriangles.Length; i++) {
				outTriangles[i] = compressedPointers[triangles[i]];
			}

			// Create the vertex array or reuse the existing buffer
			outVertices = new Int3[count];

			for (int i = 0; i < count; i++)
				outVertices[i] = vertices[i];

			ArrayPool<int>.Release(ref compressedPointers);
		}

		/// <summary>
		/// Given a set of edges between vertices, follows those edges and returns them as chains and cycles.
		///
		/// [Open online documentation to see images]
		/// </summary>
		/// <param name="outline">outline[a] = b if there is an edge from a to b.</param>
		/// <param name="hasInEdge">hasInEdge should contain b if outline[a] = b for any key a.</param>
		/// <param name="results">Will be called once for each contour with the contour as a parameter as well as a boolean indicating if the contour is a cycle or a chain (see image).</param>
		public static void TraceContours (Dictionary<int, int> outline, HashSet<int> hasInEdge, System.Action<List<int>, bool> results) {
			// Iterate through chains of the navmesh outline.
			// I.e segments of the outline that are not loops
			// we need to start these at the beginning of the chain.
			// Then iterate over all the loops of the outline.
			// Since they are loops, we can start at any point.
			var obstacleVertices = ListPool<int>.Claim();
			var outlineKeys = ListPool<int>.Claim();

			outlineKeys.AddRange(outline.Keys);
			for (int k = 0; k <= 1; k++) {
				bool cycles = k == 1;
				for (int i = 0; i < outlineKeys.Count; i++) {
					var startIndex = outlineKeys[i];

					// Chains (not cycles) need to start at the start of the chain
					// Cycles can start at any point
					if (!cycles && hasInEdge.Contains(startIndex)) {
						continue;
					}

					var index = startIndex;
					obstacleVertices.Clear();
					obstacleVertices.Add(index);

					while (outline.ContainsKey(index)) {
						var next = outline[index];
						outline.Remove(index);

						obstacleVertices.Add(next);

						// We traversed a full cycle
						if (next == startIndex) break;

						index = next;
					}

					if (obstacleVertices.Count > 1) {
						results(obstacleVertices, cycles);
					}
				}
			}

			ListPool<int>.Release(ref outlineKeys);
			ListPool<int>.Release(ref obstacleVertices);
		}

		/// <summary>Divides each segment in the list into subSegments segments and fills the result list with the new points</summary>
		public static void Subdivide (List<Vector3> points, List<Vector3> result, int subSegments) {
			for (int i = 0; i < points.Count-1; i++)
				for (int j = 0; j < subSegments; j++)
					result.Add(Vector3.Lerp(points[i], points[i+1], j / (float)subSegments));

			result.Add(points[points.Count-1]);
		}
	}
}
