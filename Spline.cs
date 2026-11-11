using System;
using System.Collections.Generic;
using UnityEngine;

namespace GatorDragonGames {

public class Spline {
    public const int BINORMAL_LUT_COUNT = 32;
    public const int DISTANCE_LUT_COUNT = 32;
    
    public struct SplineKnot {
        public Vector3 backHandle;
        public Vector3 position;
        public Vector3 frontHandle;
    }

    private bool distanceLUTGenerated = false;
    private bool binormalLUTGenerated = false;
    private List<Matrix4x4> weights = new();
    private float[] distanceLUT = new float[DISTANCE_LUT_COUNT];
    private Vector3[] binormalLUT = new Vector3[BINORMAL_LUT_COUNT];
    private List<Bounds> bounds = new();

    private float _arcLength;

    public float arcLength {
        get {
            if (!distanceLUTGenerated) {
                GenerateDistanceLUT();
            }

            return _arcLength;
        }
        private set => _arcLength = value;
    }

    public Spline() {
        SetFromCatmull(new[] { Vector3.zero, Vector3.one }, false);
    }

    private Spline(IList<Matrix4x4> weights) {
        SetWeights(weights);
    }

    private static Spline Lerp(Spline a, Spline b, float t) {
        List<Matrix4x4> merged = new List<Matrix4x4>(a.weights);
        for (int i = 0; i < merged.Count; i++) {
            Vector4 column1 = Vector4.Lerp(merged[i].GetColumn(0), b.weights[i].GetColumn(0), t);
            Vector4 column2 = Vector4.Lerp(merged[i].GetColumn(1), b.weights[i].GetColumn(1), t);
            Vector4 column3 = Vector4.Lerp(merged[i].GetColumn(2), b.weights[i].GetColumn(2), t);
            Vector4 column4 = Vector4.Lerp(merged[i].GetColumn(3), b.weights[i].GetColumn(3), t);
            merged[i] = new Matrix4x4(column1, column2, column3, column4);
        }

        return new Spline(merged);
    }

    private static Vector3 GetPosition(Matrix4x4 weightBlock, float t) {
        return weightBlock * new Vector4(1f, t, t * t, t * t * t);
    }

    private static Vector3 GetVelocity(Matrix4x4 weightBlock, float t) {
        return weightBlock * new Vector4(0f, 1f, 2f * t, 3f * t * t);
    }

    private static Vector3 GetAcceleration(Matrix4x4 weightBlock, float t) {
        return weightBlock * new Vector4(0f, 0f, 2f, 6f * t);
    }

    public List<Matrix4x4> GetWeights() => weights;

    private float[] GetDistanceLUT() {
        if (!distanceLUTGenerated) {
            GenerateDistanceLUT();
        }

        return distanceLUT;
    }

    private Vector3[] GetBinormalLUT() {
        if (!binormalLUTGenerated) {
            GenerateBinormalLUT();
        }

        return binormalLUT;
    }

    public List<Bounds> GetBounds() {
        if (bounds.Count != weights.Count) {
            GenerateBounds();
        }

        return bounds;
    }

    private Vector3 SampleCurveSegmentPosition(int curveSegmentIndex, float t) {
        return GetPosition(weights[curveSegmentIndex], t);
    }

    private Vector3 SampleCurveSegmentVelocity(int curveSegmentIndex, float t) {
        return GetVelocity(weights[curveSegmentIndex], t);
    }

    private Vector3 SampleCurveSegmentAcceleration(int curveSegmentIndex, float t) {
        return GetAcceleration(weights[curveSegmentIndex], t);
    }

    private float GetCurveSegmentTimeFromCurveTime(out int curveSegmentIndex, float t) {
        curveSegmentIndex = Mathf.Clamp(Mathf.FloorToInt(t * (weights.Count)), 0, (weights.Count) - 1);
        var offseted = t - ((float)curveSegmentIndex / (float)(weights.Count));
        return offseted * (float)(weights.Count);
    }

    public float GetTimeFromDistance(float distance) {
        if (!distanceLUTGenerated) {
            GenerateDistanceLUT();
        }

        if (distance > 0f && distance < arcLength) {
            for (int i = 0; i < DISTANCE_LUT_COUNT - 1; i++) {
                if (distance > distanceLUT[i] && distance < distanceLUT[i + 1]) {
                    return Remap(distance, distanceLUT[i], distanceLUT[i + 1],
                        (float)i / (float)(DISTANCE_LUT_COUNT - 1), (float)(i + 1) / (float)(DISTANCE_LUT_COUNT - 1));
                }
            }
        }

        return distance / arcLength;
    }

    public float GetDistanceFromTime(float t) {
        if (!distanceLUTGenerated) {
            GenerateDistanceLUT();
        }

        t = Mathf.Clamp01(t);
        int index = Mathf.Clamp(Mathf.FloorToInt(t * (DISTANCE_LUT_COUNT - 1)), 0, DISTANCE_LUT_COUNT - 2);
        float offseted = t - ((float)index / (float)(DISTANCE_LUT_COUNT - 1));
        float lerpT = offseted * (float)(DISTANCE_LUT_COUNT - 1);
        return Mathf.Lerp(distanceLUT[index], distanceLUT[index + 1], lerpT);
    }

    private bool CheckMinimaMaxima(float tValue) {
        return !float.IsNaN(tValue) && Mathf.Clamp01(tValue) == tValue;
    }

    // Again we use Freya Holmer's tutorial to understand the strategy behind generating the bounds: https://youtu.be/aVwxzDHniEw?t=791
    // This is just the derivative written in terms of t, then plugged into the quadratic formula to solve for zeros.
    // that gives us our local extremes to generate our bounds on.
    protected void GenerateBounds() {
        bounds.Clear();
        foreach (var weightGroup in weights) {
            Vector3 a = weightGroup * new Vector4(0f, 0f, 0f, 3f);
            Vector3 b = weightGroup * new Vector4(0f, 0f, 2f, 0f);
            Vector3 c = weightGroup * new Vector4(0f, 1f, 0f, 0f);

            // Good ol' quadratic formula. We solve it for each axis (X,Y,Z);
            float tpx = (-b.x + Mathf.Sqrt(b.x * b.x - 4f * a.x * c.x)) / (2f * a.x);
            float tpy = (-b.y + Mathf.Sqrt(b.y * b.y - 4f * a.y * c.y)) / (2f * a.y);
            float tpz = (-b.z + Mathf.Sqrt(b.z * b.z - 4f * a.z * c.z)) / (2f * a.z);

            float tnx = (-b.x - Mathf.Sqrt(b.x * b.x - 4f * a.x * c.x)) / (2f * a.x);
            float tny = (-b.y - Mathf.Sqrt(b.y * b.y - 4f * a.y * c.y)) / (2f * a.y);
            float tnz = (-b.z - Mathf.Sqrt(b.z * b.z - 4f * a.z * c.z)) / (2f * a.z);

            Bounds bound = new Bounds(GetPosition(weightGroup, 0f), Vector3.zero);
            // If the floats are out of our range, or if they're NaN (from a negative sqrt)-- we discard them.
            if (CheckMinimaMaxima(tpx)) {
                bound.Encapsulate(GetPosition(weightGroup, tpx));
            }

            if (CheckMinimaMaxima(tnx)) {
                bound.Encapsulate(GetPosition(weightGroup, tnx));
            }

            if (CheckMinimaMaxima(tpy)) {
                bound.Encapsulate(GetPosition(weightGroup, tpy));
            }

            if (CheckMinimaMaxima(tny)) {
                bound.Encapsulate(GetPosition(weightGroup, tny));
            }

            if (CheckMinimaMaxima(tpz)) {
                bound.Encapsulate(GetPosition(weightGroup, tpz));
            }

            if (CheckMinimaMaxima(tnz)) {
                bound.Encapsulate(GetPosition(weightGroup, tnz));
            }

            bound.Encapsulate(GetPosition(weightGroup, 1f));
            bounds.Add(bound);
        }
    }

    private void GenerateDistanceLUT() {
        float dist = 0f;
        Vector3 lastPosition = GetPositionFromT(0f);
        distanceLUT[0] = 0f;
        for (int i = 1; i < DISTANCE_LUT_COUNT; i++) {
            float t = (float)i / (DISTANCE_LUT_COUNT - 1);
            Vector3 position = GetPositionFromT(t);
            dist += Vector3.Distance(lastPosition, position);
            lastPosition = position;
            distanceLUT[i] = dist;
        }

        distanceLUTGenerated = true;
        arcLength = dist;
    }

    private void GenerateBinormalLUT() {
        // https://janakiev.com/blog/framing-parametric-curves/
        // https://legacy.cs.indiana.edu/ftp/techreports/TR425.pdf
        // Using parallel transport frames
        Vector3 lastTangent = GetVelocityFromT(0).normalized;
        // Initial reference frame, uses Vector3.up
        Vector3 lastBinormal = Vector3.Cross(GetVelocityFromT(0), Vector3.up).normalized;
        if (lastBinormal == Vector3.zero) {
            lastBinormal = Vector3.Cross(GetVelocityFromT(0), -Vector3.forward).normalized;
        }

        binormalLUT[0] = lastBinormal;
        for (int i = 1; i < BINORMAL_LUT_COUNT; i++) {
            float t = (float)i / (BINORMAL_LUT_COUNT);
            Vector3 tangent = GetVelocityFromT(t).normalized;
            Vector3 binormal = Vector3.Cross(lastTangent, tangent);
            if (binormal == Vector3.zero) {
                binormal = lastBinormal;
            } else {
                float theta = Mathf.Acos(Mathf.Clamp(Vector3.Dot(lastTangent, tangent), -1f, 1f));
                binormal = Quaternion.AngleAxis(theta * Mathf.Rad2Deg, binormal.normalized) * lastBinormal;
            }

            lastTangent = tangent;
            lastBinormal = binormal;
            binormalLUT[i] = binormal;
        }

        // Undo any twist.
        //float overallAngle = Vector3.Angle(binormalLUT[0], binormalLUT[resolution-1]);
        //for(int i=0;i<resolution;i++) {
        //float t = (float)i/(float)resolution;
        //binormalLUT[i] = Quaternion.AngleAxis(-overallAngle*t, GetVelocityFromT(t).normalized)*binormalLUT[i];
        //}
        binormalLUTGenerated = true;
    }

    private Spline SetWeights(IList<Matrix4x4> newWeights) {
        weights.Clear();
        weights.AddRange(newWeights);
        distanceLUTGenerated = false;
        binormalLUTGenerated = false;
        bounds.Clear();
        return this;
    }

    private static float Remap(float value, float from1, float to1, float from2, float to2) {
        return (value - from1) / (to1 - from1) * (to2 - from2) + from2;
    }
    
    private static void GetWeightsLooped(Matrix4x4 characteristicMatrix, ICollection<Matrix4x4> weightCollection, IList<Vector3> newPoints) {
        var pointCount = newPoints.Count;
        for (int i = 0; i < pointCount; i++) {
            Vector3 p0 = newPoints[(i+pointCount-1)%pointCount];
            Vector3 p1 = newPoints[i];
            Vector3 p2 = newPoints[(i + 1)%pointCount];
            Vector3 p3 = newPoints[(i + 2)%pointCount];
            var pointMatrix = new Matrix4x4(p0, p1, p2, p3);
            weightCollection.Add(pointMatrix * characteristicMatrix);
        }
    }

    private static void GetWeightsKnots(Matrix4x4 basisMatrix, ICollection<Matrix4x4> weightCollection, IList<SplineKnot> knots) {
        var knotCount = knots.Count;
        if (knotCount < 2) {
            throw new UnityException("Spline must have at least 2 knots.");
        }
        for (int i=0;i<knotCount-1;i++) {
            var knot = knots[i];
            var nextKnot = knots[i+1];
            var p0 = knot.position;
            var p1 = knot.frontHandle;
            var p2 = nextKnot.backHandle;
            var p3 = nextKnot.position;
            var pointMatrix = new Matrix4x4(p0, p1, p2, p3);
            weightCollection.Add(pointMatrix * basisMatrix);
        }
    }

    public float GetLengthFromSubsection(int subsectionCount, int subsectionIndex = 0, float subT = 1f) {
        var multi = (float)subsectionCount / (float)weights.Count;
        var startT = (float)subsectionIndex / (float)weights.Count;
        var t = subT * multi + startT;
        return GetDistanceFromTime(t) - GetDistanceFromTime(startT);
    }

    [Obsolete("Please use GetLengthFromSubsection instead.")]
    public float GetDistanceFromSubT(int start, int end, float subT) {
        int count = end - start;
        return GetLengthFromSubsection(count, start, subT);
    }
    
    /// <summary>
    /// Standard Bezier spline
    /// Knot positions are touched no matter what, handles define the curve between them.
    /// Improperly defined handles can cause corners and pinches.
    /// </summary>
    /// <param name="knots">The spline knots defining the curve.</param>
    /// <returns>The generated Spline</returns>
    public Spline SetFromSplineKnotsBezier(IList<SplineKnot> knots) {
        weights.Clear();
        GetWeightsKnots(new Matrix4x4(
            new Vector4(1f, 0f, 0f, 0f),
            new Vector4(-3f, 3f, 0f, 0f),
            new Vector4(3f, -6f, 3f, 0f),
            new Vector4(-1f, 3f, -3f, 1f)
        ), weights, knots);
        distanceLUTGenerated = false;
        binormalLUTGenerated = false;
        bounds.Clear();
        return this;
    }
    
    /// <summary>
    /// Set the spline from a list of SplineKnots using Hermite interpolation.
    /// The handles of each knot are velocity vectors from zero
    /// The knots are world positions.
    /// </summary>
    /// <param name="knots">Start Position, Start Velocity, End Velocity, End Position</param>
    /// <returns>The generated Spline</returns>
    public Spline SetFromSplineKnotsHermite(IList<SplineKnot> knots) {
        weights.Clear();
        GetWeightsKnots(new Matrix4x4(
            new Vector4(1f, 0f, 0f, 0f),
            new Vector4(0f, 1f, 0f, 0f),
            new Vector4(-3f, -2f, -1f, 3f),
            new Vector4(2f, 1f, 1f, -2)
        ), weights, knots);
        distanceLUTGenerated = false;
        binormalLUTGenerated = false;
        bounds.Clear();
        return this;
    }
    
    /// <summary>
    /// A proper C2 continuous spline, positions are never touched, but damn is the curve smooth...
    /// </summary>
    /// <param name="positions">the anchor positions</param>
    /// <param name="loop">whether the spline should loop</param>
    /// <returns>The generated Spline</returns>
    public Spline SetFromBSpline(IList<Vector3> positions, bool loop) {
        weights.Clear();
        GetWeightsLooped(new Matrix4x4(
            new Vector4(1f, 4f, 1f, 0f) * 1f/6f,
            new Vector4(-3f, 0f, 3f, 0f) * 1f/6f,
            new Vector4(3f, -6f, 3f, 0f) * 1f/6f,
            new Vector4(-1f, 3f, -3f, 1f) * 1f/6f
        ), weights, positions);
        if (!loop) {
            weights.RemoveAt(weights.Count - 1);
        }
        distanceLUTGenerated = false;
        binormalLUTGenerated = false;
        bounds.Clear();
        return this;
    }
    
    /// <summary>
    /// A Catmull rom spline, but with the ability to set the tension. Tension of 0 is standard Catmull-Rom, tension of 1 is linear interpolation.
    /// </summary>
    /// <param name="positions">the anchor positions</param>
    /// <param name="loop">whether the spline should loop</param>
    /// <param name="tension">the tension of the spline</param>
    /// <returns>The generated Spline</returns>
    public Spline SetFromCardinal(IList<Vector3> positions, bool loop, float tension) {
        weights.Clear();
        float s = (1f - tension) / 2f;
        GetWeightsLooped(new Matrix4x4(
            new Vector4(0f, 1f, 0f, 0f),
            new Vector4(-s, 0f, s, 0f),
            new Vector4(2f*s, s-3f, 3f-2f*s, -s),
            new Vector4(-s, 2f-s, s-2f, s)
        ), weights, positions);
        if (!loop) {
            weights.RemoveAt(weights.Count - 1);
        }
        distanceLUTGenerated = false;
        binormalLUTGenerated = false;
        bounds.Clear();
        return this;
    }
    
    /// <summary>
    /// A proper C1 continuous spline, positions are always touched, velocity is smooth.
    /// </summary>
    /// <param name="positions">the anchor positions</param>
    /// <param name="loop">whether the spline should loop</param>
    /// <returns>The generated Spline</returns>
    public Spline SetFromCatmull(IList<Vector3> positions, bool loop) {
        weights.Clear();
        GetWeightsLooped(new Matrix4x4(
            new Vector4(0f, 2f, 0f, 0f)*0.5f,
            new Vector4(-1f, 0f, 1f, 0f)*0.5f,
            new Vector4(2f, -5f, 4f, -1f)*0.5f,
            new Vector4(-1f, 3f, -3f, 1f)*0.5f
        ), weights, positions);
        if (!loop) {
            weights.RemoveAt(weights.Count - 1);
        }
        distanceLUTGenerated = false;
        binormalLUTGenerated = false;
        bounds.Clear();
        return this;
    }

    public float GetClosestTimeFromPositionFast(Vector3 position, int samples = 32) {
        // Broad pass that just finds the closest bounds
        int closestBounds = 0;
        float distToBounds = float.MaxValue;
        List<Bounds> curveBounds = GetBounds();
        for (int i = 0; i < curveBounds.Count; i++) {
            float dist = Vector3.Distance(curveBounds[i].ClosestPoint(position), position);
            if (dist < distToBounds) {
                closestBounds = i;
                distToBounds = dist;
            }
        }

        // With the closest sub-curve found, we can then do some very deliberate samples.
        float closestTValue = 0f;
        float distToCurve = float.MaxValue;
        for (int i = 0; i < samples; i++) {
            float tSample = (float)i / (float)samples;
            Vector3 samplePosition = SampleCurveSegmentPosition(closestBounds, tSample);
            float dist = Vector3.Distance(samplePosition, position);
            if (dist < distToCurve) {
                closestTValue = tSample;
                distToCurve = dist;
            }
        }

        // Just gotta take it from subT to overall t value
        return closestTValue / (float)(curveBounds.Count) + ((float)closestBounds / (float)(curveBounds.Count));
    }

    public float GetClosestTimeFromPosition(Vector3 position, int samples = 32) {
        var closestTValue = 0f;
        var distToCurve = float.MaxValue;
        for (int i = 0; i < samples; i++) {
            var tSample = (float)i / (float)samples;
            Vector3 samplePosition = GetPositionFromT(tSample);
            var dist = Vector3.Distance(samplePosition, position);
            if (dist < distToCurve) {
                closestTValue = tSample;
                distToCurve = dist;
            }
        }

        return closestTValue;
    }

    public Vector3 GetPositionFromDistance(float distance) {
        var t = GetTimeFromDistance(distance);
        return GetPositionFromT(t);
    }

    public Vector3 GetPositionFromT(float t) {
        var subT = GetCurveSegmentTimeFromCurveTime(out var curveSegmentIndex, t);
        return SampleCurveSegmentPosition(curveSegmentIndex, subT);
    }

    public Vector3 GetVelocityFromDistance(float distance) {
        var t = GetTimeFromDistance(distance);
        return GetVelocityFromT(t);
    }

    public Vector3 GetAccelerationFromDistance(float distance) {
        var t = GetTimeFromDistance(distance);
        return GetAccelerationFromT(t);
    }

    public Vector3 GetVelocityFromT(float t) {
        var subT = GetCurveSegmentTimeFromCurveTime(out var curveSegmentIndex, t);
        return SampleCurveSegmentVelocity(curveSegmentIndex, subT);
    }

    public Vector3 GetAccelerationFromT(float t) {
        var subT = GetCurveSegmentTimeFromCurveTime(out var curveSegmentIndex, t);
        return SampleCurveSegmentAcceleration(curveSegmentIndex, subT);
    }

    public Vector3 GetBinormalFromT(float t) {
        if (!binormalLUTGenerated) {
            GenerateBinormalLUT();
        }

        var index = Mathf.Clamp(Mathf.FloorToInt(t * (BINORMAL_LUT_COUNT - 1)), 0, BINORMAL_LUT_COUNT - 2);
        var offseted = t - ((float)index / (float)(BINORMAL_LUT_COUNT - 1));
        var lerpT = offseted * (float)(BINORMAL_LUT_COUNT - 1);
        return Vector3.Lerp(binormalLUT[index], binormalLUT[index + 1], lerpT);
    }

    public Matrix4x4 GetReferenceFrameFromT(float t) {
        Vector3 tangent = GetVelocityFromT(t).normalized;
        Vector3 binormal = GetBinormalFromT(t).normalized;
        Vector3 normal = Vector3.Cross(tangent, binormal);
        Vector3.OrthoNormalize(ref tangent, ref binormal, ref normal);

        // Change of basis https://math.stackexchange.com/questions/3540973/change-of-coordinates-and-change-of-basis-matrices
        // It also shows up here: https://docs.unity3d.com/ScriptReference/Vector3.OrthoNormalize.html
        Matrix4x4 bezierBasis = new Matrix4x4();
        bezierBasis.SetRow(0, binormal); // Our X axis
        bezierBasis.SetRow(1, normal); // Y Axis
        bezierBasis.SetRow(2, tangent); // Z Axis
        bezierBasis[3, 3] = 1f;
        // Change of basis formula is B = P⁻¹ * A * P, where P is the basis transform.
        return bezierBasis.inverse;
    }

    public void DebugDraw(Color splineColor, Color knotColor, float duration = 0f) {
        DebugDrawSpline(this, splineColor, knotColor, duration);
    }
    
    public void GizmoDraw(Color splineColor, Color knotColor, int resolution = 64) {
        GizmosDrawSpline(this, splineColor, knotColor);
    }

    private const float knotSize = 0.025f;

    private static void DrawBox(Vector3 position, Quaternion rotation, Vector3 extents, Color color, float duration = 0f) {
        Debug.DrawLine(rotation * new Vector3(-extents.x, extents.y, -extents.z) + position,
            rotation * new Vector3(extents.x, extents.y, -extents.z) + position, color, duration);
        Debug.DrawLine(rotation * new Vector3(-extents.x, extents.y, -extents.z) + position,
            rotation * new Vector3(-extents.x, extents.y, extents.z) + position, color, duration);
        Debug.DrawLine(rotation * new Vector3(-extents.x, extents.y, extents.z) + position,
            rotation * new Vector3(extents.x, extents.y, extents.z) + position, color, duration);
        Debug.DrawLine(rotation * new Vector3(extents.x, extents.y, -extents.z) + position,
            rotation * new Vector3(extents.x, extents.y, extents.z) + position, color, duration);
        Debug.DrawLine(rotation * new Vector3(-extents.x, -extents.y, -extents.z) + position,
            rotation * new Vector3(-extents.x, extents.y, -extents.z) + position, color, duration);
        Debug.DrawLine(rotation * new Vector3(extents.x, -extents.y, -extents.z) + position,
            rotation * new Vector3(extents.x, extents.y, -extents.z) + position, color, duration);
        Debug.DrawLine(rotation * new Vector3(-extents.x, -extents.y, extents.z) + position,
            rotation * new Vector3(-extents.x, extents.y, extents.z) + position, color, duration);
        Debug.DrawLine(rotation * new Vector3(extents.x, -extents.y, extents.z) + position,
            rotation * new Vector3(extents.x, extents.y, extents.z) + position, color, duration);
        Debug.DrawLine(rotation * new Vector3(-extents.x, -extents.y, -extents.z) + position,
            rotation * new Vector3(extents.x, -extents.y, -extents.z) + position, color, duration);
        Debug.DrawLine(rotation * new Vector3(-extents.x, -extents.y, -extents.z) + position,
            rotation * new Vector3(-extents.x, -extents.y, extents.z) + position, color, duration);
        Debug.DrawLine(rotation * new Vector3(-extents.x, -extents.y, extents.z) + position,
            rotation * new Vector3(extents.x, -extents.y, extents.z) + position, color, duration);
        Debug.DrawLine(rotation * new Vector3(extents.x, -extents.y, -extents.z) + position,
            rotation * new Vector3(extents.x, -extents.y, extents.z) + position, color, duration);
    }

    public static void DebugDrawSpline(Spline spline, Color splineColor, Color knotColor, float duration = 0f) {
        Vector3 current = spline.GetPositionFromT(0f);
        for (int i = 1; i < 64; i++) {
            Vector3 next = spline.GetPositionFromT((float)i / 63);
            Debug.DrawLine(current, next, splineColor, duration);
            current = next;
        }

        for (int i = 0; i < spline.GetWeights().Count; i++) {
            var weight = spline.GetWeights()[i];
            var startPos = GetPosition(weight, 0f);
            var startVel = GetVelocity(weight, 0f).normalized;
            float t = (float)i / (spline.GetWeights().Count - 1);
            var startBinormal = spline.GetBinormalFromT(t);
            Quaternion rot = startVel != startBinormal && startVel != Vector3.zero
                ? Quaternion.LookRotation(startVel, startBinormal)
                : Quaternion.identity;
            DrawBox(startPos, rot, (Vector3.one - Vector3.forward * 0.8f) * knotSize, knotColor, duration);
        }

        var endPos = GetPosition(spline.GetWeights()[^1], 1f);
        var endVel = GetVelocity(spline.GetWeights()[^1], 1f).normalized;
        var endBinormal = spline.GetBinormalFromT(1f);
        Quaternion endRot = endVel != endBinormal && endVel != Vector3.zero
            ? Quaternion.LookRotation(endVel, endBinormal)
            : Quaternion.identity;
        DrawBox(endPos, endRot, (Vector3.one - Vector3.forward * 0.8f) * knotSize, knotColor, duration);
    }

    public static void GizmosDrawSpline(Spline spline, Color splineColor, Color knotColor, int resolution = 64) {
        var oldColor = Gizmos.color;
        Gizmos.color = splineColor;
        Vector3 lastPoint = spline.GetPositionFromT(0f);
        for (int i = 0; i <= resolution; i++) {
            Vector3 newPoint = spline.GetPositionFromT((float)i / resolution);
            Gizmos.DrawLine(lastPoint, newPoint);
            lastPoint = newPoint;
        }

        var save = Gizmos.matrix;
        for (int i = 0; i < spline.GetWeights().Count; i++) {
            var weight = spline.GetWeights()[i];
            float t = (float)i / (spline.GetWeights().Count - 1);
            Vector3 pointA = GetPosition(weight, 0f);
            var startVel = GetVelocity(weight, 0f).normalized;
            var startBinormal = spline.GetBinormalFromT(t);
            Quaternion rot = startVel != startBinormal && startVel != Vector3.zero
                ? Quaternion.LookRotation(startVel, startBinormal)
                : Quaternion.identity;
            Gizmos.color = knotColor;
            Gizmos.matrix = Matrix4x4.TRS(pointA, rot, Vector3.one - Vector3.forward * 0.8f);
            Gizmos.DrawWireCube(Vector3.zero, Vector3.one * knotSize);
        }

        var endPos = GetPosition(spline.GetWeights()[^1], 1f);
        var endVel = GetVelocity(spline.GetWeights()[^1], 1f).normalized;
        var endBinormal = spline.GetBinormalFromT(1);
        Quaternion endRot = endVel != endBinormal && endVel != Vector3.zero
            ? Quaternion.LookRotation(endVel, endBinormal)
            : Quaternion.identity;
        Gizmos.matrix = Matrix4x4.TRS(endPos, endRot, Vector3.one - Vector3.forward * 0.8f);
        Gizmos.DrawWireCube(Vector3.zero, Vector3.one * knotSize);

        Gizmos.matrix = save;
        Gizmos.color = oldColor;
    }
}

}