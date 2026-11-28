using System;
using System.Collections.Generic;
using UnityEngine;

namespace GatorDragonGames {

[Serializable]
public struct Spline3DSpace {
    public Vector3 binormalLUTStartingUp;
    public FixedArrayVector3x32 binormalLUT;
    
    public Spline3DSpace(ref Spline3D spline, Vector3 startingUp) {
        binormalLUT = new FixedArrayVector3x32();
        binormalLUTStartingUp = startingUp;
        // https://janakiev.com/blog/framing-parametric-curves/
        // https://legacy.cs.indiana.edu/ftp/techreports/TR425.pdf
        // Using parallel transport frames
        Vector3 lastTangent = spline.GetVelocityFromT(0).normalized;
        // Initial reference frame, uses Vector3.up
        Vector3 lastBinormal = Vector3.Cross(spline.GetVelocityFromT(0), startingUp).normalized;
        if (lastBinormal == Vector3.zero) {
            lastBinormal = Vector3.Cross(spline.GetVelocityFromT(0), new Vector3(startingUp.z, startingUp.x, startingUp.y)).normalized;
        }
        binormalLUT[0] = lastBinormal;
        for (int i = 1; i < FixedArrayVector3x32.sampleCount; i++) {
            float t = (float)i / (FixedArrayVector3x32.sampleCount);
            Vector3 tangent = spline.GetVelocityFromT(t).normalized;
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
    }
    public Vector3 GetBinormalFromT(float t, Vector3 startingUp) {
        var index = Mathf.Clamp(Mathf.FloorToInt(t * (FixedArrayVector3x32.sampleCount - 1)), 0, FixedArrayVector3x32.sampleCount - 2);
        var offseted = t - ((float)index / (float)(FixedArrayVector3x32.sampleCount - 1));
        var lerpT = offseted * (float)(FixedArrayVector3x32.sampleCount - 1);
        return Vector3.Lerp(binormalLUT[index], binormalLUT[index + 1], lerpT);
    }

    public Matrix4x4 GetBasisFromT(ref Spline3D spline, float t, Vector3 startingUp) {
        Vector3 tangent = spline.GetVelocityFromT(t).normalized;
        Vector3 binormal = GetBinormalFromT(t, startingUp).normalized;
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

    public Matrix4x4 GetReferenceFrameFromT(ref Spline3D spline, float t, Vector3 startingUp) {
        return Matrix4x4.Translate(spline.GetPositionFromT(t)) * GetBasisFromT(ref spline, t, startingUp);
    }

}

[Serializable]
public struct Spline3D {
    public struct BezierKnot {
        public Vector3 backHandle;
        public Vector3 position;
        public Vector3 frontHandle;
    }
    
    public struct HermiteKnot {
        public Vector3 position;
        public Vector3 velocity;
    }

    [SerializeField] private FixedListMatrix16 weights;
    [SerializeField] private FixedArrayFloat32 distanceLUT;
    private float _arcLength;

    public float arcLength {
        get {
            GenerateDistanceLUT();
            return _arcLength;
        }
        private set => _arcLength = value;
    }

    private Spline3D(FixedListMatrix16 weights) : this() {
        SetWeights(weights);
    }

    private static Spline3D Lerp(Spline3D a, Spline3D b, float t) {
        return new Spline3D(FixedListMatrix16.Lerp(a.weights, b.weights, t));
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

    public FixedListMatrix16 GetWeights() => weights;

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
        curveSegmentIndex = Mathf.Clamp(Mathf.FloorToInt(t * (weights.Length)), 0, (weights.Length) - 1);
        var offseted = t - ((float)curveSegmentIndex / (float)(weights.Length));
        return offseted * (float)(weights.Length);
    }

    public float GetTimeFromDistance(float distance) {
        GenerateDistanceLUT();
        if (distance > 0f && distance < arcLength) {
            for (int i = 0; i < FixedArrayFloat32.sampleCount - 1; i++) {
                if (distance > distanceLUT[i] && distance < distanceLUT[i + 1]) {
                    return Remap(distance, distanceLUT[i], distanceLUT[i + 1],
                        (float)i / (float)(FixedArrayFloat32.sampleCount - 1), (float)(i + 1) / (float)(FixedArrayFloat32.sampleCount - 1));
                }
            }
        }

        return distance / arcLength;
    }

    public float GetDistanceFromTime(float t) {
        GenerateDistanceLUT();
        t = Mathf.Clamp01(t);
        int index = Mathf.Clamp(Mathf.FloorToInt(t * (FixedArrayFloat32.sampleCount - 1)), 0, FixedArrayFloat32.sampleCount - 2);
        float offseted = t - ((float)index / (float)(FixedArrayFloat32.sampleCount - 1));
        float lerpT = offseted * (float)(FixedArrayFloat32.sampleCount - 1);
        return Mathf.Lerp(distanceLUT[index], distanceLUT[index + 1], lerpT);
    }

    private bool CheckMinimaMaxima(float tValue) {
        return !float.IsNaN(tValue) && Mathf.Clamp01(tValue) == tValue;
    }

    // Again we use Freya Holmer's tutorial to understand the strategy behind generating the bounds: https://youtu.be/aVwxzDHniEw?t=791
    // This is just the derivative written in terms of t, then plugged into the quadratic formula to solve for zeros.
    // that gives us our local extremes to generate our bounds on.
    public void GenerateBounds(List<Bounds> bounds) {
        bounds.Clear();
        for (int i=0;i<weights.Length;i++) {
            var weightGroup = weights[i];
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
        for (int i = 1; i < FixedArrayFloat32.sampleCount; i++) {
            float t = (float)i / (FixedArrayFloat32.sampleCount - 1);
            Vector3 position = GetPositionFromT(t);
            dist += Vector3.Distance(lastPosition, position);
            lastPosition = position;
            distanceLUT[i] = dist;
        }
        arcLength = dist;
    }

    
    private Spline3D SetWeights(FixedListMatrix16 newWeights) {
        weights = newWeights;
        GenerateDistanceLUT();
        return this;
    }

    private static float Remap(float value, float from1, float to1, float from2, float to2) {
        return (value - from1) / (to1 - from1) * (to2 - from2) + from2;
    }

    private static Matrix4x4 CreateWeightMatrix(Matrix4x4 characteristicMatrix, Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3) {
        var pointMatrix = new Matrix4x4(p0, p1, p2, p3);
        return pointMatrix * characteristicMatrix;
    }
    
    private static void GetWeightsWithEndHandles(Matrix4x4 characteristicMatrix, ref FixedListMatrix16 weightCollection, IList<Vector3> newPoints, Vector3 startHandle, Vector3 endHandle) {
        var pointCount = newPoints.Count;
        if (pointCount < 2) {
            throw new UnityException("Spline must have at least 2 knots.");
        }

        weightCollection.Clear();
        for (int i = 0; i < pointCount-1; i++) {
            var previousPoint = i == 0 ? startHandle : newPoints[i - 1];
            var nextPoint = i == pointCount - 2 ? endHandle : newPoints[i + 2];
            var weightMatrix = CreateWeightMatrix(
                characteristicMatrix, 
                previousPoint, 
                newPoints[i], 
                newPoints[i + 1], 
                nextPoint
            );
            weightCollection.Add(weightMatrix);
        }
    }

    private static void GetWeightsLooped(Matrix4x4 characteristicMatrix, ref FixedListMatrix16 weightCollection, IList<Vector3> newPoints) {
        var pointCount = newPoints.Count;
        weightCollection.Clear();
        for (int i = 0; i < pointCount; i++) {
            var weightMatrix = CreateWeightMatrix(
                characteristicMatrix,
                newPoints[(i+pointCount-1)%pointCount],
                newPoints[i],
                newPoints[(i + 1)%pointCount],
                newPoints[(i + 2)%pointCount]
            );
            weightCollection.Add(weightMatrix);
        }
    }

    private static void GetWeightsBezierKnots(Matrix4x4 characteristicMatrix, ref FixedListMatrix16 weightCollection, IList<BezierKnot> knots) {
        var knotCount = knots.Count;
        if (knotCount < 2) {
            throw new UnityException("Spline must have at least 2 knots.");
        }
        weightCollection.Clear();
        for (int i=0;i<knotCount-1;i++) {
            var knot = knots[i];
            var nextKnot = knots[i+1];
            var weightMatrix = CreateWeightMatrix(
                characteristicMatrix,
                knot.position,
                knot.frontHandle,
                nextKnot.backHandle,
                nextKnot.position
            );
            weightCollection.Add(weightMatrix);
        }
    }
    private static void GetWeightsHermiteKnots(Matrix4x4 characteristicMatrix, ref FixedListMatrix16 weightCollection, IList<HermiteKnot> knots) {
        var knotCount = knots.Count;
        if (knotCount < 2) {
            throw new UnityException("Spline must have at least 2 knots.");
        }
        weightCollection.Clear();
        for (int i=0;i<knotCount-1;i++) {
            var knot = knots[i];
            var nextKnot = knots[i+1];
            var weightMatrix = CreateWeightMatrix(
                characteristicMatrix,
                knot.position,
                knot.velocity,
                nextKnot.velocity,
                nextKnot.position
            );
            weightCollection.Add(weightMatrix);
        }
    }

    public float GetLengthFromSubsection(int subsectionCount, int subsectionIndex = 0, float subT = 1f) {
        var multi = (float)subsectionCount / (float)weights.Length;
        var startT = (float)subsectionIndex / (float)weights.Length;
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
    public void SetFromBezierKnots(IList<BezierKnot> knots) {
        GetWeightsBezierKnots(new Matrix4x4(
            new Vector4(1f, 0f, 0f, 0f),
            new Vector4(-3f, 3f, 0f, 0f),
            new Vector4(3f, -6f, 3f, 0f),
            new Vector4(-1f, 3f, -3f, 1f)
        ), ref weights, knots);
        GenerateDistanceLUT();
    }
    
    /// <summary>
    /// Set the spline from a list of SplineKnots using Hermite interpolation.
    /// The handles of each knot are velocity vectors from zero
    /// The knots are world positions.
    /// </summary>
    /// <param name="knots">Start Position, Start Velocity, End Velocity, End Position</param>
    public void SetFromHermiteKnots(IList<HermiteKnot> knots) {
        GetWeightsHermiteKnots(new Matrix4x4(
            new Vector4(1f, 0f, 0f, 0f),
            new Vector4(0f, 1f, 0f, 0f),
            new Vector4(-3f, -2f, -1f, 3f),
            new Vector4(2f, 1f, 1f, -2)
        ), ref weights, knots);
        GenerateDistanceLUT();
    }
    
    /// <summary>
    /// A proper C2 continuous spline, positions are never touched, but damn is the curve smooth...
    /// </summary>
    /// <param name="positions">the anchor positions</param>
    private void SetFromBSplinePositionsLooped(IList<Vector3> positions) {
        GetWeightsLooped(new Matrix4x4(
            new Vector4(1f, 4f, 1f, 0f) * 1f/6f,
            new Vector4(-3f, 0f, 3f, 0f) * 1f/6f,
            new Vector4(3f, -6f, 3f, 0f) * 1f/6f,
            new Vector4(-1f, 3f, -3f, 1f) * 1f/6f
        ), ref weights, positions);
        GenerateDistanceLUT();
    }
    
    /// <summary>
    /// A proper C2 continuous spline, positions are never touched, but damn is the curve smooth...
    /// </summary>
    /// <param name="positions">the anchor positions</param>
    /// <param name="startHandle">handle for start position</param>
    /// <param name="endHandle">handle for end position</param>
    public void SetFromBSplinePositions(IList<Vector3> positions, Vector3 startHandle, Vector3 endHandle) {
        GetWeightsWithEndHandles(new Matrix4x4(
            new Vector4(1f, 4f, 1f, 0f) * 1f/6f,
            new Vector4(-3f, 0f, 3f, 0f) * 1f/6f,
            new Vector4(3f, -6f, 3f, 0f) * 1f/6f,
            new Vector4(-1f, 3f, -3f, 1f) * 1f/6f
        ), ref weights, positions, startHandle, endHandle);
        GenerateDistanceLUT();
    }

    /// <summary>
    /// A proper C2 continuous spline, positions are never touched, but damn is the curve smooth... With automatic start and end handles
    /// </summary>
    /// <param name="positions">the anchor positions</param>
    /// <param name="startHandle">handle for start position</param>
    /// <param name="endHandle">handle for end position</param>
    public void SetFromBSplinePositions(IList<Vector3> positions, bool looped = false) {
        if (looped) {
            SetFromBSplinePositionsLooped(positions);
            return;
        }
        var startHandle = positions[0] + (positions[0] - positions[1]);
        var endHandle = positions[^1] + (positions[^1] - positions[^2]);
        SetFromBSplinePositions(positions, startHandle, endHandle);
    }

    /// <summary>
    /// A Catmull rom spline, but with the ability to set the tension. Tension of 0 is standard Catmull-Rom, tension of 1 is linear interpolation.
    /// </summary>
    /// <param name="positions">the anchor positions</param>
    /// <param name="tension">the tension of the spline</param>
    private void SetFromCardinalPositionsLooped(IList<Vector3> positions, float tension) {
        float s = (1f - tension) / 2f;
        GetWeightsLooped(new Matrix4x4(
            new Vector4(0f, 1f, 0f, 0f),
            new Vector4(-s, 0f, s, 0f),
            new Vector4(2f*s, s-3f, 3f-2f*s, -s),
            new Vector4(-s, 2f-s, s-2f, s)
        ), ref weights, positions);
        GenerateDistanceLUT();
    }
    
    /// <summary>
    /// A Catmull rom spline, but with the ability to set the tension. Tension of 0 is standard Catmull-Rom, tension of 1 is linear interpolation.
    /// </summary>
    /// <param name="positions">the anchor positions</param>
    /// <param name="startHandle">handle for start position</param>
    /// <param name="endHandle">handle for end position</param>
    /// <param name="tension">the tension of the spline</param>
    public void SetFromCardinalPositions(IList<Vector3> positions, Vector3 startHandle, Vector3 endHandle, float tension) {
        float s = (1f - tension) / 2f;
        GetWeightsWithEndHandles(new Matrix4x4(
            new Vector4(0f, 1f, 0f, 0f),
            new Vector4(-s, 0f, s, 0f),
            new Vector4(2f*s, s-3f, 3f-2f*s, -s),
            new Vector4(-s, 2f-s, s-2f, s)
        ), ref weights, positions, startHandle, endHandle);
        GenerateDistanceLUT();
    }
    
    /// <summary>
    /// A Catmull rom spline, but with the ability to set the tension. Tension of 0 is standard Catmull-Rom, tension of 1 is linear interpolation.
    /// </summary>
    /// <param name="positions">the anchor positions</param>
    /// <param name="tension">the tension of the spline</param>
    /// <param name="looped">whether the spline is looped</param>
    public void SetFromCardinalPositions(IList<Vector3> positions, float tension, bool looped = false) {
        if (looped) {
            SetFromCardinalPositionsLooped(positions, tension);
            return;
        }
        var startHandle = positions[0] + (positions[0] - positions[1]);
        var endHandle = positions[^1] + (positions[^1] - positions[^2]);
        SetFromCardinalPositions(positions, startHandle, endHandle, tension);
    }
    
    /// <summary>
    /// A proper C1 continuous spline, positions are always touched, velocity is smooth.
    /// </summary>
    /// <param name="positions">the anchor positions</param>
    private void SetFromCatmullPositionsLooped(IList<Vector3> positions) {
        GetWeightsLooped(new Matrix4x4(
            new Vector4(0f, 2f, 0f, 0f)*0.5f,
            new Vector4(-1f, 0f, 1f, 0f)*0.5f,
            new Vector4(2f, -5f, 4f, -1f)*0.5f,
            new Vector4(-1f, 3f, -3f, 1f)*0.5f
        ), ref weights, positions);
        GenerateDistanceLUT();
    }

    /// <summary>
    /// A proper C1 continuous spline, positions are always touched, velocity is smooth.
    /// </summary>
    /// <param name="positions">the anchor positions</param>
    /// <param name="startHandle">handle for start position</param>
    /// <param name="endHandle">handle for end position</param>
    public void SetFromCatmullPositions(IList<Vector3> positions, Vector3 startHandle, Vector3 endHandle) {
        GetWeightsWithEndHandles(new Matrix4x4(
            new Vector4(0f, 2f, 0f, 0f)*0.5f,
            new Vector4(-1f, 0f, 1f, 0f)*0.5f,
            new Vector4(2f, -5f, 4f, -1f)*0.5f,
            new Vector4(-1f, 3f, -3f, 1f)*0.5f
        ), ref weights, positions, startHandle, endHandle);
        GenerateDistanceLUT();
    }

    /// <summary>
    /// A proper C1 continuous spline, positions are always touched, velocity is smooth. With automatic start and end handles
    /// </summary>
    /// <param name="positions">the anchor positions</param>
    /// <param name="looped">whether the spline should be looped</param>
    public void SetFromCatmullPositions(IList<Vector3> positions, bool looped = false) {
        if (looped) {
            SetFromCatmullPositionsLooped(positions);
            return;
        }
        var startHandle = positions[0] + (positions[0] - positions[1]);
        var endHandle = positions[^1] + (positions[^1] - positions[^2]);
        SetFromCatmullPositions(positions, startHandle, endHandle);
    }

    public float GetClosestTimeFromPositionFast(List<Bounds> bounds, Vector3 position, int samples = 32) {
        // Broad pass that just finds the closest bounds
        int closestBounds = 0;
        float distToBounds = float.MaxValue;
        GenerateBounds(bounds);
        var boundsCount = bounds.Count;
        for (int i = 0; i < boundsCount; i++) {
            float dist = Vector3.Distance(bounds[i].ClosestPoint(position), position);
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
        return closestTValue / (float)(boundsCount) + ((float)closestBounds / (float)(boundsCount));
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
        switch (t) {
            case < 0f: {
                var arclength = arcLength;
                var vel = SampleCurveSegmentVelocity(0, 0f);
                var pos = SampleCurveSegmentPosition(0, 0f);
                return pos + vel.normalized * (t * arclength);
            }
            case > 1f: {
                var arclength = arcLength;
                var endSegment = weights.Length - 1;
                var vel = SampleCurveSegmentVelocity(endSegment, 1f);
                var pos = SampleCurveSegmentPosition(endSegment, 1f);
                return pos + vel.normalized * ((t-1f) * arclength);
            }
            default: {
                var subT = GetCurveSegmentTimeFromCurveTime(out var curveSegmentIndex, t);
                return SampleCurveSegmentPosition(curveSegmentIndex, subT);
            }
        }
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
        switch (t) {
            case < 0f: {
                var vel = SampleCurveSegmentVelocity(0, 0f);
                return vel.normalized;
            }
            case > 1f: {
                var vel = SampleCurveSegmentVelocity(weights.Length - 1, 1f);
                return vel.normalized;
            }
            default: {
                var subT = GetCurveSegmentTimeFromCurveTime(out var curveSegmentIndex, t);
                return SampleCurveSegmentVelocity(curveSegmentIndex, subT);
            }
        }
    }

    public Vector3 GetAccelerationFromT(float t) {
        switch (t) {
            case < 0f:
            case > 1f:
                return Vector3.zero;
            default: {
                var subT = GetCurveSegmentTimeFromCurveTime(out var curveSegmentIndex, t);
                return SampleCurveSegmentAcceleration(curveSegmentIndex, subT);
            }
        }
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

    public static void DebugDrawSpline(Spline3D spline3D, Color splineColor, Color knotColor, float duration = 0f) {
        Vector3 current = spline3D.GetPositionFromT(0f);
        for (int i = 1; i < 64; i++) {
            Vector3 next = spline3D.GetPositionFromT((float)i / 63);
            Debug.DrawLine(current, next, splineColor, duration);
            current = next;
        }

        var weights = spline3D.GetWeights();
        var weightCount = weights.Length;
        for (int i = 0; i < weightCount; i++) {
            var weight = weights[i];
            var startPos = GetPosition(weight, 0f);
            var startVel = GetVelocity(weight, 0f).normalized;
            if (startVel == Vector3.zero) {
                startVel = Vector3.forward;
            }
            DrawBox(startPos, Quaternion.LookRotation(startVel), (Vector3.one - Vector3.forward * 0.8f) * knotSize, knotColor, duration);
        }

        var endPos = GetPosition(weights[weightCount - 1], 1f);
        var endVel = GetVelocity(weights[weightCount - 1], 1f).normalized;
        if (endVel == Vector3.zero) {
            endVel = Vector3.forward;
        }
        DrawBox(endPos, Quaternion.LookRotation(endVel), (Vector3.one - Vector3.forward * 0.8f) * knotSize, knotColor, duration);
    }

    public static void GizmosDrawSpline(Spline3D spline3D, Color splineColor, Color knotColor, int resolution = 64) {
        var oldColor = Gizmos.color;
        Gizmos.color = splineColor;
        Vector3 lastPoint = spline3D.GetPositionFromT(0f);
        for (int i = 0; i <= resolution; i++) {
            Vector3 newPoint = spline3D.GetPositionFromT((float)i / resolution);
            Gizmos.DrawLine(lastPoint, newPoint);
            lastPoint = newPoint;
        }

        var save = Gizmos.matrix;
        var weights = spline3D.GetWeights();
        var weightCount = weights.Length;
        for (int i = 0; i < weightCount; i++) {
            var weight = weights[i];
            float t = (float)i / (weightCount - 1);
            Vector3 pointA = GetPosition(weight, 0f);
            var startVel = GetVelocity(weight, 0f).normalized;
            if (startVel == Vector3.zero) {
                startVel = Vector3.forward;
            }
            Gizmos.color = knotColor;
            Gizmos.matrix = Matrix4x4.TRS(pointA, Quaternion.LookRotation(startVel), Vector3.one - Vector3.forward * 0.8f);
            Gizmos.DrawWireCube(Vector3.zero, Vector3.one * knotSize);
        }

        var endPos = GetPosition(weights[weightCount - 1], 1f);
        var endVel = GetVelocity(weights[weightCount - 1], 1f).normalized;
        if (endVel == Vector3.zero) {
            endVel = Vector3.forward;
        }
        Gizmos.matrix = Matrix4x4.TRS(endPos, Quaternion.LookRotation(endVel), Vector3.one - Vector3.forward * 0.8f);
        Gizmos.DrawWireCube(Vector3.zero, Vector3.one * knotSize);

        Gizmos.matrix = save;
        Gizmos.color = oldColor;
    }

    public Spline3DSpace GetSplineSpace(Vector3 up) {
        return new Spline3DSpace(ref this, up);
    }
}

}