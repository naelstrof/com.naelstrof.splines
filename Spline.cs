using System.Collections.Generic;
using UnityEngine;

namespace GatorDragonGames {

public struct Spline {
    public struct BezierKnot {
        public float backHandle;
        public float position;
        public float frontHandle;
    }
    
    public struct HermiteKnot {
        public float position;
        public float velocity;
    }

    private FixedListVector4x16 weights;
    private FixedArrayFloat32 distanceLUT;
    private float arcLength;

    private void GenerateDistanceLUT() {
        float dist = 0f;
        float lastPosition = GetPositionFromT(0f);
        distanceLUT[0] = 0f;
        for (int i = 1; i < FixedArrayFloat32.sampleCount; i++) {
            float t = (float)i / (FixedArrayFloat32.sampleCount - 1);
            float position = GetPositionFromT(t);
            dist += Mathf.Abs(lastPosition - position);
            lastPosition = position;
            distanceLUT[i] = dist;
        }
        arcLength = dist;
    }
    
    public float GetTimeFromDistance(float distance) {
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

    private Spline(FixedListVector4x16 weights) : this() {
        SetWeights(weights);
    }

    private static Spline Lerp(Spline a, Spline b, float t) {
        return new Spline(FixedListVector4x16.Lerp(a.weights, b.weights, t));
    }

    private static float GetPosition(Vector4 weightBlock, float t) {
        return Vector4.Dot(weightBlock, new Vector4(1f, t, t * t, t * t * t));
    }

    private static float GetVelocity(Vector4 weightBlock, float t) {
        return Vector4.Dot(weightBlock, new Vector4(0f, 1f, 2f * t, 3f * t * t));
    }

    private static float GetAcceleration(Vector4 weightBlock, float t) {
        return Vector4.Dot(weightBlock, new Vector4(0f, 0f, 2f, 6f * t));
    }

    private FixedListVector4x16 GetWeights() => weights;

    private float SampleCurveSegmentPosition(int curveSegmentIndex, float t) {
        return GetPosition(weights[curveSegmentIndex], t);
    }

    private float SampleCurveSegmentVelocity(int curveSegmentIndex, float t) {
        return GetVelocity(weights[curveSegmentIndex], t);
    }

    private float SampleCurveSegmentAcceleration(int curveSegmentIndex, float t) {
        return GetAcceleration(weights[curveSegmentIndex], t);
    }

    private float GetCurveSegmentTimeFromCurveTime(out int curveSegmentIndex, float t) {
        var weightCount = weights.Length;
        curveSegmentIndex = Mathf.Clamp(Mathf.FloorToInt(t * (weightCount)), 0, (weightCount) - 1);
        var offseted = t - ((float)curveSegmentIndex / (float)(weightCount));
        return offseted * (float)(weightCount);
    }

    private bool CheckMinimaMaxima(float tValue) {
        return !float.IsNaN(tValue) && Mathf.Clamp01(tValue) == tValue;
    }

    private void SetWeights(FixedListVector4x16 newWeights) {
        weights = newWeights;
        GenerateDistanceLUT();
    }

    private static float Remap(float value, float from1, float to1, float from2, float to2) {
        return (value - from1) / (to1 - from1) * (to2 - from2) + from2;
    }

    private static Vector4 CreateWeightMatrix(Matrix4x4 characteristicMatrix, float p0, float p1, float p2, float p3) {
        var pointMatrix = new Vector4(p0, p1, p2, p3);
        return characteristicMatrix.transpose * pointMatrix;
    }
    
    private static void GetWeightsWithEndHandles(Matrix4x4 characteristicMatrix, ref FixedListVector4x16 weightCollection, IList<float> newPoints, float startHandle, float endHandle) {
        weightCollection.Clear();
        var pointCount = newPoints.Count;
        if (pointCount < 2) {
            throw new UnityException("Spline must have at least 2 knots.");
        }
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

    private static void GetWeightsLooped(Matrix4x4 characteristicMatrix, ref FixedListVector4x16 weightCollection, IList<float> newPoints) {
        weightCollection.Clear();
        var pointCount = newPoints.Count;
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

    private static void GetWeightsBezierKnots(Matrix4x4 characteristicMatrix, ref FixedListVector4x16 weightCollection, IList<BezierKnot> knots) {
        weightCollection.Clear();
        var knotCount = knots.Count;
        if (knotCount < 2) {
            throw new UnityException("Spline must have at least 2 knots.");
        }
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
    private static void GetWeightsHermiteKnots(Matrix4x4 characteristicMatrix, ref FixedListVector4x16 weightCollection, IList<HermiteKnot> knots) {
        weightCollection.Clear();
        var knotCount = knots.Count;
        if (knotCount < 2) {
            throw new UnityException("Spline must have at least 2 knots.");
        }
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
    private void SetFromBSplinePositionsLooped(IList<float> positions) {
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
    public void SetFromBSplinePositions(IList<float> positions, float startHandle, float endHandle) {
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
    public void SetFromBSplinePositions(IList<float> positions, bool looped = false) {
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
    private void SetFromCardinalPositionsLooped(IList<float> positions, float tension) {
        float k = 1f - tension;
        GetWeightsLooped(new Matrix4x4(
            new Vector4(0f, 1f, 0f, 0f),
            new Vector4(-0.5f * k, -tension, 0.5f * k + tension, 0f),
            new Vector4(k, -2.5f * k, 2f * k, -0.5f * k),
            new Vector4(-0.5f * k, 1.5f * k, -1.5f * k, 0.5f * k)
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
    public void SetFromCardinalPositions(IList<float> positions, float startHandle, float endHandle, float tension) {
        float k = 1f - tension;
        GetWeightsWithEndHandles(new Matrix4x4(
                new Vector4(0f, 1f, 0f, 0f),
                new Vector4(-0.5f * k, -tension, 0.5f * k + tension, 0f),
                new Vector4(k, -2.5f * k, 2f * k, -0.5f * k),
                new Vector4(-0.5f * k, 1.5f * k, -1.5f * k, 0.5f * k)
            ), ref weights, positions, startHandle, endHandle);
        GenerateDistanceLUT();
    }
    
    private void SetFromLinearLooped(IList<float> positions) {
        GetWeightsLooped(new Matrix4x4(
            new Vector4(0f, 1f, 0f, 0f),
            new Vector4(0f, -1f, 1f, 0f),
            Vector4.zero,
            Vector4.zero 
        ), ref weights, positions);
        GenerateDistanceLUT();
    }
    
    public void SetFromLinearPositions(IList<float> positions, bool looped = false) {
        if (looped) {
            SetFromLinearLooped(positions);
            return;
        }
        weights.Clear();
        GetWeightsWithEndHandles(new Matrix4x4(
            new Vector4(0f, 1f, 0f, 0f),
            new Vector4(0f, -1f, 1f, 0f),
            Vector4.zero,
            Vector4.zero 
        ), ref weights, positions, 0, 0);
        GenerateDistanceLUT();
    }
    
    /// <summary>
    /// A Catmull rom spline, but with the ability to set the tension. Tension of 0 is standard Catmull-Rom, tension of 1 is linear interpolation.
    /// </summary>
    /// <param name="positions">the anchor positions</param>
    /// <param name="tension">the tension of the spline</param>
    /// <param name="looped">whether the spline is looped</param>
    public void SetFromCardinalPositions(IList<float> positions, float tension, bool looped = false) {
        if (looped) {
            SetFromCardinalPositionsLooped(positions, tension);
            return;
        }
        var startHandle = positions[0] + (positions[0] - positions[1]);
        var endHandle = positions[^1] + (positions[^1] - positions[^2]);
        SetFromCardinalPositions(positions, startHandle, endHandle, tension);
        return;
    }
    
    /// <summary>
    /// A proper C1 continuous spline, positions are always touched, velocity is smooth.
    /// </summary>
    /// <param name="positions">the anchor positions</param>
    private void SetFromCatmullPositionsLooped(IList<float> positions) {
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
    public void SetFromCatmullPositions(IList<float> positions, float startHandle, float endHandle) {
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
    public void SetFromCatmullPositions(IList<float> positions, bool looped = false) {
        if (looped) {
            SetFromCatmullPositionsLooped(positions);
            return;
        }
        var startHandle = positions[0] + (positions[0] - positions[1]);
        var endHandle = positions[^1] + (positions[^1] - positions[^2]);
        SetFromCatmullPositions(positions, startHandle, endHandle);
    }

    public float GetClosestTimeFromPosition(float position, int samples = 32) {
        var closestTValue = 0f;
        var distToCurve = float.MaxValue;
        for (int i = 0; i < samples; i++) {
            var tSample = (float)i / (float)samples;
            float samplePosition = GetPositionFromT(tSample);
            var dist = Mathf.Abs(samplePosition-position);
            if (dist < distToCurve) {
                closestTValue = tSample;
                distToCurve = dist;
            }
        }

        return closestTValue;
    }

    public float GetPositionFromT(float t) {
        switch (t) {
            case < 0f: {
                var vel = SampleCurveSegmentVelocity(0, 0f);
                var pos = SampleCurveSegmentPosition(0, 0f);
                return pos + Mathf.Sign(vel) * t;
            }
            case > 1f: {
                var endSegment = weights.Length - 1;
                var vel = SampleCurveSegmentVelocity(endSegment, 1f);
                var pos = SampleCurveSegmentPosition(endSegment, 1f);
                return pos + Mathf.Sign(vel) * (t-1f);
            }
            default: {
                var subT = GetCurveSegmentTimeFromCurveTime(out var curveSegmentIndex, t);
                return SampleCurveSegmentPosition(curveSegmentIndex, subT);
            }
        }
    }

    public float GetVelocityFromT(float t) {
        switch (t) {
            case < 0f: {
                var vel = SampleCurveSegmentVelocity(0, 0f);
                return vel;
            }
            case > 1f: {
                var vel = SampleCurveSegmentVelocity(weights.Length - 1, 1f);
                return vel;
            }
            default: {
                var subT = GetCurveSegmentTimeFromCurveTime(out var curveSegmentIndex, t);
                return SampleCurveSegmentVelocity(curveSegmentIndex, subT);
            }
        }
    }

    public float GetAccelerationFromT(float t) {
        switch (t) {
            case < 0f:
            case > 1f:
                return 0f;
            default: {
                var subT = GetCurveSegmentTimeFromCurveTime(out var curveSegmentIndex, t);
                return SampleCurveSegmentAcceleration(curveSegmentIndex, subT);
            }
        }
    }
    public void DrawGizmos(Color lineColor, int samples = 64) {
        Gizmos.color = Color.blue;
        Gizmos.DrawLine(new Vector3(0, 0, 0f), new Vector3(1f, 0f, 0f));
        Gizmos.DrawLine(new Vector3(0, 0, 0f), new Vector3(0f, 1f, 0f));
        Gizmos.DrawLine(new Vector3(1f, 0, 0f), new Vector3(1f, 1f, 0f));
        Gizmos.DrawLine(new Vector3(0f, 1f, 0f), new Vector3(1f, 1f, 0f));
        Gizmos.color = lineColor;
        var previousPoint = GetPositionFromT(0f);
        var previousT = 0f;
        for (int i = 1; i <= samples; i++) {
            var tSample = (float)i / (float)samples;
            var currentPoint = GetPositionFromT(tSample);
            Gizmos.DrawLine(new Vector3(previousT, previousPoint, 0f), new Vector3(tSample, currentPoint, 0f));
            previousPoint = currentPoint;
            previousT = tSample;
        }
    }
}

}