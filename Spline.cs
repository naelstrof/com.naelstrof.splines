using System;
using System.Collections.Generic;
using UnityEngine;

namespace GatorDragonGames {

public class Spline {
    public struct BezierKnot {
        public float backHandle;
        public float position;
        public float frontHandle;
    }
    
    public struct HermiteKnot {
        public float position;
        public float velocity;
    }

    private List<Vector4> weights = new();

    public Spline() {
        SetFromCatmullPositions(new [] { 0f, 1f });
    }

    private Spline(IList<Vector4> weights) {
        SetWeights(weights);
    }

    private static Spline Lerp(Spline a, Spline b, float t) {
        List<Vector4> merged = new List<Vector4>(a.weights);
        for (int i = 0; i < merged.Count; i++) {
            merged[i] = Vector4.Lerp(merged[i], b.weights[i], t);
        }
        return new Spline(merged);
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

    public List<Vector4> GetWeights() => weights;

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
        curveSegmentIndex = Mathf.Clamp(Mathf.FloorToInt(t * (weights.Count)), 0, (weights.Count) - 1);
        var offseted = t - ((float)curveSegmentIndex / (float)(weights.Count));
        return offseted * (float)(weights.Count);
    }

    private bool CheckMinimaMaxima(float tValue) {
        return !float.IsNaN(tValue) && Mathf.Clamp01(tValue) == tValue;
    }


    private Spline SetWeights(IList<Vector4> newWeights) {
        weights.Clear();
        weights.AddRange(newWeights);
        return this;
    }

    private static float Remap(float value, float from1, float to1, float from2, float to2) {
        return (value - from1) / (to1 - from1) * (to2 - from2) + from2;
    }

    private static Vector4 CreateWeightMatrix(Matrix4x4 characteristicMatrix, float p0, float p1, float p2, float p3) {
        var pointMatrix = new Vector4(p0, p1, p2, p3);
        return characteristicMatrix.transpose * pointMatrix;
    }
    
    private static void GetWeightsWithEndHandles(Matrix4x4 characteristicMatrix, ICollection<Vector4> weightCollection, IList<float> newPoints, float startHandle, float endHandle) {
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

    private static void GetWeightsLooped(Matrix4x4 characteristicMatrix, ICollection<Vector4> weightCollection, IList<float> newPoints) {
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

    private static void GetWeightsBezierKnots(Matrix4x4 characteristicMatrix, ICollection<Vector4> weightCollection, IList<BezierKnot> knots) {
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
    private static void GetWeightsHermiteKnots(Matrix4x4 characteristicMatrix, ICollection<Vector4> weightCollection, IList<HermiteKnot> knots) {
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
    /// <returns>The generated Spline</returns>
    public Spline SetFromBezierKnots(IList<BezierKnot> knots) {
        weights.Clear();
        GetWeightsBezierKnots(new Matrix4x4(
            new Vector4(1f, 0f, 0f, 0f),
            new Vector4(-3f, 3f, 0f, 0f),
            new Vector4(3f, -6f, 3f, 0f),
            new Vector4(-1f, 3f, -3f, 1f)
        ), weights, knots);
        return this;
    }
    
    /// <summary>
    /// Set the spline from a list of SplineKnots using Hermite interpolation.
    /// The handles of each knot are velocity vectors from zero
    /// The knots are world positions.
    /// </summary>
    /// <param name="knots">Start Position, Start Velocity, End Velocity, End Position</param>
    /// <returns>The generated Spline</returns>
    public Spline SetFromHermiteKnots(IList<HermiteKnot> knots) {
        weights.Clear();
        GetWeightsHermiteKnots(new Matrix4x4(
            new Vector4(1f, 0f, 0f, 0f),
            new Vector4(0f, 1f, 0f, 0f),
            new Vector4(-3f, -2f, -1f, 3f),
            new Vector4(2f, 1f, 1f, -2)
        ), weights, knots);
        return this;
    }
    
    /// <summary>
    /// A proper C2 continuous spline, positions are never touched, but damn is the curve smooth...
    /// </summary>
    /// <param name="positions">the anchor positions</param>
    /// <returns>The generated Spline</returns>
    private Spline SetFromBSplinePositionsLooped(IList<float> positions) {
        weights.Clear();
        GetWeightsLooped(new Matrix4x4(
            new Vector4(1f, 4f, 1f, 0f) * 1f/6f,
            new Vector4(-3f, 0f, 3f, 0f) * 1f/6f,
            new Vector4(3f, -6f, 3f, 0f) * 1f/6f,
            new Vector4(-1f, 3f, -3f, 1f) * 1f/6f
        ), weights, positions);
        return this;
    }
    
    /// <summary>
    /// A proper C2 continuous spline, positions are never touched, but damn is the curve smooth...
    /// </summary>
    /// <param name="positions">the anchor positions</param>
    /// <param name="startHandle">handle for start position</param>
    /// <param name="endHandle">handle for end position</param>
    /// <returns>The generated Spline</returns>
    public Spline SetFromBSplinePositions(IList<float> positions, float startHandle, float endHandle) {
        weights.Clear();
        GetWeightsWithEndHandles(new Matrix4x4(
            new Vector4(1f, 4f, 1f, 0f) * 1f/6f,
            new Vector4(-3f, 0f, 3f, 0f) * 1f/6f,
            new Vector4(3f, -6f, 3f, 0f) * 1f/6f,
            new Vector4(-1f, 3f, -3f, 1f) * 1f/6f
        ), weights, positions, startHandle, endHandle);
        return this;
    }

    /// <summary>
    /// A proper C2 continuous spline, positions are never touched, but damn is the curve smooth... With automatic start and end handles
    /// </summary>
    /// <param name="positions">the anchor positions</param>
    /// <param name="startHandle">handle for start position</param>
    /// <param name="endHandle">handle for end position</param>
    /// <returns>The generated Spline</returns>
    public Spline SetFromBSplinePositions(IList<float> positions, bool looped = false) {
        if (looped) {
            return SetFromBSplinePositionsLooped(positions);
        }
        var startHandle = positions[0] + (positions[0] - positions[1]);
        var endHandle = positions[^1] + (positions[^1] - positions[^2]);
        return SetFromBSplinePositions(positions, startHandle, endHandle);
    }

    /// <summary>
    /// A Catmull rom spline, but with the ability to set the tension. Tension of 0 is standard Catmull-Rom, tension of 1 is linear interpolation.
    /// </summary>
    /// <param name="positions">the anchor positions</param>
    /// <param name="tension">the tension of the spline</param>
    /// <returns>The generated Spline</returns>
    private Spline SetFromCardinalPositionsLooped(IList<float> positions, float tension) {
        weights.Clear();
        float s = (1f - tension) / 2f;
        GetWeightsLooped(new Matrix4x4(
            new Vector4(0f, 1f, 0f, 0f),
            new Vector4(-s, 0f, s, 0f),
            new Vector4(2f*s, s-3f, 3f-2f*s, -s),
            new Vector4(-s, 2f-s, s-2f, s)
        ), weights, positions);
        return this;
    }
    
    /// <summary>
    /// A Catmull rom spline, but with the ability to set the tension. Tension of 0 is standard Catmull-Rom, tension of 1 is linear interpolation.
    /// </summary>
    /// <param name="positions">the anchor positions</param>
    /// <param name="startHandle">handle for start position</param>
    /// <param name="endHandle">handle for end position</param>
    /// <param name="tension">the tension of the spline</param>
    /// <returns>The generated Spline</returns>
    public Spline SetFromCardinalPositions(IList<float> positions, float startHandle, float endHandle, float tension) {
        weights.Clear();
        float s = (1f - tension) / 2f;
        GetWeightsWithEndHandles(new Matrix4x4(
            new Vector4(0f, 1f, 0f, 0f),
            new Vector4(-s, 0f, s, 0f),
            new Vector4(2f*s, s-3f, 3f-2f*s, -s),
            new Vector4(-s, 2f-s, s-2f, s)
        ), weights, positions, startHandle, endHandle);
        return this;
    }
    
    /// <summary>
    /// A Catmull rom spline, but with the ability to set the tension. Tension of 0 is standard Catmull-Rom, tension of 1 is linear interpolation.
    /// </summary>
    /// <param name="positions">the anchor positions</param>
    /// <param name="tension">the tension of the spline</param>
    /// <param name="looped">whether the spline is looped</param>
    /// <returns>The generated Spline</returns>
    public Spline SetFromCardinalPositions(IList<float> positions, float tension, bool looped = false) {
        if (looped) {
            return SetFromCardinalPositionsLooped(positions, tension);
        }
        var startHandle = positions[0] + (positions[0] - positions[1]);
        var endHandle = positions[^1] + (positions[^1] - positions[^2]);
        return SetFromCardinalPositions(positions, startHandle, endHandle, tension);
    }
    
    /// <summary>
    /// A proper C1 continuous spline, positions are always touched, velocity is smooth.
    /// </summary>
    /// <param name="positions">the anchor positions</param>
    /// <returns>The generated Spline</returns>
    private Spline SetFromCatmullPositionsLooped(IList<float> positions) {
        weights.Clear();
        GetWeightsLooped(new Matrix4x4(
            new Vector4(0f, 2f, 0f, 0f)*0.5f,
            new Vector4(-1f, 0f, 1f, 0f)*0.5f,
            new Vector4(2f, -5f, 4f, -1f)*0.5f,
            new Vector4(-1f, 3f, -3f, 1f)*0.5f
        ), weights, positions);
        return this;
    }

    /// <summary>
    /// A proper C1 continuous spline, positions are always touched, velocity is smooth.
    /// </summary>
    /// <param name="positions">the anchor positions</param>
    /// <param name="startHandle">handle for start position</param>
    /// <param name="endHandle">handle for end position</param>
    /// <returns>The generated Spline</returns>
    public Spline SetFromCatmullPositions(IList<float> positions, float startHandle, float endHandle) {
        weights.Clear();
        GetWeightsWithEndHandles(new Matrix4x4(
            new Vector4(0f, 2f, 0f, 0f)*0.5f,
            new Vector4(-1f, 0f, 1f, 0f)*0.5f,
            new Vector4(2f, -5f, 4f, -1f)*0.5f,
            new Vector4(-1f, 3f, -3f, 1f)*0.5f
        ), weights, positions, startHandle, endHandle);
        return this;
    }

    /// <summary>
    /// A proper C1 continuous spline, positions are always touched, velocity is smooth. With automatic start and end handles
    /// </summary>
    /// <param name="positions">the anchor positions</param>
    /// <param name="looped">whether the spline should be looped</param>
    /// <returns>The generated Spline</returns>
    public Spline SetFromCatmullPositions(IList<float> positions, bool looped = false) {
        if (looped) {
            return SetFromCatmullPositionsLooped(positions);
        }
        var startHandle = positions[0] + (positions[0] - positions[1]);
        var endHandle = positions[^1] + (positions[^1] - positions[^2]);
        return SetFromCatmullPositions(positions, startHandle, endHandle);
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
                var endSegment = weights.Count - 1;
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
                var vel = SampleCurveSegmentVelocity(weights.Count - 1, 1f);
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
}

}