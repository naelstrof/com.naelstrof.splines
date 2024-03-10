using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CatmullSpline {
    public const int BINORMAL_LUT_COUNT = 32;
    public const int DISTANCE_LUT_COUNT = 32;

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

    private static Matrix4x4 catmullMatrix = new Matrix4x4(
        new Vector4(0f, 2f, 0f, 0f)*0.5f,
        new Vector4(-1f, 0f, 1f, 0f)*0.5f,
        new Vector4(2f, -5f, 4f, -1f)*0.5f,
        new Vector4(-1f, 3f, -3f, 1f)*0.5f);

    private static Matrix4x4 GenerateCentripetalCatmullMatrix(float a, float b, float c, float d) {
        Vector4 p0Fact = new Vector4(b * c * c,  -2f * b * c - c * c,    b + 2f * c,  -1f) * (1f / ((b - a) * (c - a) * (c - b)));
        Vector4 p1Fact = new Vector4(-a * c * c, 2f * a * c + c * c,     -a - 2f * c, 1f)  * (1f / ((b - a) * (c - a) * (c - b)));
        p1Fact +=        new Vector4(-a * c * c, 2f * a * c + c * c,     -a - 2f * c, 1f)  * (1f / ((c - b) * (c - a) * (c - b)));
        p1Fact +=        new Vector4(-b * c * d, b * d + b * c + c * d,  -d - c - b,  1f)  * (1f / ((c - b) * (d - b) * (c - b)));
        Vector4 p2Fact = new Vector4(a * b * c,  -a * b - a * c - b * c, a + b + c,   -1f) * (1f / ((c - b) * (c - a) * (c - b)));
        p2Fact +=        new Vector4(d * b * b,  -(b*b)-2f*b*d,          d+2f*b,      -1f) * (1f / ((c - b) * (d - b) * (c - b)));
        p2Fact +=        new Vector4(d * b * b,  -(b*b)-2f*b*d,          d+2f*b,      -1f) * (1f / ((d - c) * (d - b) * (c - b)));
        Vector4 p3Fact = new Vector4(-c*b*b,     b*b+2f*b*c,             -c-2f*b,     1f)  * (1f / ((d - c) * (d - b) * (c - b)));
        
        Matrix4x4 rowMajor = new Matrix4x4();
        rowMajor.SetRow(0, p0Fact);
        rowMajor.SetRow(1, p1Fact);
        rowMajor.SetRow(2, p2Fact);
        rowMajor.SetRow(3, p3Fact);
        return rowMajor;
    }
    private static Matrix4x4 GenerateCentripetalCatmullMatrixFast(float a, float d) {
        Vector4 p0Fact = new Vector4(0f, -1f,        2f,      -1f) * (1f / (-a*(1f - a)));
        Vector4 p1Fact = new Vector4(-a, 2f * a + 1f,-a - 2f, 1f)  * (1f / (-a * (1f - a)));
        p1Fact +=        new Vector4(-a, 2f * a + 1f,-a - 2f, 1f)  * (1f / (1f - a));
        p1Fact +=        new Vector4(0f, d,          -d - 1f, 1f)  * (1f / d);
        Vector4 p2Fact = new Vector4(0f, -a,         a + 1f,  -1f) * (1f / (1f - a));
        p2Fact +=        new Vector4(0f, 0f,         d,       -1f) * (1f / d);
        p2Fact +=        new Vector4(0f, 0f,         d,       -1f) * (1f / ((d - 1f) * d));
        Vector4 p3Fact = new Vector4(0f, 0f,         -1f,     1f)  * (1f / ((d - 1f) * (d)));
        
        Matrix4x4 rowMajor = new Matrix4x4();
        rowMajor.SetRow(0, p0Fact);
        rowMajor.SetRow(1, p1Fact);
        rowMajor.SetRow(2, p2Fact);
        rowMajor.SetRow(3, p3Fact);
        return rowMajor;
    }

    public CatmullSpline(IList<Vector3> points) {
        SetWeightsFromPoints(points);
    }
    
    public CatmullSpline(IList<Matrix4x4> weights) {
        SetWeights(weights);
    }

    private static CatmullSpline Lerp(CatmullSpline a, CatmullSpline b, float t) {
        List<Matrix4x4> merged = new List<Matrix4x4>(a.weights);
        for(int i=0;i<merged.Count;i++) {
            Vector4 column1 = Vector4.Lerp(merged[i].GetColumn(0), b.weights[i].GetColumn(0), t);
            Vector4 column2 = Vector4.Lerp(merged[i].GetColumn(1), b.weights[i].GetColumn(1), t);
            Vector4 column3 = Vector4.Lerp(merged[i].GetColumn(2), b.weights[i].GetColumn(2), t);
            Vector4 column4 = Vector4.Lerp(merged[i].GetColumn(3), b.weights[i].GetColumn(3), t);
            merged[i] = new Matrix4x4(column1, column2, column3, column4);
        }
        return new CatmullSpline(merged);
    }
    public static Vector3 GetPosition(Matrix4x4 weightBlock, float t) {
        return weightBlock * new Vector4(1f, t, t * t, t * t * t);
    }
    public static Vector3 GetVelocity(Matrix4x4 weightBlock, float t) {
        return weightBlock * new Vector4(0f, 1f, 2f*t, 3f*t * t);
    }
    public static Vector3 GetAcceleration(Matrix4x4 weightBlock, float t) {
        return weightBlock * new Vector4(0f, 0f, 2f, 6f*t);
    }

    public List<Matrix4x4> GetWeights() => weights;

    public float[] GetDistanceLUT() {
        if (!distanceLUTGenerated) {
            GenerateDistanceLUT();
        }
        return distanceLUT;
    }

    public Vector3[] GetBinormalLUT() {
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
        curveSegmentIndex = Mathf.Clamp(Mathf.FloorToInt(t*(weights.Count)),0,(weights.Count)-1);
        var offseted = t-((float)curveSegmentIndex/(float)(weights.Count));
        return offseted * (float)(weights.Count);
    }
    
    public float GetTimeFromDistance(float distance) {
        if (!distanceLUTGenerated) {
            GenerateDistanceLUT();
        }
        if (distance > 0f && distance < arcLength) {
            for(int i=0;i<DISTANCE_LUT_COUNT-1;i++) {
                if (distance>distanceLUT[i] && distance<distanceLUT[i+1]) {
                    return Remap(distance,distanceLUT[i],distanceLUT[i+1],(float)i/(float)(DISTANCE_LUT_COUNT),(float)(i+1)/(float)(DISTANCE_LUT_COUNT));
                }
            }
        }
        return distance/arcLength;
    }
    public float GetDistanceFromTime(float t) {
        if (!distanceLUTGenerated) {
            GenerateDistanceLUT();
        }
        t = Mathf.Clamp01(t);
        int index = Mathf.Clamp(Mathf.FloorToInt(t*(DISTANCE_LUT_COUNT-1)),0,DISTANCE_LUT_COUNT-2);
        float offseted = t-((float)index/(float)(DISTANCE_LUT_COUNT));
        float lerpT = offseted * (float)(DISTANCE_LUT_COUNT-1);
        return Mathf.Lerp(distanceLUT[index], distanceLUT[index+1], lerpT);
    }
    private bool CheckMinimaMaxima(float tValue) {
        return !float.IsNaN(tValue) && Mathf.Clamp01(tValue) == tValue;
    }
    // Again we use Freya Holmer's tutorial to understand the strategy behind generating the bounds: https://youtu.be/aVwxzDHniEw?t=791
    // This is just the derivative written in terms of t, then plugged into the quadratic formula to solve for zeros.
    // that gives us our local extremes to generate our bounds on.
    protected void GenerateBounds() {
        bounds.Clear();
        foreach( var weightGroup in weights) {
            Vector3 a = weightGroup * new Vector4(0f,0f,0f,3f);
            Vector3 b = weightGroup * new Vector4(0f,0f,2f,0f);
            Vector3 c = weightGroup * new Vector4(0f,1f,0f,0f);

            // Good ol' quadratic formula. We solve it for each axis (X,Y,Z);
            float tpx = (-b.x + Mathf.Sqrt(b.x*b.x-4f*a.x*c.x))/(2f*a.x);
            float tpy = (-b.y + Mathf.Sqrt(b.y*b.y-4f*a.y*c.y))/(2f*a.y);
            float tpz = (-b.z + Mathf.Sqrt(b.z*b.z-4f*a.z*c.z))/(2f*a.z);

            float tnx = (-b.x - Mathf.Sqrt(b.x*b.x-4f*a.x*c.x))/(2f*a.x);
            float tny = (-b.y - Mathf.Sqrt(b.y*b.y-4f*a.y*c.y))/(2f*a.y);
            float tnz = (-b.z - Mathf.Sqrt(b.z*b.z-4f*a.z*c.z))/(2f*a.z);

            Bounds bound = new Bounds(GetPosition(weightGroup, 0f), Vector3.zero);
            // If the floats are out of our range, or if they're NaN (from a negative sqrt)-- we discard them.
            if (CheckMinimaMaxima(tpx)) { bound.Encapsulate(GetPosition(weightGroup,tpx)); }
            if (CheckMinimaMaxima(tnx)) { bound.Encapsulate(GetPosition(weightGroup,tnx)); }
            if (CheckMinimaMaxima(tpy)) { bound.Encapsulate(GetPosition(weightGroup,tpy)); }
            if (CheckMinimaMaxima(tny)) { bound.Encapsulate(GetPosition(weightGroup,tny)); }
            if (CheckMinimaMaxima(tpz)) { bound.Encapsulate(GetPosition(weightGroup,tpz)); }
            if (CheckMinimaMaxima(tnz)) { bound.Encapsulate(GetPosition(weightGroup,tnz)); }
            bound.Encapsulate(GetPosition(weightGroup,1f));
            bounds.Add(bound);
        }
    }
    protected void GenerateDistanceLUT() {
        float dist = 0f;
        Vector3 lastPosition = GetPositionFromT(0f);
        for(int i=0;i<DISTANCE_LUT_COUNT;i++) {
            float t = (((float)i)/(float)DISTANCE_LUT_COUNT);
            Vector3 position = GetPositionFromT(t);
            dist += Vector3.Distance(lastPosition, position);
            lastPosition = position;
            distanceLUT[i] = dist;
        }
        distanceLUTGenerated = true;
        arcLength = dist;
    }
    protected void GenerateBinormalLUT() {
        // https://en.wikipedia.org/wiki/Frenet%E2%80%93Serret_formulas
        // https://janakiev.com/blog/framing-parametric-curves/
        Vector3 lastTangent = GetVelocityFromT(0).normalized;
        // Initial reference frame, uses Vector3.up
        Vector3 lastBinormal = Vector3.Cross(GetVelocityFromT(0),Vector3.up).normalized;
        if (lastBinormal.magnitude == 0f) {
            lastBinormal = Vector3.Cross(GetVelocityFromT(0),Vector3.right).normalized;
        }
        for(int i=0;i<BINORMAL_LUT_COUNT;i++) {
            float t = (((float)i)/(float)BINORMAL_LUT_COUNT);
            Vector3 tangent = GetVelocityFromT(t).normalized;
            Vector3 binormal = Vector3.Cross(lastTangent, tangent);
            if (binormal.magnitude == 0f) {
                binormal = lastBinormal;
            } else {
                float theta = Vector3.Angle(lastTangent, tangent); // equivalent to Mathf.Acos(Vector3.Dot(lastTangent,tangent))
                binormal = Quaternion.AngleAxis(theta,binormal.normalized)*lastBinormal;
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
    protected CatmullSpline SetWeights(IList<Matrix4x4> newWeights) {
        weights.Clear();
        weights.AddRange(newWeights);
        distanceLUTGenerated = false;
        binormalLUTGenerated = false;
        bounds.Clear();
        return this;
    }
    
    private static float Remap (float value, float from1, float to1, float from2, float to2) {
        return (value - from1) / (to1 - from1) * (to2 - from2) + from2;
    }
    private static float GetKnotInterval( Vector3 a, Vector3 b, float alpha) {
        return Mathf.Pow( Vector3.SqrMagnitude( a - b ), 0.5f*alpha);
    }

    public static void GetWeightsFromPoints(ICollection<Matrix4x4> weightCollection, IList<Vector3> newPoints) {
        // 0 for standard Catmull-Rom
        // 0.5 for Centripetal Catmull-Rom
        // 1 for Chordal Catmull-Rom
        const float alpha = 0.5f;
        for (int i=0;i<newPoints.Count-1;i++) {
            var distance = 0f;
            float interval1;
            Vector3 p0;
            Vector3 p1 = newPoints[i];
            Vector3 p2 = newPoints[i+1];
            Vector3 p3;
            if (i==0) { // Projected point
                Vector3 previousPoint = p1 + (p1-p2);
                p0 = previousPoint;
                distance += GetKnotInterval(previousPoint, p1, alpha);
                interval1 = distance;
            } else {
                Vector3 previousPoint = newPoints[i-1];
                p0 = previousPoint;
                distance += GetKnotInterval(previousPoint, p1, alpha);
                interval1 = distance;
            }
            distance += GetKnotInterval(p1, p2, alpha);
            var interval2 = distance;
            if (i >= newPoints.Count - 2) { // Projected end
                Vector3 pointAfter = p2 + (p2 - p1);
                p3 = pointAfter;
                distance += GetKnotInterval(p2, pointAfter, alpha);
            } else {
                Vector3 pointAfter = newPoints[i + 2];
                p3 = pointAfter;
                distance += GetKnotInterval(p2, pointAfter, alpha);
            }
            var pointMatrix = new Matrix4x4(p0,p1,p2,p3);
            weightCollection.Add(pointMatrix*GenerateCentripetalCatmullMatrixFast( Remap(0f, interval1/distance, interval2/distance, 0f, 1f), Remap(1f, interval1/distance, interval2/distance, 0f, 1f)));
        }
    }
    public float GetDistanceFromSubT(int start, int end, float subT) {
        var subSplineCount = weights.Count;
        var subSection = end - start;
        var multi = (float)subSection / (float)subSplineCount;
        var startT = (float)start / (float)subSplineCount;
        var t = subT * multi + startT;
        return GetDistanceFromTime(t);
    }

    public CatmullSpline SetWeightsFromPoints(IList<Vector3> newPoints) {
        weights.Clear();
        GetWeightsFromPoints(weights,newPoints);
        distanceLUTGenerated = false;
        binormalLUTGenerated = false;
        bounds.Clear();
        return this;
    }
    public float GetClosestTimeFromPositionFast(Vector3 position, int samples=32) {
        // Broad pass that just finds the closest bounds
        int closestBounds = 0;
        float distToBounds = float.MaxValue;
        List<Bounds> curveBounds = GetBounds();
        for(int i=0;i<curveBounds.Count;i++) {
            float dist = Vector3.Distance(curveBounds[i].ClosestPoint(position),position);
            if (dist < distToBounds) {
                closestBounds = i;
                distToBounds = dist;
            }
        }
        // With the closest sub-curve found, we can then do some very deliberate samples.
        float closestTValue = 0f;
        float distToCurve = float.MaxValue;
        for (int i=0;i<samples;i++) {
            float tSample = (float)i/(float)samples;
            Vector3 samplePosition = SampleCurveSegmentPosition(closestBounds, tSample);
            float dist = Vector3.Distance(samplePosition, position);
            if (dist<distToCurve) {
                closestTValue = tSample;
                distToCurve = dist;
            }
        }
        // Just gotta take it from subT to overall t value
        return closestTValue/(float)(curveBounds.Count)+((float)closestBounds/(float)(curveBounds.Count));
    }
    public float GetClosestTimeFromPosition(Vector3 position, int samples=32) {
        var closestTValue = 0f;
        var distToCurve = float.MaxValue;
        for (int i=0;i<samples;i++) {
            var tSample = (float)i/(float)samples;
            Vector3 samplePosition = GetPositionFromT(tSample);
            var dist = Vector3.Distance(samplePosition, position);
            if (dist<distToCurve) {
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
        var index = Mathf.Clamp(Mathf.FloorToInt(t*(BINORMAL_LUT_COUNT-1)),0,BINORMAL_LUT_COUNT-2);
        var offseted = t-((float)index/(float)(BINORMAL_LUT_COUNT));
        var lerpT = offseted * (float)(BINORMAL_LUT_COUNT-1);
        return Vector3.Lerp(binormalLUT[index], binormalLUT[index+1], lerpT);
    }
    public Matrix4x4 GetReferenceFrameFromT(float t) {
        Vector3 tangent = GetVelocityFromT(t).normalized;
        Vector3 binormal = GetBinormalFromT(t).normalized;
        Vector3 normal = Vector3.Cross(tangent, binormal);

        // Change of basis https://math.stackexchange.com/questions/3540973/change-of-coordinates-and-change-of-basis-matrices
        // It also shows up here: https://docs.unity3d.com/ScriptReference/Vector3.OrthoNormalize.html
        Matrix4x4 bezierBasis = new Matrix4x4();
        bezierBasis.SetRow(0,binormal); // Our X axis
        bezierBasis.SetRow(1,normal); // Y Axis
        bezierBasis.SetRow(2,tangent); // Z Axis
        bezierBasis[3,3] = 1f;
        // Change of basis formula is B = P⁻¹ * A * P, where P is the basis transform.
        return bezierBasis.inverse;
    }
}