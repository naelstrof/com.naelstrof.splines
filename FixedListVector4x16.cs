using System;
using System.Runtime.InteropServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;

[Serializable]
[StructLayout(LayoutKind.Sequential)]
public struct FixedListVector4x16 {
    private const int maxLength = 16;
    public int Length { get; private set; }
    [SerializeField] private Vector4 vec0;
    [SerializeField] private Vector4 vec1;
    [SerializeField] private Vector4 vec2;
    [SerializeField] private Vector4 vec3;
    [SerializeField] private Vector4 vec4;
    [SerializeField] private Vector4 vec5;
    [SerializeField] private Vector4 vec6;
    [SerializeField] private Vector4 vec7;
    [SerializeField] private Vector4 vec8;
    [SerializeField] private Vector4 vec9;
    [SerializeField] private Vector4 vec10;
    [SerializeField] private Vector4 vec11;
    [SerializeField] private Vector4 vec12;
    [SerializeField] private Vector4 vec13;
    [SerializeField] private Vector4 vec14;
    [SerializeField] private Vector4 vec15;
    
    public Vector4 this[int index] {
        get => Get(index);
        set => Set(index, value);
    }

    public void Clear() {
        Length = 0;
    }
    
    public void Add(Vector4 vector) {
        if (Length >= maxLength) {
            throw new InvalidOperationException($"Cannot add more vectors to FixedListVector4x16. Maximum capacity reached {Length}.");
        }
        Length++;
        Set(Length-1, vector);
    }
    
    private Vector4 Get(int index) {
        if (index < 0 || index >= Length) {
            throw new IndexOutOfRangeException($"GET: Index out of range {index} for FixedListVector4x16");
        }
        switch (index) {
            case 0: return vec0;
            case 1: return vec1;
            case 2: return vec2;
            case 3: return vec3;
            case 4: return vec4;
            case 5: return vec5;
            case 6: return vec6;
            case 7: return vec7;
            case 8: return vec8;
            case 9: return vec9;
            case 10: return vec10;
            case 11: return vec11;
            case 12: return vec12;
            case 13: return vec13;
            case 14: return vec14;
            case 15: return vec15;
        }
        throw new IndexOutOfRangeException($"GET: Index out of range {index} for FixedListVector4x16");
    }

    private void Set(int index, Vector4 vector) {
        if (index < 0 || index >= Length) {
            throw new IndexOutOfRangeException($"SET: Index out of range {index} for FixedListVector4x16");
        }
        switch (index) {
            case 0: vec0 = vector; break;
            case 1: vec1 = vector; break;
            case 2: vec2 = vector; break;
            case 3: vec3 = vector; break;
            case 4: vec4 = vector; break;
            case 5: vec5 = vector; break;
            case 6: vec6 = vector; break;
            case 7: vec7 = vector; break;
            case 8: vec8 = vector; break;
            case 9: vec9 = vector; break;
            case 10: vec10 = vector; break;
            case 11: vec11 = vector; break;
            case 12: vec12 = vector; break;
            case 13: vec13 = vector; break;
            case 14: vec14 = vector; break;
            case 15: vec15 = vector; break;
            default: {
                throw new IndexOutOfRangeException($"SET: Index out of range {index} for FixedListVector4x16");
            }
        }
    }
    
    public static FixedListVector4x16 Lerp(FixedListVector4x16 a, FixedListVector4x16 b, float t) {
        FixedListVector4x16 result = new FixedListVector4x16();
        for (int i = 0; i < Mathf.Min(a.Length, b.Length); i++) {
            Vector4 vectorA = a.Get(i);
            Vector4 vectorB = b.Get(i);
            result.Set(i, Vector4.Lerp(vectorA, vectorB, t));
        }
        return result;
    }
    
    public unsafe void CopyFrom(NativeArray<Vector4> source) {
        if (!source.IsCreated || source.Length != maxLength) {
            throw new ArgumentException(
                $"Source must be a created array of {maxLength} slices.",
                nameof(source));
        }

        if (sizeof(FixedListVector4x16) != maxLength * sizeof(Vector4)) {
            throw new InvalidOperationException("FixedListVector4x16 layout has changed.");
        }

        fixed (Vector4* destination = &vec0) {
            UnsafeUtility.MemCpy(
                destination,
                source.GetUnsafeReadOnlyPtr(),
                (long)maxLength * sizeof(Vector4));
        }
    }

    public unsafe void CopyTo(NativeArray<Vector4> destination) {
        if (!destination.IsCreated || destination.Length != maxLength) {
            throw new ArgumentException(
                $"Destination must be a created array of {maxLength} slices.",
                nameof(destination));
        }

        if (sizeof(FixedListVector4x16) != maxLength * sizeof(Vector4)) {
            throw new InvalidOperationException("FixedListVector4x16 layout has changed.");
        }

        fixed (Vector4* source = &vec0) {
            UnsafeUtility.MemCpy(
                destination.GetUnsafePtr(),
                source,
                (long)maxLength * sizeof(Vector4));
        }
    }
}
