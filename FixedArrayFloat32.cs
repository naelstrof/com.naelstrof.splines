using System;
using System.Runtime.InteropServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;

[Serializable]
[StructLayout(LayoutKind.Sequential)]
public struct FixedArrayFloat32 {
    public const int sampleCount = 32;
    
    [SerializeField] private float f0;
    [SerializeField] private float f1;
    [SerializeField] private float f2;
    [SerializeField] private float f3;
    [SerializeField] private float f4;
    [SerializeField] private float f5;
    [SerializeField] private float f6;
    [SerializeField] private float f7;
    [SerializeField] private float f8;
    [SerializeField] private float f9;
    [SerializeField] private float f10;
    [SerializeField] private float f11;
    [SerializeField] private float f12;
    [SerializeField] private float f13;
    [SerializeField] private float f14;
    [SerializeField] private float f15;
    [SerializeField] private float f16;
    [SerializeField] private float f17;
    [SerializeField] private float f18;
    [SerializeField] private float f19;
    [SerializeField] private float f20;
    [SerializeField] private float f21;
    [SerializeField] private float f22;
    [SerializeField] private float f23;
    [SerializeField] private float f24;
    [SerializeField] private float f25;
    [SerializeField] private float f26;
    [SerializeField] private float f27;
    [SerializeField] private float f28;
    [SerializeField] private float f29;
    [SerializeField] private float f30;
    [SerializeField] private float f31;
    
    public float this[int index] {
        get => Get(index);
        set => Set(index, value);
    }
    
    public float Get(int index) {
        switch (index) {
            case 0: return f0;
            case 1: return f1;
            case 2: return f2;
            case 3: return f3;
            case 4: return f4;
            case 5: return f5;
            case 6: return f6;
            case 7: return f7;
            case 8: return f8;
            case 9: return f9;
            case 10: return f10;
            case 11: return f11;
            case 12: return f12;
            case 13: return f13;
            case 14: return f14;
            case 15: return f15;
            case 16: return f16;
            case 17: return f17;
            case 18: return f18;
            case 19: return f19;
            case 20: return f20;
            case 21: return f21;
            case 22: return f22;
            case 23: return f23;
            case 24: return f24;
            case 25: return f25;
            case 26: return f26;
            case 27: return f27;
            case 28: return f28;
            case 29: return f29;
            case 30: return f30;
            case 31: return f31;
        }
        throw new IndexOutOfRangeException($"Index out of range {index} for FixedArrayFloat32");
    }

    public void Set(int index, float value) {
        switch (index) {
            case 0: f0 = value; break;
            case 1: f1 = value; break;
            case 2: f2 = value; break;
            case 3: f3 = value; break;
            case 4: f4 = value; break;
            case 5: f5 = value; break;
            case 6: f6 = value; break;
            case 7: f7 = value; break;
            case 8: f8 = value; break;
            case 9: f9 = value; break;
            case 10: f10 = value; break;
            case 11: f11 = value; break;
            case 12: f12 = value; break;
            case 13: f13 = value; break;
            case 14: f14 = value; break;
            case 15: f15 = value; break;
            case 16: f16 = value; break;
            case 17: f17 = value; break;
            case 18: f18 = value; break;
            case 19: f19 = value; break;
            case 20: f20 = value; break;
            case 21: f21 = value; break;
            case 22: f22 = value; break;
            case 23: f23 = value; break;
            case 24: f24 = value; break;
            case 25: f25 = value; break;
            case 26: f26 = value; break;
            case 27: f27 = value; break;
            case 28: f28 = value; break;
            case 29: f29 = value; break;
            case 30: f30 = value; break;
            case 31: f31 = value; break;
            default: {
                throw new IndexOutOfRangeException($"Index out of range {index} for FixedArrayFloat32");
            }
        }
    }
    
    public unsafe void CopyFrom(NativeArray<float> source) {
        if (!source.IsCreated || source.Length != sampleCount) {
            throw new ArgumentException(
                $"Source must be a created array of {sampleCount} slices.",
                nameof(source));
        }

        if (sizeof(FixedArrayFloat32) != sampleCount * sizeof(float)) {
            throw new InvalidOperationException("FixedArrayFloat32 layout has changed.");
        }

        fixed (float* destination = &f0) {
            UnsafeUtility.MemCpy(
                destination,
                source.GetUnsafeReadOnlyPtr(),
                (long)sampleCount * sizeof(float));
        }
    }

    public unsafe void CopyTo(NativeArray<float> destination) {
        if (!destination.IsCreated || destination.Length != sampleCount) {
            throw new ArgumentException(
                $"Destination must be a created array of {sampleCount} slices.",
                nameof(destination));
        }

        if (sizeof(FixedArrayFloat32) != sampleCount * sizeof(float)) {
            throw new InvalidOperationException("FixedArrayFloat32 layout has changed.");
        }

        fixed (float* source = &f0) {
            UnsafeUtility.MemCpy(
                destination.GetUnsafePtr(),
                source,
                (long)sampleCount * sizeof(float));
        }
    }
}
