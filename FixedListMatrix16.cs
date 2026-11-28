using System;
using System.Runtime.InteropServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;

[Serializable]
[StructLayout(LayoutKind.Sequential)]
public struct FixedListMatrix16 {
    private const int maxLength = 16;
    public int Length { get; private set; }
    [SerializeField] private Matrix4x4 mat0;
    [SerializeField] private Matrix4x4 mat1;
    [SerializeField] private Matrix4x4 mat2;
    [SerializeField] private Matrix4x4 mat3;
    [SerializeField] private Matrix4x4 mat4;
    [SerializeField] private Matrix4x4 mat5;
    [SerializeField] private Matrix4x4 mat6;
    [SerializeField] private Matrix4x4 mat7;
    [SerializeField] private Matrix4x4 mat8;
    [SerializeField] private Matrix4x4 mat9;
    [SerializeField] private Matrix4x4 mat10;
    [SerializeField] private Matrix4x4 mat11;
    [SerializeField] private Matrix4x4 mat12;
    [SerializeField] private Matrix4x4 mat13;
    [SerializeField] private Matrix4x4 mat14;
    [SerializeField] private Matrix4x4 mat15;
    
    public Matrix4x4 this[int index] {
        get => Get(index);
        set => Set(index, value);
    }

    public void Clear() {
        Length = 0;
    }
    
    public void Add(Matrix4x4 matrix) {
        if (Length >= maxLength) {
            throw new InvalidOperationException($"Cannot add more matrices to FixedListMatrix16. Maximum capacity reached {Length}.");
        }
        Length++;
        Set(Length-1, matrix);
    }
    
    private Matrix4x4 Get(int index) {
        if (index < 0 || index >= Length) {
            throw new IndexOutOfRangeException($"GET: Index out of range {index} for FixedArrayMatrix16");
        }
        switch (index) {
            case 0: return mat0;
            case 1: return mat1;
            case 2: return mat2;
            case 3: return mat3;
            case 4: return mat4;
            case 5: return mat5;
            case 6: return mat6;
            case 7: return mat7;
            case 8: return mat8;
            case 9: return mat9;
            case 10: return mat10;
            case 11: return mat11;
            case 12: return mat12;
            case 13: return mat13;
            case 14: return mat14;
            case 15: return mat15;
        }
        throw new IndexOutOfRangeException($"GET: Index out of range {index} for FixedArrayMatrix16");
    }

    private void Set(int index, Matrix4x4 matrix) {
        if (index < 0 || index >= Length) {
            throw new IndexOutOfRangeException($"SET: Index out of range {index} for FixedArrayMatrix16");
        }
        switch (index) {
            case 0: mat0 = matrix; break;
            case 1: mat1 = matrix; break;
            case 2: mat2 = matrix; break;
            case 3: mat3 = matrix; break;
            case 4: mat4 = matrix; break;
            case 5: mat5 = matrix; break;
            case 6: mat6 = matrix; break;
            case 7: mat7 = matrix; break;
            case 8: mat8 = matrix; break;
            case 9: mat9 = matrix; break;
            case 10: mat10 = matrix; break;
            case 11: mat11 = matrix; break;
            case 12: mat12 = matrix; break;
            case 13: mat13 = matrix; break;
            case 14: mat14 = matrix; break;
            case 15: mat15 = matrix; break;
            default: {
                throw new IndexOutOfRangeException($"SET: Index out of range {index} for FixedArrayMatrix16");
            }
        }
    }
    
    public static FixedListMatrix16 Lerp(FixedListMatrix16 a, FixedListMatrix16 b, float t) {
        FixedListMatrix16 result = new FixedListMatrix16();
        for (int i = 0; i < Mathf.Min(a.Length, b.Length); i++) {
            Matrix4x4 matrixA = a.Get(i);
            Matrix4x4 matrixB = b.Get(i);
            Vector4 column1 = Vector4.Lerp(matrixA.GetColumn(0), matrixB.GetColumn(0), t);
            Vector4 column2 = Vector4.Lerp(matrixA.GetColumn(1), matrixB.GetColumn(1), t);
            Vector4 column3 = Vector4.Lerp(matrixA.GetColumn(2), matrixB.GetColumn(2), t);
            Vector4 column4 = Vector4.Lerp(matrixA.GetColumn(3), matrixB.GetColumn(3), t);
            result.Set(i, new Matrix4x4(column1, column2, column3, column4));
        }
        return result;
    }
    
    public unsafe void CopyFrom(NativeArray<Matrix4x4> source) {
        if (!source.IsCreated || source.Length != maxLength) {
            throw new ArgumentException(
                $"Source must be a created array of {maxLength} slices.",
                nameof(source));
        }

        if (sizeof(FixedListMatrix16) != maxLength * sizeof(Matrix4x4)) {
            throw new InvalidOperationException("FixedArrayMatrix16 layout has changed.");
        }

        fixed (Matrix4x4* destination = &mat0) {
            UnsafeUtility.MemCpy(
                destination,
                source.GetUnsafeReadOnlyPtr(),
                (long)maxLength * sizeof(Matrix4x4));
        }
    }

    public unsafe void CopyTo(NativeArray<Matrix4x4> destination) {
        if (!destination.IsCreated || destination.Length != maxLength) {
            throw new ArgumentException(
                $"Destination must be a created array of {maxLength} slices.",
                nameof(destination));
        }

        if (sizeof(FixedListMatrix16) != maxLength * sizeof(Matrix4x4)) {
            throw new InvalidOperationException("FixedArrayMatrix16 layout has changed.");
        }

        fixed (Matrix4x4* source = &mat0) {
            UnsafeUtility.MemCpy(
                destination.GetUnsafePtr(),
                source,
                (long)maxLength * sizeof(Matrix4x4));
        }
    }
}
