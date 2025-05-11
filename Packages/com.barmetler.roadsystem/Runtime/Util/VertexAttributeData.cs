using System;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering;

namespace Barmetler.RoadSystem.Util
{
    [BurstCompatible]
    public struct VertexAttributeData : IDisposable
    {
        [ReadOnly]
        private Mesh.MeshData _sourceMeshData;

        [ReadOnly]
        private NativeArray<VertexAttributeDescriptor> _sourceVertexAttributes;

        private NativeArray<byte> _stream0, _stream1, _stream2, _stream3;
        private readonly int _stream0Stride, _stream1Stride, _stream2Stride, _stream3Stride;
        public readonly int UVChannelCount;

        private struct VertexAttributeInfo
        {
            public int Offset;
            public int Size;
        }

        private NativeArray<VertexAttributeInfo> _vertexAttributeInfos;

        public VertexAttributeData(Mesh.MeshData sourceMeshData,
            NativeArray<VertexAttributeDescriptor> sourceVertexAttributes)
        {
            _sourceMeshData = sourceMeshData;
            _sourceVertexAttributes = sourceVertexAttributes;
            _stream0 = _sourceMeshData.vertexBufferCount >= 1
                ? _sourceMeshData.GetVertexData<byte>(stream: 0)
                : default;
            _stream1 = _sourceMeshData.vertexBufferCount >= 2
                ? _sourceMeshData.GetVertexData<byte>(stream: 1)
                : default;
            _stream2 = _sourceMeshData.vertexBufferCount >= 3
                ? _sourceMeshData.GetVertexData<byte>(stream: 2)
                : default;
            _stream3 = _sourceMeshData.vertexBufferCount >= 4
                ? _sourceMeshData.GetVertexData<byte>(stream: 3)
                : default;
            _stream0Stride = 0;
            _stream1Stride = 0;
            _stream2Stride = 0;
            _stream3Stride = 0;
            UVChannelCount = 0;

            _vertexAttributeInfos =
                new NativeArray<VertexAttributeInfo>(_sourceVertexAttributes.Length, Allocator.Temp);

            foreach (var attribute in _sourceVertexAttributes)
            {
                if (attribute.dimension == 0) continue;
                var info = new VertexAttributeInfo();
                info.Size = attribute.format switch
                {
                    VertexAttributeFormat.UNorm8 => 1,
                    VertexAttributeFormat.SNorm8 => 1,
                    VertexAttributeFormat.UInt8 => 1,
                    VertexAttributeFormat.SInt8 => 1,
                    VertexAttributeFormat.Float16 => 2,
                    VertexAttributeFormat.UNorm16 => 2,
                    VertexAttributeFormat.SNorm16 => 2,
                    VertexAttributeFormat.UInt16 => 2,
                    VertexAttributeFormat.SInt16 => 2,
                    VertexAttributeFormat.Float32 => 4,
                    VertexAttributeFormat.UInt32 => 4,
                    VertexAttributeFormat.SInt32 => 4,
                    _ => info.Size
                };
                info.Size *= attribute.dimension;
                switch (attribute.stream)
                {
                    case 0:
                        info.Offset = _stream0Stride;
                        _stream0Stride += info.Size;
                        break;
                    case 1:
                        info.Offset = _stream1Stride;
                        _stream1Stride += info.Size;
                        break;
                    case 2:
                        info.Offset = _stream2Stride;
                        _stream2Stride += info.Size;
                        break;
                    case 3:
                        info.Offset = _stream3Stride;
                        _stream3Stride += info.Size;
                        break;
                }

                _vertexAttributeInfos[(int)attribute.attribute] = info;

                switch (attribute.attribute)
                {
                    case VertexAttribute.TexCoord0:
                    case VertexAttribute.TexCoord1:
                    case VertexAttribute.TexCoord2:
                    case VertexAttribute.TexCoord3:
                    case VertexAttribute.TexCoord4:
                    case VertexAttribute.TexCoord5:
                    case VertexAttribute.TexCoord6:
                    case VertexAttribute.TexCoord7:
                        UVChannelCount++;
                        break;
                }
            }
        }

        private bool GetVertexData(
            int index, VertexAttribute attribute,
            out NativeArray<byte> result,
            out VertexAttributeDescriptor attributeDescriptor,
            out VertexAttributeInfo attributeInfo)
        {
            attributeDescriptor = _sourceVertexAttributes[(int)attribute];
            attributeInfo = _vertexAttributeInfos[(int)attribute];
            if (attributeDescriptor.dimension == 0) goto fail;
            if (attributeDescriptor.stream < 0 ||
                attributeDescriptor.stream >= _sourceMeshData.vertexBufferCount) goto fail;
            var stream = attributeDescriptor.stream switch
            {
                0 => _stream0,
                1 => _stream1,
                2 => _stream2,
                3 => _stream3,
                _ => throw new ArgumentOutOfRangeException() // unreachable
            };
            var stride = attributeDescriptor.stream switch
            {
                0 => _stream0Stride,
                1 => _stream1Stride,
                2 => _stream2Stride,
                3 => _stream3Stride,
                _ => throw new ArgumentOutOfRangeException() // unreachable
            };
            var offset = index * stride + attributeInfo.Offset;
            result = stream.GetSubArray(offset, attributeInfo.Size);
            return true;
            fail:
            result = default;
            return false;
        }


        public unsafe bool GetFloat(int index, VertexAttribute vertexAttribute, out float result)
        {
            if (!GetVertexData(index, vertexAttribute,
                    out var bytes,
                    out var attributeDescriptor,
                    out _)) goto fail;
            switch (attributeDescriptor.format)
            {
                case VertexAttributeFormat.Float32:
                    result = UnsafeUtility.AsRef<float>(bytes.GetUnsafeReadOnlyPtr());
                    return true;
                case VertexAttributeFormat.Float16:
                    result = UnsafeUtility.AsRef<half>(bytes.GetUnsafeReadOnlyPtr());
                    return true;
            }

            fail:
            result = 0;
            return false;
        }

        public unsafe bool GetFloat2(int index, VertexAttribute vertexAttribute, out float2 result)
        {
            if (!GetVertexData(index, vertexAttribute,
                    out var bytes,
                    out var attributeDescriptor,
                    out _)) goto fail;
            switch (attributeDescriptor.format)
            {
                case VertexAttributeFormat.Float32:
                    result = UnsafeUtility.AsRef<float2>(bytes.GetUnsafeReadOnlyPtr());
                    return true;
                case VertexAttributeFormat.Float16:
                    result = UnsafeUtility.AsRef<half2>(bytes.GetUnsafeReadOnlyPtr());
                    return true;
            }

            fail:
            result = default;
            return false;
        }

        public unsafe bool GetFloat3(int index, VertexAttribute vertexAttribute, out float3 result)
        {
            if (!GetVertexData(index, vertexAttribute,
                    out var bytes,
                    out var attributeDescriptor,
                    out _)) goto fail;
            switch (attributeDescriptor.format)
            {
                case VertexAttributeFormat.Float32:
                    result = UnsafeUtility.AsRef<float3>(bytes.GetUnsafeReadOnlyPtr());
                    return true;
                case VertexAttributeFormat.Float16:
                    result = UnsafeUtility.AsRef<half3>(bytes.GetUnsafeReadOnlyPtr());
                    return true;
            }

            fail:
            result = default;
            return false;
        }

        public unsafe bool GetFloat4(int index, VertexAttribute vertexAttribute, out float4 result)
        {
            if (!GetVertexData(index, vertexAttribute,
                    out var bytes,
                    out var attributeDescriptor,
                    out _)) goto fail;
            switch (attributeDescriptor.format)
            {
                case VertexAttributeFormat.Float32:
                    result = UnsafeUtility.AsRef<float4>(bytes.GetUnsafeReadOnlyPtr());
                    return true;
                case VertexAttributeFormat.Float16:
                    result = UnsafeUtility.AsRef<half4>(bytes.GetUnsafeReadOnlyPtr());
                    return true;
            }

            fail:
            result = default;
            return false;
        }

        public void Dispose()
        {
            if (_stream0.IsCreated) _stream0.Dispose();
            if (_stream1.IsCreated) _stream1.Dispose();
            if (_stream2.IsCreated) _stream2.Dispose();
            if (_stream3.IsCreated) _stream3.Dispose();
            _vertexAttributeInfos.Dispose();
        }
    }
}
