using System;
using System.Diagnostics;
using JetBrains.Annotations;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;

namespace Barmetler.RoadSystem.Util
{
    /// <summary>
    /// Two-dimensional array that is stored in a single-dimensional NativeArray.
    /// </summary>
    /// <typeparam name="T">Type of the elements in the array.</typeparam>
    [DebuggerDisplay("Length = {Length}")]
    [DebuggerTypeProxy(typeof(TwoDimensionalNativeArrayDebugView<>))]
    [BurstCompatible(GenericTypeArguments = new[] { typeof(int) })]
    public struct TwoDimensionalNativeArray<T> where T : struct
    {
        public readonly int Width;
        public readonly int Height;
        public readonly int Length;
        private NativeArray<T> _data;

        public TwoDimensionalNativeArray(int width, int height, Allocator allocator)
        {
            if (width < 0 || height < 0)
                throw new ArgumentException("Width and height must be >= 0");
            Width = width;
            Height = height;
            Length = width * height;
            _data = new NativeArray<T>(width * height, allocator);
        }

        public bool IsCreated => _data.IsCreated;

        [WriteAccessRequired]
        public void Dispose() => _data.Dispose();

        public void Dispose(JobHandle inputDeps) => _data.Dispose(inputDeps);

        public T this[int x, int y]
        {
            get => _data[x + y * Width];
            [WriteAccessRequired] set => _data[x + y * Width] = value;
        }

        public T this[int index]
        {
            get => _data[index];
            [WriteAccessRequired] set => _data[index] = value;
        }

        public unsafe ref T ElementAt(int x, int y) =>
            ref UnsafeUtility.ArrayElementAsRef<T>(_data.GetUnsafePtr(), x + y * Width);

        public unsafe ref T ElementAt(int index) =>
            ref UnsafeUtility.ArrayElementAsRef<T>(_data.GetUnsafePtr(), index);

        public unsafe void* GetUnsafePtr() => _data.GetUnsafePtr();

        public NativeArray<T> AsNativeArray() => _data;

        public T[] ToArray() => _data.ToArray();

        public void CopyFrom(T[] array) => _data.CopyFrom(array);
    }

    internal sealed class TwoDimensionalNativeArrayDebugView<T> where T : struct
    {
        private TwoDimensionalNativeArray<T> _array;

        public TwoDimensionalNativeArrayDebugView(TwoDimensionalNativeArray<T> array) => _array = array;

        [UsedImplicitly] public T[] Items => _array.ToArray();
    }

    /// <summary>
    /// Two-dimensional array that adds columns and rows to an existing array without the need to copy the data.
    /// </summary>
    /// <typeparam name="T">Type of the elements in the array.</typeparam>
    [DebuggerDisplay("Length = {Length}")]
    [DebuggerTypeProxy(typeof(ExtendedTwoDimensionalNativeArrayDebugView<>))]
    [BurstCompatible(GenericTypeArguments = new[] { typeof(int) })]
    public struct ExtendedTwoDimensionalNativeArray<T> where T : struct
    {
        public readonly int Width;
        public readonly int Height;
        public readonly int StartX;
        public readonly int StartY;
        public readonly int Length;
        private TwoDimensionalNativeArray<T> _data;
        private TwoDimensionalNativeArray<T> _horizontal;
        private TwoDimensionalNativeArray<T> _vertical;

        public ExtendedTwoDimensionalNativeArray(TwoDimensionalNativeArray<T> data, int startX, int startY, int width,
            int height, Allocator allocator)
        {
            if (startX + data.Width > width || startY + data.Height > height)
                throw new ArgumentException("Data does not fit into the array.");
            Width = width;
            Height = height;
            StartX = startX;
            StartY = startY;
            Length = width * height;
            _data = data;
            // top and bottom rows in full width
            _horizontal = new TwoDimensionalNativeArray<T>(width, height - _data.Height, allocator);
            // left and right columns without top and bottom rows
            _vertical = new TwoDimensionalNativeArray<T>(width - _data.Width, _data.Height, allocator);
        }

        public bool IsCreated => _data.IsCreated;

        /// <summary>
        /// Does not dispose _data
        /// </summary>
        [WriteAccessRequired]
        public void Dispose()
        {
            _horizontal.Dispose();
            _vertical.Dispose();
        }

        public void Dispose(JobHandle inputDeps)
        {
            _horizontal.Dispose(inputDeps);
            _vertical.Dispose(inputDeps);
        }

        private enum Location
        {
            Horizontal,
            Vertical,
            Data
        }

        private void GetDataLocation(int x, int y, out Location location, out int index)
        {
            if (y < StartY)
            {
                location = Location.Horizontal;
                index = y * Width + x;
            }
            else if (y >= StartY + _data.Height)
            {
                location = Location.Horizontal;
                index = (y - _data.Height) * Width + x;
            }
            else if (x < StartX)
            {
                location = Location.Vertical;
                index = (y - StartY) * _vertical.Width + x;
            }
            else if (x >= StartX + _data.Width)
            {
                location = Location.Vertical;
                index = (y - StartY) * _vertical.Width + (x - _data.Width);
            }
            else
            {
                location = Location.Data;
                index = (y - StartY) * _data.Width + (x - StartX);
            }
        }

        public T this[int x, int y]
        {
            get
            {
                GetDataLocation(x, y, out var location, out var index);
                switch (location)
                {
                    case Location.Horizontal:
                        return _horizontal[index];
                    case Location.Vertical:
                        return _vertical[index];
                    case Location.Data:
                        return _data[index];
                    default:
                        Debug.Fail("Invalid location.");
                        return default;
                }
            }
            set
            {
                GetDataLocation(x, y, out var location, out var index);
                switch (location)
                {
                    case Location.Horizontal:
                        _horizontal[index] = value;
                        break;
                    case Location.Vertical:
                        _vertical[index] = value;
                        break;
                    case Location.Data:
                        _data[index] = value;
                        break;
                    default:
                        Debug.Fail("Invalid location.");
                        break;
                }
            }
        }

        public T this[int index]
        {
            get => this[index % Width, index / Width];
            set => this[index % Width, index / Width] = value;
        }

        public unsafe ref T ElementAt(int x, int y)
        {
            GetDataLocation(x, y, out var location, out var index);
            switch (location)
            {
                case Location.Horizontal:
                    return ref _horizontal.ElementAt(index);
                case Location.Vertical:
                    return ref _vertical.ElementAt(index);
                case Location.Data:
                    return ref _data.ElementAt(index);
                default:
                    Debug.Fail("Invalid location.");
                    return ref UnsafeUtility.AsRef<T>(null);
            }
        }

        public ref T ElementAt(int index) => ref ElementAt(index % Width, index / Width);

        /// <summary>
        /// Not necessarily cheap, as it needs to copy the data from individual arrays to a new array.
        /// Some optimizations are done, like copying the entire _data array if no margins to the left or right are present (_vertical).
        /// </summary>
        /// <returns>Array with the same data as this array.</returns>
        public T[] ToArray()
        {
            var result = new NativeArray<T>(Length, Allocator.Temp);
            var horizontal = _horizontal.AsNativeArray();
            var vertical = _vertical.AsNativeArray();
            var data = _data.AsNativeArray();
            if (StartY > 0)
            {
                result.Slice(0, StartY * Width).CopyFrom(horizontal.Slice(0, StartY * Width));
            }

            if (Width == _data.Width)
            {
                result.Slice(StartY * Width, _data.Length).CopyFrom(data);
            }
            else
            {
                for (var dataRow = 0; dataRow < _data.Height; ++dataRow)
                {
                    if (StartX > 0)
                    {
                        result.Slice((StartY + dataRow) * Width, StartX)
                            .CopyFrom(vertical.Slice(dataRow * _vertical.Width, StartX));
                    }

                    if (_data.Width > 0)
                    {
                        result.Slice((StartY + dataRow) * Width + StartX, _data.Width)
                            .CopyFrom(data.Slice(dataRow * _data.Width, _data.Width));
                    }

                    if (StartX + _data.Width < Width)
                    {
                        result.Slice((StartY + dataRow) * Width + StartX + _data.Width, Width - StartX - _data.Width)
                            .CopyFrom(vertical.Slice(dataRow * _vertical.Width + StartX, Width - StartX - _data.Width));
                    }
                }
            }

            if (StartY + _data.Height < Height)
            {
                result.Slice((StartY + _data.Height) * Width, (Height - StartY - _data.Height) * Width)
                    .CopyFrom(horizontal.Slice((Height - StartY - _data.Height) * Width));
            }

            var resultArray = result.ToArray();
            result.Dispose();
            return resultArray;
        }
    }

    internal sealed class ExtendedTwoDimensionalNativeArrayDebugView<T> where T : struct
    {
        private ExtendedTwoDimensionalNativeArray<T> _array;

        public ExtendedTwoDimensionalNativeArrayDebugView(ExtendedTwoDimensionalNativeArray<T> array) => _array = array;

        [UsedImplicitly] public T[] Items => _array.ToArray();
    }
}
