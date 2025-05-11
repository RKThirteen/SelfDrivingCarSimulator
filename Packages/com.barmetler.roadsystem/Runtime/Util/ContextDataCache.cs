using System;
using System.Collections.Generic;

namespace Barmetler.RoadSystem.Util
{
    public interface IInValidatable
    {
        public void Invalidate();
    }

    public class ContextDataCache<DataType, ContextType> : IInValidatable
    {
        private Dictionary<int, DataType> data = new Dictionary<int, DataType>();
        public readonly List<IInValidatable> children = new List<IInValidatable>();

        public void SetData(DataType data, ContextType context)
        {
            this.data[context.GetHashCode()] = data;
        }

        public DataType GetData(ContextType context)
        {
            if (!IsValid(context)) throw new Exception("Cache is invalid");
            return data[context.GetHashCode()];
        }

        public void Invalidate()
        {
            foreach (var child in children)
                child.Invalidate();
            data.Clear();
        }

        public bool IsValid(ContextType context)
        {
            return data.ContainsKey(context.GetHashCode());
        }
    }
}
