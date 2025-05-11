namespace Barmetler.RoadSystem.Util
{
    public static class Functional
    {
        public static T Let<T>(this T value, out T result) where T : class
        {
            result = value;
            return value;
        }
    }
}
