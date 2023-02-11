using System;

namespace Nyoom
{
    public static class FMath
    {
        public static float Remap(this float value, float from1, float to1, float from2, float to2)
        {
            return (value - from1) / (to1 - from1) * (to2 - from2) + from2;
        }
        
        public static  bool NearlyEqual(float a, float b, float epsilon) {
            var absA = Math.Abs(a);
            var absB = Math.Abs(b);
            var diff = Math.Abs(a - b);

            // ReSharper disable once CompareOfFloatsByEqualityOperator
            if (a == b)
            { // shortcut, handles infinities
                return true;
            } 
            if (a == 0 || b == 0 || absA + absB < float.MinValue) 
            {
                // a or b is zero or both are extremely close to it
                // relative error is less meaningful here
                return diff < (epsilon * float.MinValue);
            }
            
            return diff / (absA + absB) < epsilon;
        }
    }
}