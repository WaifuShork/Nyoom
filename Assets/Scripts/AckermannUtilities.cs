using System;
using UnityEngine;
// ReSharper disable Unity.InefficientPropertyAccess

namespace Nyoom
{
    public static class AckermannUtilities
    {
        public static void SetSteeringAngle(AxleInfo[] m_axleInfos, float steerAngle)
        {
            
        }
        
        public static float GetSecondaryAngle(float primaryAngle, float separation, float width) 
        {
            // To avoid NaN assume primary angle isn't within [-1,1]
            if (Mathf.Abs(primaryAngle) < 1)
            {
                primaryAngle = Mathf.Abs(primaryAngle);
            }
            
            float close = separation / Mathf.Tan(Mathf.Abs(primaryAngle) * Mathf.Deg2Rad);
            float far = close + width;
            return Mathf.Sign(primaryAngle) * Mathf.Atan(separation / far) * Mathf.Rad2Deg;
        }
        
        public static float[] GetRadii(float primaryAngle, float separation, float width) {
            // To avoid NaN we assume primaryAngle to be at least 1
            primaryAngle = Mathf.Clamp(primaryAngle, 1, Mathf.Infinity);

            var frontPrimary = separation / (float) Math.Sin(primaryAngle * Mathf.Deg2Rad);
            var rearPrimary = separation / (float) Math.Tan(primaryAngle * Mathf.Deg2Rad);
            var rearSecondary = width + rearPrimary;
            var frontSecondary = (float) Math.Sqrt(separation * separation + rearSecondary * rearSecondary);

            return new[] {
                frontPrimary,
                frontSecondary,
                rearPrimary,
                rearSecondary
            };
        }

        public static float GetRadius(float primaryAngle, float separation, float width){
            // To avoid NaN we assume primaryAngle to be at least 1
            primaryAngle = Math.Clamp(primaryAngle, 1f, Mathf.Infinity);
            var radii = GetRadii(primaryAngle, separation, width);
            float sum = 0;

            for (int i = 0; i < radii.Length; i++)
                sum += radii[i];

            return sum / 4;
        }
    }
}