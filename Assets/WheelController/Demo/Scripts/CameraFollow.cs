using UnityEngine;
using System.Collections;

namespace NWH.WheelController3D
{
    public class CameraFollow : MonoBehaviour
    {
        public Transform target;
        [Range(0, 30f)]
        public float distance;
        [Range(0, 10f)]
        public float height;
        [Range(0, 10f)]
        public float targetUpOffset;
        [Range(-10f, 10f)]
        public float targetForwardOffset;
        [Range(0, 50f)]
        public float smoothing;
        private float angle;
        [Range(0, 5f)]
        public float angleFollowStrength;
        private Vector3 targetForward;

        void Update()
        {

            Vector3 prevTargetForward = targetForward;
            targetForward = Vector3.Lerp(prevTargetForward, target.forward, Time.deltaTime);

            angle = AngleSigned(target.forward, (target.position - transform.position), Vector3.up);

            Vector3 targetCamPos = target.position + targetForward * -(distance) + Vector3.up * height;

            transform.position = targetCamPos;
            transform.LookAt(target.position + Vector3.up * targetUpOffset + target.forward * targetForwardOffset);
            transform.rotation = Quaternion.AngleAxis(-angle * angleFollowStrength, Vector3.up) * transform.rotation;
        }

        /// <summary>
        /// Determine the signed angle between two vectors, with normal 'n'
        /// as the rotation axis.
        /// </summary>
        public static float AngleSigned(Vector3 v1, Vector3 v2, Vector3 n)
        {
            return Mathf.Atan2(
                Vector3.Dot(n, Vector3.Cross(v1, v2)),
                Vector3.Dot(v1, v2)) * Mathf.Rad2Deg;
        }
    }
}
