using UnityEngine;

namespace Nyoom
{
    public class AntiRollBar : MonoBehaviour
    {
        [SerializeField] private WheelCollider m_frontLeftWheel;
        [SerializeField] private WheelCollider m_frontRightWheel;
        [SerializeField] private WheelCollider m_rearLeftWheel;
        [SerializeField] private WheelCollider m_rearRightWheel;
        
        [SerializeField] private float m_antiRoll = 5000f;


        public void Stabilize(Rigidbody rb, AxleInfo axleInfo)
        {
            var left = axleInfo.LeftWheel;
            var right = axleInfo.RightWheel;
            
            var leftTravel = 1f;
            var rightTravel = 1f;
            
            var isLeftGrounded = left.GetGroundHit(out var hit);
            if (isLeftGrounded)
            {
                leftTravel = (-left.transform.InverseTransformPoint(hit.point).y - left.radius) / left.suspensionDistance;
            }

            var isRightGrounded = right.GetGroundHit(out hit);
            if (isRightGrounded)
            {
                rightTravel = (-right.transform.InverseTransformPoint(hit.point).y - right.radius) / right.suspensionDistance;
            }

            var antiRollForce = (leftTravel - rightTravel) * m_antiRoll;

            var leftTransform = left.transform;
            var rightTransform = right.transform;
            if (isLeftGrounded)
            {
                rb.AddForceAtPosition(leftTransform.up * -antiRollForce, leftTransform.position);
            }

            if (isRightGrounded)
            {
                rb.AddForceAtPosition(rightTransform.up * -antiRollForce, rightTransform.position);
            }
        }
    }
}