using NWH.WheelController3D;
using UnityEngine;

namespace Nyoom
{
    /*
    public class TrailController : MonoBehaviour
    {
        private KartController m_kartController;
        private WheelController m_leftWheel;
        private WheelController m_rightWheel;

        private TrailRenderer m_antiGravityTrail;
        private TrailRenderer m_dirtTrail;
        private TrailRenderer m_roadDriftTrail;

        private void Awake()
        {
            m_kartController = GetComponentInParent<KartController>();
        }

        private void Update()
        {
            var canEmit = false;
            m_trail.emitting = false;
            switch (m_kartController.RuntimeInfo.KartMode)
            {
                case KartMode.AntiGravity:
                    if (m_isAntiGrav)
                    {
                        canEmit = true;
                    }
                    else
                    {
                        canEmit = false;
                    }
                    break;
                case KartMode.Normal:
                case KartMode.InAir:
                default:
                    if (m_isAntiGrav)
                    {
                        canEmit = false;
                    }
                    else
                    {
                        canEmit = true;
                    }
                    break;
            }

            if (canEmit)
            {
                if (m_wheelCollider.GetGroundHit(out var hit) && hit.collider.CompareTag("Road"))
                {
                    m_trail.emitting = true;
                }
                else
                {
                    m_trail.emitting = false;
                }

                if (m_trail.emitting)
                {
                    Reposition();
                }
            }
        }

        private void SetPositionAndRotation()
        {
            if (m_leftWheel.GetGroundHit(out var leftHit))
            {
                var rotation = 
            }

            if (Physics.Raycast(m_transform.position, -m_transform.up, out var hit, 4f) && hit.normal.y > 0.5f)
            {
                var rotation = m_transform.rotation;
                m_transform.rotation = Quaternion.Lerp(rotation, Quaternion.FromToRotation(m_transform.up * 2, hit.normal) * rotation, 7.5f * Time.deltaTime);
                // Debug.DrawRay(hit.point, hit.normal, Color.white, 20f);
                if (m_wheelCollider.GetGroundHit(out var wheelHit))
                {
                    var currentPosition = m_transform.position;
                    var targetPosition = wheelHit.point;
                    targetPosition.y += 0.1f;
                    m_transform.position = new Vector3(currentPosition.x, targetPosition.y, currentPosition.z);
                }                
            }
        }

        private void EmitAntiGravity()
        {
            m_antiGravityTrail.emitting = true;
            m_dirtTrail.emitting = false;
            m_roadDriftTrail.emitting = false;
        }

        private void EmitDirt()
        {
            m_dirtTrail.emitting = true;
            m_antiGravityTrail.emitting = false;
            m_roadDriftTrail.emitting = false;
        }

        private void EmitRoadDrift()
        {
            m_roadDriftTrail.emitting = true;
            m_antiGravityTrail.emitting = false;
            m_dirtTrail.emitting = false;
        }

        private void DisableEmit()
        {
            m_antiGravityTrail.emitting = false;
            m_dirtTrail.emitting = false;
            m_roadDriftTrail.emitting = false;
        }
    }*/
}