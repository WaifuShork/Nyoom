using System;
using UnityEngine;

namespace Nyoom
{
    public class GroundTrailPositioner : MonoBehaviour
    {
        [SerializeField] private WheelCollider m_wheelCollider;
        [SerializeField] private bool m_isAntiGrav;

        private TrailRenderer m_trail;
        private Transform m_transform;

        private KartController m_kartController;
        
        private void Awake()
        {
            m_transform = transform;
            m_kartController = GetComponentInParent<KartController>();
            m_trail = GetComponentInChildren<TrailRenderer>();
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
                    Reposition(hit);
                }
            }
        }

        private void Reposition(WheelHit wheelHit)
        {
            //ground normal rotation
            // Ray ground = new Ray(m_transform.position, -transform.up);
            // RaycastHit hit;
            if (Physics.Raycast(m_transform.position, -m_transform.up, out var hit, 4f) && hit.normal.y > 0.5f)
            {
                var rotation = m_transform.rotation;
                m_transform.rotation = Quaternion.Lerp(rotation, Quaternion.FromToRotation(m_transform.up * 2, hit.normal) * rotation, 7.5f * Time.deltaTime);
                // Debug.DrawRay(hit.point, hit.normal, Color.white, 20f);
                
                if (m_wheelCollider.isGrounded)
                {
                    var currentPosition = m_transform.position;
                    var targetPosition = wheelHit.point;
                    targetPosition.y += 0.1f;
                    m_transform.position = new Vector3(currentPosition.x, targetPosition.y, currentPosition.z);
                }                
            }
        }
    }
}
