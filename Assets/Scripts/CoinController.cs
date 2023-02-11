using System;
using DG.Tweening;
using UnityEditor.Rendering;
using UnityEngine;

namespace Nyoom
{
    public class CoinController : MonoBehaviour
    {
        [SerializeField] private float m_rotationSpeed = 50f;
        [SerializeField] private float m_speed = 5f;
        [SerializeField] private float m_height = 0.1f;

        private const float c_respawnDuration = 5f;
        private float m_respawnTimer;
        private bool m_isEnabled = true;

        private Vector3 m_originalPosition;
        private Quaternion m_originalRotation;
        private Transform m_transform;

        private void Awake()
        {
            m_transform = transform;
            m_originalPosition = m_transform.position;
            m_originalRotation = m_transform.rotation;
        }

        private void OnTriggerEnter(Collider other)
        {
            if (!m_isEnabled)
            {
                return;
            }

            var kartController = other.GetComponent<KartController>();
            if (kartController is not null)
            {
                kartController.CollectCoin(1);
                m_isEnabled = false;
                
                m_transform.DOPunchScale(m_transform.localScale * 1.1f, 0.2f).onComplete += () =>
                {
                    m_transform.localScale = Vector3.one;
                    Disable();
                };
            }
        }

        private void Update()
        {
            if (m_isEnabled)
            {
                m_respawnTimer = 0f;
                MoveCoin();
            }
            else
            {
                m_respawnTimer += Time.deltaTime;
            }

            if (m_respawnTimer >= c_respawnDuration)
            {
                m_isEnabled = true;
                Enable();
            }
        }

        private void MoveCoin()
        {
            var newY = (float) Math.Sin(Time.time * m_speed);
            m_transform.position += Vector3.up * (newY * m_height);
            m_transform.Rotate(Vector3.up * (m_rotationSpeed * Time.deltaTime));
        }

        private void Enable()
        {
            // m_transform.position = m_originalPosition;
            // m_transform.rotation = m_originalRotation;
            m_transform.GetChild(0).gameObject.Enable();
        }

        private void Disable()
        {
            m_transform.GetChild(0).gameObject.Disable();
        }
    }
}