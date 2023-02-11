using System;
using UnityEngine;

namespace Nyoom
{
    public class Rotater : MonoBehaviour
    {
        [SerializeField] private float m_speed = 35f;
        
        private Transform m_transform;

        private void Awake()
        {
            m_transform = transform;
        }

        private void Update()
        {
            var eulerAngles = m_transform.localEulerAngles;
            eulerAngles.z += m_speed * Time.deltaTime;
            m_transform.localEulerAngles = eulerAngles;
            
            // m_transform.Rotate(m_transform.forward * (m_speed * Time.deltaTime));
        }
    }
}