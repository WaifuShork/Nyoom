using System;
using UnityEngine;
using Sirenix.OdinInspector;

namespace Nyoom
{
    public class AxleLookAt : MonoBehaviour
    {
        [SerializeField, Required] private Transform m_wheelTransform;
        private Transform m_transform;

        private void Awake()
        {
            m_transform = transform;
        }
        
        private void Update()
        {
            m_transform.LookAt(m_wheelTransform);
        }
    }
}
