using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace NWH.WheelController3D
{
    [RequireComponent(typeof(Rigidbody))]
    [ExecuteInEditMode]
    public class CenterOfMass : MonoBehaviour
    {
        private Vector3 m_centerOfMass;
        public Vector3 centerOfMassOffset = Vector3.zero;
        private Vector3 m_prevOffset = Vector3.zero;
        [SerializeField] private Rigidbody m_rigidbody;
        public bool showCOM = true;

        private void Start()
        {
            // m_rigidbody = GetComponent<Rigidbody>();
            m_centerOfMass = m_rigidbody.centerOfMass;
        }
        
        private void Update()
        {
            if (centerOfMassOffset != m_prevOffset)
            {
                m_rigidbody.centerOfMass = m_centerOfMass + centerOfMassOffset;
            }
            
            m_prevOffset = centerOfMassOffset;
        }

        private void OnDrawGizmos()
        {
            if (showCOM && m_rigidbody != null)
            {
                var radius = 0.1f;
                try
                {
                    radius = GetComponent<MeshFilter>().sharedMesh.bounds.size.z / 10f;
                }
                catch { }

                Gizmos.color = Color.green;
                Gizmos.DrawSphere(m_rigidbody.transform.TransformPoint(m_rigidbody.centerOfMass), radius);
            }
        }
    }
}

