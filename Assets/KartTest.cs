using System.Collections;
using System.Collections.Generic;
using Nyoom;
using UnityEngine;

public class KartTest : MonoBehaviour
{
    [SerializeField] private AxleInfo[] m_axleInfos;
    private Rigidbody m_rigidbody;

    private void Awake()
    {
        m_rigidbody = GetComponent<Rigidbody>();
        var com = transform.Find("CoM");
        m_rigidbody.centerOfMass = transform.InverseTransformPoint(com.position);
    }

    private void Update()
    {
        foreach (var axleInfo in m_axleInfos)
        {
            if (axleInfo.IsRear)
            {
                axleInfo.LeftWheel.GetWorldPose(out var leftPos, out var leftRot);
                axleInfo.LeftWheelMesh.SetPositionAndRotation(leftPos, leftRot);
            
                axleInfo.RightWheel.GetWorldPose(out var rightPos, out var rightRot);
                axleInfo.RightWheelMesh.SetPositionAndRotation(rightPos, rightRot);
            }
        }
    }
    
    private void FixedUpdate()
    {
        var motorTorque = Input.GetAxisRaw("Vertical") * 800f;
        var steerAngle = Input.GetAxisRaw("Horizontal") * 45f;
        foreach (var axleInfo in m_axleInfos)
        {
            if (axleInfo.Steering)
            {
                axleInfo.LeftWheel.steerAngle = steerAngle;
                axleInfo.RightWheel.steerAngle = steerAngle;
            }

            if (axleInfo.Motor)
            {
                axleInfo.LeftWheel.motorTorque = motorTorque;
                axleInfo.RightWheel.motorTorque = motorTorque;
            }
        }
    }
}
