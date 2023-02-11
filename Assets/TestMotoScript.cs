using System;
using System.Collections;
using System.Collections.Generic;
using DG.Tweening;
using UnityEngine;

public class TestMotoScript : MonoBehaviour
{
    private Transform m_transform;
    private Rigidbody m_rigidbody;
    [SerializeField] private WheelCollider m_frontWheel;
    [SerializeField] private WheelCollider m_backWheel;

    private void Awake()
    {
        m_transform = transform;
        m_rigidbody = GetComponent<Rigidbody>();
    }
    private void FixedUpdate()
    {
        // Stabilize();
        var steering = Input.GetAxisRaw("Horizontal") * 35f;
        var motor = Input.GetAxisRaw("Vertical") * 500f;

        m_frontWheel.steerAngle = steering;
        m_backWheel.motorTorque = motor;
        
        m_frontWheel.GetWorldPose(out var frontPosition, out var frontRotation);
        m_frontWheel.transform.GetChild(0).SetPositionAndRotation(frontPosition, frontRotation);
        
        m_backWheel.GetWorldPose(out var backPosition, out var backRotation);
        m_backWheel.transform.GetChild(0).SetPositionAndRotation(backPosition, backRotation);


        var angles = m_transform.localEulerAngles;
        if (steering < 0f)
        {
            angles.z = 10f;
        }
        else if (steering > 0f)
        {
            angles.z = -10f;
        }
        else
        {
            angles.z = 0f;
        }

        m_transform.localEulerAngles = angles;
        // m_rigidbody.MoveRotation(Quaternion.Slerp(m_rigidbody.rotation, Quaternion.Euler(angles), 5f * Time.deltaTime));
    }

    /*private void Stabilize()
    {
        Physics.Raycast(m_transform.position, -m_transform.up, out var hit);
        // var t = rb.transform;
        var rotation = m_transform.rotation;
        var targetRotation = Quaternion.FromToRotation(m_transform.up, hit.normal) * rotation;
        m_rigidbody.MoveRotation(Quaternion.Slerp(rotation, targetRotation, 5f * Time.deltaTime));
    }*/
}
