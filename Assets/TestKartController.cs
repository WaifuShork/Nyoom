using System;
using UnityEngine;
using NWH.WheelController3D;
using System.Collections.Generic;

namespace Nyoom
{
    public class TestKartController : MonoBehaviour
    {
        private Rigidbody m_rigidbody;
        [SerializeField] private List<_Wheel> m_wheels;

        [SerializeField] private float m_maxSteeringAngle = 50f;
        [SerializeField] private float m_minSteeringAngle = 20f;

        [SerializeField] private float m_maxMotorTorque;
        [SerializeField] private float m_maxBrakeTorque;
        [SerializeField] private float m_antiRollBarForce;

        private float m_velocity;

        private void Awake()
        {
            m_rigidbody = GetComponent<Rigidbody>();
        }
        
        private void FixedUpdate()
        {
            var throttle = Input.GetAxisRaw("Vertical");
            var steering = Input.GetAxisRaw("Horizontal");

            m_velocity = transform.InverseTransformDirection(m_rigidbody.velocity).z;
            var groundNormal = Vector3.up;
            foreach (var wheel in m_wheels)
            {
                if (Input.GetKey(KeyCode.Space))
                {
                    wheel.wc.brakeTorque = m_maxBrakeTorque;
                }
                else
                {
                    wheel.wc.brakeTorque = 0f;
                }

                if (wheel.power)
                {
                    wheel.wc.motorTorque = throttle * m_maxMotorTorque;
                }
                if (wheel.steer)
                {
                    wheel.wc.steerAngle = Mathf.Lerp(m_maxSteeringAngle, m_minSteeringAngle, 10f * Time.deltaTime) * steering;
                }
            }
            
            Debug.Log(groundNormal);
            m_rigidbody.AddForce(groundNormal * -9.81f, ForceMode.Acceleration);
            
            ApplyAntiRollBar();
        }
        
        private void ApplyAntiRollBar()
        {
            for (var i = 0; i < m_wheels.Count; i += 2)
            {
                var leftWheel = m_wheels[i].wc;
                var rightWheel = m_wheels[i + 1].wc;

                if (!leftWheel.springOverExtended && !leftWheel.springBottomedOut && !rightWheel.springOverExtended && !rightWheel.springBottomedOut)
                {
                    var leftTravel = leftWheel.springTravel;
                    var rightTravel = rightWheel.springTravel;

                    var arf = (leftTravel - rightTravel) * m_antiRollBarForce;

                    if (leftWheel.isGrounded)
                    {
                        m_rigidbody.AddForceAtPosition(leftWheel.wheel.up * -arf, leftWheel.wheel.worldPosition);
                    }

                    if (rightWheel.isGrounded)
                    {
                        m_rigidbody.AddForceAtPosition(rightWheel.wheel.up * arf, rightWheel.wheel.worldPosition);
                    }
                }
            }
        }
    }
}
