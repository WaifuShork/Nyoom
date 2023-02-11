using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using UnityEngine.UI;
using System;
using NWH.WheelController3D;

/// <summary>
/// Simple vehicle controller intended as a demo script to help showcase WC3D.
/// If you need a complete vehicle physics package that uses WC3D check out NWH Vehicle Physics.
/// Owners of WC3D get 30% off: https://assetstore.unity.com/packages/tools/physics/nwh-vehicle-physics-107332
/// </summary>
namespace NWH.WheelController3D
{
    [System.Serializable]
    public class _Wheel
    {
        public WheelController wc;
        public bool steer;
        public bool power;
    }

    public class CarController : MonoBehaviour
    {
        public bool vehicleIsActive;
        public bool trackSteer;

        [SerializeField]
        public List<_Wheel> wheels;

        private float xAxis;
        private float smoothXAxis;
        private float xAxisVelocity;
        private float yAxis;

        [HideInInspector]
        public float velocity;

        public float maxSteeringAngle = 35;
        public float minSteeringAngle = 20;

        public float maxMotorTorque;
        public float maxBrakeTorque;
        public float antiRollBarForce;

        public void FixedUpdate()
        {
            if (vehicleIsActive)
            {

                xAxis = Input.GetAxis("Horizontal");
                yAxis = Input.GetAxis("Vertical");

                velocity = transform.InverseTransformDirection(this.GetComponent<Rigidbody>().velocity).z;
                smoothXAxis = Mathf.SmoothDamp(smoothXAxis, xAxis, ref xAxisVelocity, 0.12f);

                foreach (_Wheel w in wheels)
                {
                    if (Input.GetKey(KeyCode.Space))
                    {
                        w.wc.brakeTorque = maxBrakeTorque;
                    }
                    else
                    {
                        w.wc.brakeTorque = 0.0f;
                    }

                    if (Mathf.Sign(velocity) < 0.1f && yAxis > 0.1f)
                        w.wc.brakeTorque = maxBrakeTorque;

                    if (w.power) w.wc.motorTorque = maxMotorTorque * yAxis;
                    if (w.steer) w.wc.steerAngle = Mathf.Lerp(maxSteeringAngle, minSteeringAngle, Mathf.Abs(velocity) * 0.05f) * xAxis;
                }
            }

            ApplyAntirollBar();
        }

        public void ApplyAntirollBar()
        {
            for (int i = 0; i < wheels.Count; i += 2)
            {
                WheelController leftWheel = wheels[i].wc;
                WheelController rightWheel = wheels[i + 1].wc;

                if (!leftWheel.springOverExtended && !leftWheel.springBottomedOut && !rightWheel.springOverExtended && !rightWheel.springBottomedOut)
                {
                    float leftTravel = leftWheel.springTravel;
                    float rightTravel = rightWheel.springTravel;

                    float arf = (leftTravel - rightTravel) * antiRollBarForce;

                    if (leftWheel.isGrounded)
                        leftWheel.parent.GetComponent<Rigidbody>().AddForceAtPosition(leftWheel.wheel.up * -arf, leftWheel.wheel.worldPosition);

                    if (rightWheel.isGrounded)
                        rightWheel.parent.GetComponent<Rigidbody>().AddForceAtPosition(rightWheel.wheel.up * arf, rightWheel.wheel.worldPosition);
                }
            }
        }

        public void Active(bool state)
        {
            vehicleIsActive = state;
        }

        public void OnMotorValueChanged(float v)
        {
            maxMotorTorque = v;
        }

        public void OnBrakeValueChanged(float a)
        {
            maxBrakeTorque = a;
        }

    }

}
