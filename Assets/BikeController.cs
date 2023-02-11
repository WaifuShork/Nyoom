using System;
using UnityEngine;
using DG.Tweening;
using Sirenix.OdinInspector;
using UnityEngine.EventSystems;

namespace Nyoom
{
    public class BikeController : MonoBehaviour
    {
        // private KartMode m_kartMode = KartMode.Normal;
        
        private Rigidbody m_rigidbody;
        private Transform m_transform;
        [SerializeField] private float m_maxVelocity = 30f;
        [SerializeField] private float m_maxMotorTorque = 500f;
        [SerializeField] private float m_maxSteerAngle = 45f;
        
        private InputController m_inputController;
        [SerializeField] private Transform m_camera;
        [SerializeField] private Transform m_cameraPositionTransform;

        [SerializeField, Required] private WheelCollider m_frontWheel;
        [SerializeField, Required] private WheelCollider m_rearWheel;
        
        [SerializeField, Required] private Transform m_bikeMesh;

        public RuntimeInformation RuntimeInfo;
        private float m_velocityMagnitude;
        private float m_medRpm;
        
        private void Awake()
        {
            m_transform = transform;
            RuntimeInfo = new RuntimeInformation();
            m_inputController = new InputController();
            m_rigidbody = GetComponent<Rigidbody>();

            m_rigidbody.mass = 400f;
            m_rigidbody.interpolation = RigidbodyInterpolation.Extrapolate;
            m_rigidbody.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;

            var centerOfMassObject = new GameObject("CenterOfMass")
            {
                transform =
                {
                    parent = m_transform,
                    localPosition = new Vector3(0f, -0.3f, 0f),
                }
            };
            m_rigidbody.centerOfMass = m_transform.InverseTransformPoint(centerOfMassObject.transform.position);
        }

        private void Update()
        {
            if (m_frontWheel.GetGroundHit(out var hit))
            {
                Debug.DrawRay(hit.point, Vector3.up * 0.5f, Color.red, 0.1f);
                
                var distance = Vector3.Distance(m_frontWheel.transform.position, hit.point);
                var result = distance - m_frontWheel.suspensionDistance;
                //Debug.Log(result);
            }
        }

        private void FixedUpdate()
        {
            TickGravity();
            Physics.Raycast(m_transform.position, -m_transform.up, out var hit);
            if (m_inputController.GetButtonDown(InputAction.Drift) && IsGrounded())
            {
                m_rigidbody.AddForce(hit.normal * 7f, ForceMode.VelocityChange);
            }

            if (!IsGrounded())
            {
                m_rigidbody.AddRelativeTorque(hit.normal * (m_inputController.GetAxisRaw(InputAction.Steering) * 0.4f), ForceMode.VelocityChange);
            }

            m_medRpm = (m_frontWheel.rpm + m_rearWheel.rpm) / 2f;
            m_velocityMagnitude = m_rigidbody.velocity.magnitude;
            var throttle = m_inputController.GetThrottle();
            var steering = m_inputController.GetAxisRaw(InputAction.Steering);

            if (m_medRpm > 0f)
            {
                var torque = throttle * m_rigidbody.mass * 4f;
                m_rearWheel.motorTorque = torque;
                m_frontWheel.motorTorque = torque;
            }
            else
            {
                var torque = throttle * m_rigidbody.mass * 1.5f;
                m_rearWheel.motorTorque = torque;
                m_frontWheel.motorTorque = torque;
            }

            var steerAngle = steering * m_maxSteerAngle;
            m_frontWheel.steerAngle = Mathf.Lerp(m_frontWheel.steerAngle, steerAngle, 10f * Time.deltaTime);

            if (Math.Abs(m_rearWheel.rpm) > 10000f)
            {
                m_rearWheel.motorTorque = 0f;
                m_rearWheel.brakeTorque = m_rigidbody.mass * 5f;
            }

            if (m_velocityMagnitude < 1f && Math.Abs(throttle) < 0.1f)
            {
                var brakeTorque = m_rigidbody.mass * 2f;
                m_rearWheel.brakeTorque = brakeTorque;
                m_frontWheel.brakeTorque = brakeTorque;
            }
            else
            {
                m_rearWheel.brakeTorque = 0f;
                m_frontWheel.brakeTorque = 0f;
            }
            
            // ApplyWheelRotations();
            Stabilizer();
            m_rigidbody.velocity = Vector3.ClampMagnitude(m_rigidbody.velocity, m_maxVelocity);

            /*var sidewaysFriction = m_rearWheel.sidewaysFriction;
            if (m_inputController.GetButtonHeld(InputAction.Drift))
            {
                sidewaysFriction.extremumValue = 0.8f;
                sidewaysFriction.stiffness = 1.8f;
            }
            else
            {
                sidewaysFriction.extremumValue = 1f;
                sidewaysFriction.stiffness = 2f;
            }

            m_rearWheel.sidewaysFriction = sidewaysFriction;
*/
            // var angles = m_transform.eulerAngles;
            // angles.z = -steering * 20f;
            // angles.z = Math.Clamp(angles.z, -20f, 20f);
            // m_rigidbody.MoveRotation(Quaternion.Euler(angles));
            // m_rigidbody.AddRelativeTorque(m_transform.forward * (-steering * 6f), ForceMode.Acceleration);
        }
        
        private void Stabilizer()
        {
            Physics.Raycast(m_transform.position, -m_transform.up, out var hit);
            
            // var axisFromRotate = Vector3.Cross(transform.up, Vector3.up);
            var axisFromRotate = Vector3.Cross(transform.up, hit.normal);
            var torqueForce = axisFromRotate.normalized * (axisFromRotate.magnitude * 50);
            torqueForce.x *= 0.4f;
            torqueForce -= m_rigidbody.angularVelocity;
            m_rigidbody.AddTorque(torqueForce * (m_rigidbody.mass * 0.02f), ForceMode.Impulse);
     
            var rpmSign = Mathf.Sign(m_medRpm) * 0.02f;
            if (m_velocityMagnitude > 1.0f && m_frontWheel.isGrounded is true and true)
            {
                var steering = m_inputController.GetAxisRaw(InputAction.Steering);
                m_rigidbody.angularVelocity += new Vector3 (0, -steering * rpmSign, 0f);
            }
        }
        
        private void LateUpdate()
        {
            m_camera.DOMove(m_cameraPositionTransform.position, 0.1f);
            // m_camera.DOLookAt(m_transform.position, 01f);
            
            m_camera.DORotateQuaternion(m_cameraPositionTransform.rotation, 0.1f);
        }

        private void TickGravity()
        {
            if (Physics.Raycast(m_frontWheel.transform.position, -m_transform.up, out var hit))
            {
                m_rigidbody.AddForce(hit.normal * -11.25f, ForceMode.Acceleration);
            }
            else
            {
                m_rigidbody.AddForce(Vector3.up * -11.25f, ForceMode.Acceleration);
            }
        }
        
        private bool IsGrounded()
        {
            return m_frontWheel.isGrounded && m_rearWheel.isGrounded;
        }

        private void OnTriggerExit(Collider other)
        {
            if (other.gameObject.CompareTag("AntiGravity") && RuntimeInfo.CanSwitchModes)
            {
                RuntimeInfo.CanSwitchModes = false;
                Invoke(nameof(ResetCanSwitchModes), 2f);
                switch (RuntimeInfo.KartMode)
                {
                    case KartMode.AntiGravity:
                        RuntimeInfo.KartMode = KartMode.Normal;
                        break;
                    case KartMode.Normal:
                    case KartMode.InAir:
                    default:
                        RuntimeInfo.KartMode = KartMode.AntiGravity;
                        break;
                }
            }
        }

        private void ResetCanSwitchModes()
        {
            RuntimeInfo.CanSwitchModes = true;
        }

        /*[SerializeField] private float m_maxMotorTorque = 500f;
        [SerializeField] private float m_maxSteerAngle = 50f;
        [SerializeField] private WheelCollider m_frontWheel;
        [SerializeField] private WheelCollider m_backWheel;
        [SerializeField] private Transform m_cameraPositionTransform;
        [SerializeField] private Camera m_camera;
        
        
        private InputController m_inputController;
        private Transform m_transform;
        private Rigidbody m_rigidbody;

        private DriftDirection m_driftDirection = DriftDirection.None;
        
        private void Awake()
        {
            m_transform = transform;
            m_rigidbody = GetComponent<Rigidbody>();
            m_inputController = new InputController();
        }

        private void LateUpdate()
        {
            var cameraTransform = m_camera.transform;
            cameraTransform.DOMove(m_cameraPositionTransform.position, 0.1f);
            cameraTransform.DOLocalRotateQuaternion(m_cameraPositionTransform.rotation, 0.1f);
        }

        private void FixedUpdate()
        {
            Physics.Raycast(m_transform.position, -m_transform.up, out var hit);
            var currentVelocity = m_transform.InverseTransformDirection(m_rigidbody.velocity);
            
            if (!IsGrounded())
            {
                m_rigidbody.AddRelativeTorque(hit.normal * (m_inputController.GetAxisRaw(InputAction.Steering) * 1000f), ForceMode.VelocityChange);
            }
            else
            {
                if (m_inputController.GetButtonDown(InputAction.Drift))
                {
                    m_rigidbody.AddForce(hit.normal * 5f, ForceMode.VelocityChange);
                }
            }
            
            
            var isDrifting = m_inputController.GetButtonHeld(InputAction.Drift);
            TickGravity();
            
            var steering = m_inputController.GetAxisRaw(InputAction.Steering) * m_maxSteerAngle;
            var motor = m_inputController.GetThrottle() * m_maxMotorTorque;

            // m_frontWheel.steerAngle = Mathf.Lerp(m_frontWheel.steerAngle, steering, 10f * Time.smoothDeltaTime);
            m_frontWheel.motorTorque = motor;
            m_backWheel.motorTorque = motor;

            ApplyWheelRotations();

            // m_rigidbody.AddRelativeTorque(m_transform.forward * steering);
            
            var angles = m_transform.localEulerAngles;

            if (!IsGrounded())
            {
                angles.z = 0f;
            }
            else
            {
                var min = -20f;
                var max = 20f;
                if (isDrifting)
                {
                    switch (m_driftDirection)
                    {
                        case DriftDirection.Left:
                            min = 0f;
                            max = 20f;
                            break;
                        case DriftDirection.Right:
                            min = -20f;
                            max = 0f;
                            break;
                        case DriftDirection.None:
                        default:
                            min = -20f;
                            max = 20f;
                            break;
                    }
                }
                
                angles.z = -steering;
                angles.z = Math.Clamp(angles.z, min, max);
            }
            
            m_transform.DOLocalRotate(angles, 3f);
            
            var sidewaysFriction = m_backWheel.sidewaysFriction;
            var forwardsFriction = m_backWheel.forwardFriction;
            if (isDrifting)
            {
                sidewaysFriction.stiffness = 0.2f;
                sidewaysFriction.extremumValue = 0.2f;
                forwardsFriction.stiffness = 0.2f;
                forwardsFriction.extremumValue = 0.2f;
            }
            else
            {
                sidewaysFriction.stiffness = 2f;
                sidewaysFriction.extremumValue = 2f;
                forwardsFriction.stiffness = 2f;
                forwardsFriction.extremumValue = 2f;
            }
            
            m_backWheel.sidewaysFriction = sidewaysFriction;
            m_backWheel.forwardFriction = forwardsFriction;
            
            if (isDrifting)
            {
                var xVelocity = currentVelocity.x;
                if (xVelocity < 0f)
                {
                    m_driftDirection = DriftDirection.Right;
                }
                else if (xVelocity > 0f)
                {
                    m_driftDirection = DriftDirection.Left;
                }
            }
            else
            {
                m_driftDirection = DriftDirection.None;
            }
        }

        private void TickGravity()
        {
            if (Physics.Raycast(m_frontWheel.transform.position, -m_transform.up, out var hit))
            {
                m_rigidbody.AddForce(hit.normal * -9.81f, ForceMode.Acceleration);
            }
            else
            {
                m_rigidbody.AddForce(Vector3.up * -9.81f, ForceMode.Acceleration);
            }
        }

        private void ApplyWheelRotations()
        {
            m_frontWheel.GetWorldPose(out var frontPosition, out var frontRotation);
            m_frontWheel.transform.GetChild(0).SetPositionAndRotation(frontPosition, frontRotation);
        
            m_backWheel.GetWorldPose(out var backPosition, out var backRotation);
            m_backWheel.transform.GetChild(0).SetPositionAndRotation(backPosition, backRotation);
        }
        
        private bool IsGrounded()
        {
            return m_frontWheel.GetGroundHit(out _) && m_backWheel.GetGroundHit(out _);
        }*/
    }
}
