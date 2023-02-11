using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using DG.Tweening;
using UnityEngine;
using Sirenix.OdinInspector;
using UnityEngine.Serialization;

namespace Nyoom
{
    [Serializable]
    public class WheelFriction
    {
        public float ExtremumSlip;
        public float ExtremumValue;
        public float AsymptoteSlip;
        public float AsymptoteValue;
        public float Stiffness;
    }

    public enum KartMode
    {
        Normal,
        InAir,
        AntiGravity,
    }
    
    // Wheel Collider:
    // Extremum Slip -
    // Extremum Value -
    // Asymptote Slip -
    // Asymptote Value -
    // Stiffness -
    [Serializable]
    public class AxleInfo
    {
        public WheelFriction BaseForwardsFriction;
        public WheelFriction BaseSidewaysFriction;
        public WheelFriction DriftingForwardsFriction;
        public WheelFriction DriftingSidewaysFriction;

        public WheelCollider LeftWheel;
        public WheelCollider RightWheel;
        public Transform LeftWheelMesh;
        public Transform RightWheelMesh;

        public bool IsFront;
        public bool IsRear => !IsFront;
        
        public bool Steering;
        public bool Motor;

        private float m_leftSteerRef = 0f;
        private float m_rightSteerRef = 0f;

        /**
         * ExtremumSlip;
ExtremumValue;
AsymptoteSlip;
AsymptoteValue;
Stiffness;
         */
        public void SetForwardsFriction(WheelFriction wheelFriction)
        {
            var leftFriction = LeftWheel.forwardFriction;
            leftFriction.extremumSlip = Mathf.Lerp(leftFriction.extremumSlip, wheelFriction.ExtremumSlip, 10f * Time.smoothDeltaTime);
            leftFriction.extremumValue = Mathf.Lerp(leftFriction.extremumValue, wheelFriction.ExtremumValue, 10f * Time.smoothDeltaTime);
            leftFriction.asymptoteSlip = Mathf.Lerp(leftFriction.asymptoteSlip, wheelFriction.AsymptoteSlip, 10f * Time.smoothDeltaTime);
            leftFriction.asymptoteValue = Mathf.Lerp(leftFriction.asymptoteValue, wheelFriction.AsymptoteValue, 10f * Time.smoothDeltaTime);
            leftFriction.stiffness = Mathf.Lerp(leftFriction.stiffness, wheelFriction.Stiffness, 10f * Time.smoothDeltaTime);
            
            
            var rightFriction = RightWheel.forwardFriction;
            rightFriction.extremumSlip = Mathf.Lerp(rightFriction.extremumSlip, wheelFriction.ExtremumSlip, 10f * Time.smoothDeltaTime);
            rightFriction.extremumValue = Mathf.Lerp(rightFriction.extremumValue, wheelFriction.ExtremumValue, 10f * Time.smoothDeltaTime);
            rightFriction.asymptoteSlip = Mathf.Lerp(rightFriction.asymptoteSlip, wheelFriction.AsymptoteSlip, 10f * Time.smoothDeltaTime);
            rightFriction.asymptoteValue = Mathf.Lerp(rightFriction.asymptoteValue, wheelFriction.AsymptoteValue, 10f * Time.smoothDeltaTime);
            rightFriction.stiffness = Mathf.Lerp(rightFriction.stiffness, wheelFriction.Stiffness, 10f * Time.smoothDeltaTime);

            LeftWheel.forwardFriction = leftFriction;
            RightWheel.forwardFriction = rightFriction;
        }

        public void SetSidewaysFriction(WheelFriction wheelFriction)
        {
            var leftFriction = LeftWheel.sidewaysFriction;
            leftFriction.extremumSlip = Mathf.Lerp(leftFriction.extremumSlip, wheelFriction.ExtremumSlip, 10f * Time.smoothDeltaTime);
            leftFriction.extremumValue = Mathf.Lerp(leftFriction.extremumValue, wheelFriction.ExtremumValue, 10f * Time.smoothDeltaTime);
            leftFriction.asymptoteSlip = Mathf.Lerp(leftFriction.asymptoteSlip, wheelFriction.AsymptoteSlip, 10f * Time.smoothDeltaTime);
            leftFriction.asymptoteValue = Mathf.Lerp(leftFriction.asymptoteValue, wheelFriction.AsymptoteValue, 10f * Time.smoothDeltaTime);
            leftFriction.stiffness = Mathf.Lerp(leftFriction.stiffness, wheelFriction.Stiffness, 10f * Time.smoothDeltaTime);
            
            
            var rightFriction = RightWheel.sidewaysFriction;
            rightFriction.extremumSlip = Mathf.Lerp(rightFriction.extremumSlip, wheelFriction.ExtremumSlip, 10f * Time.smoothDeltaTime);
            rightFriction.extremumValue = Mathf.Lerp(rightFriction.extremumValue, wheelFriction.ExtremumValue, 10f * Time.smoothDeltaTime);
            rightFriction.asymptoteSlip = Mathf.Lerp(rightFriction.asymptoteSlip, wheelFriction.AsymptoteSlip, 10f * Time.smoothDeltaTime);
            rightFriction.asymptoteValue = Mathf.Lerp(rightFriction.asymptoteValue, wheelFriction.AsymptoteValue, 10f * Time.smoothDeltaTime);
            rightFriction.stiffness = Mathf.Lerp(rightFriction.stiffness, wheelFriction.Stiffness, 10f * Time.smoothDeltaTime);

            LeftWheel.sidewaysFriction = leftFriction;
            RightWheel.sidewaysFriction = rightFriction;
        }

        public void SetSteerAngle(float steerAngle)
        {
            LeftWheel.steerAngle = Mathf.SmoothDamp(LeftWheel.steerAngle, steerAngle, ref m_leftSteerRef, 0.2f);
            RightWheel.steerAngle = Mathf.SmoothDamp(RightWheel.steerAngle, steerAngle, ref m_rightSteerRef, 0.2f);
        }

        public void SetMotorTorque(float torque)
        {
            LeftWheel.motorTorque = torque;
            RightWheel.motorTorque = torque;
        }

        public bool IsGrounded()
        {
            return LeftWheel.isGrounded &&
                   RightWheel.isGrounded;
        } 
    }

    [Serializable]
    public class RuntimeInformation
    {
        public float CurrentSpeed;
        public Vector3 CurrentVelocity;
        public Transform Transform;
        public RaycastHit RaycastHit;
        public Vector3 GroundNormal;
        public bool IsDrifting;
        public bool CanHop = true;
        public DriftDirection DriftDirection = DriftDirection.None;

        public bool FirstStageDrift;
        public bool SecondStageDrift;
        public bool ThirdStageDrift;
        public int DriftMode = 0;

        // drift direction -1 is left, 1 is right
        public float DriftPower;

        public bool IsBoosting;
        public float BoostPower;

        public bool TransitionToAntiGravOver;
        public bool TransitionToNormalOver;

        public KartMode KartMode = KartMode.Normal;

        public AxleInfo[] AxleInfos;

        public bool CanSwitchModes = true;
        public float DriftTimer;

        [FormerlySerializedAs("CountCount")] [MaxValue(10)] public int CoinCount;
    }

    public enum DriftDirection
    {
        None,
        Left,
        Right,
    }
    
    public class KartController : MonoBehaviour
    {
        // Used for "ShowIfAttribute" to only display members at runtime 
#pragma warning disable CS0414
        private bool m_isPlaying = false;
#pragma warning restore CS0414

        // ==================== Layers ====================
        [Title("Layers")] 
        [SerializeField] private LayerMask m_playerMask;
        
        // ==================== Tags ====================
        [Title("Tags")]
        [SerializeField] private string m_roadTag = "Road";

        // ==================== World Information ====================
        [Title("World Information")]
        [SerializeField] private float m_gravity = 11.25f;
        
        // ==================== Camera Specs ====================
        [Title("Camera Specs")]
        [SerializeField] private bool m_isFirstPerson = false;
        [SerializeField, Required] private Transform m_fpCameraPositionTransform;
        [SerializeField, Required] private Transform m_fpCameraLookAtForward;
        [SerializeField, Required] private Transform m_fpCameraLookAtRight;
        [SerializeField, Required] private Transform m_fpCameraLookAtLeft;
        
        [SerializeField] private float m_normalCameraFollowSmoothing = 0.2f;
        [SerializeField] private float m_boostCameraFollowSmoothing = 0.05f;
        [SerializeField] private float m_cameraLookAtSmoothing = 0.8f;
        [SerializeField] private float m_baseFOV = 60f;
        [SerializeField] private float m_boostFOV = 80f;
        [SerializeField] private Vector3 m_baseCamPosition;
        [SerializeField] private Vector3 m_baseCamRotation;
        [SerializeField] private Vector3 m_driftingCamPosition;
        [SerializeField] private Vector3 m_driftingCamRotation;
        
        // ==================== Kart Specs ====================
        [Title("Kart Specs")] 
        [SerializeField] private float m_stageOneBoostDuration = 0.85f;
        [SerializeField] private float m_stageTwoBoostDuration = 1f;
        [SerializeField] private float m_stageThreeBoostDuration = 1.5f;
        
        [SerializeField, ShowIf("m_isPlaying")] private Color m_currentParticleColor;
        [SerializeField] private Color[] m_driftColors = new Color[3];
        [SerializeField, Range(0f, 5000f)] private float m_kartMass = 1500f;
        [SerializeField, Range(0f, 100f)] private float m_maxVelocity = 30f;
        [SerializeField, Range(0f, 100f)] private float m_maxVelocityWithCoins = 30f;
        [SerializeField, Range(0f, 100f)] private float m_maxRoadVelocity = 30f;
        [SerializeField, Range(0f, 50f)] private float m_maxOffRoadVelocity = 10f;
        [SerializeField, Range(0f, 50f)] private float m_maxBoostVelocity = 50f;
        [SerializeField, Range(0f, 50f)] private float m_driftVelocityThreshold = 20f;
        [SerializeField] private float m_maxMotorTorque = 1000f;
        [SerializeField] private float m_maxSteeringAngle = 50f;
        [SerializeField] private float m_steerSmoothing = 0.2f;
        [SerializeField] private float m_hopForce = 3f;
        [SerializeField] private float m_hopCooldown = 0.85f;
        private float m_medRpm = 0f;
        private float m_velocityMagnitude = 0f;
        [FormerlySerializedAs("m_raycastPosition")] [SerializeField, Required] private Transform m_raycastPositionTransform;
        
        // [SerializeField] private float m_rearBaseExtremumValue = 2f;
        // [SerializeField] private float m_rearDriftExtremumValue = 0.5f;
        // [SerializeField] private float m_baseStiffness = 0.6f;
        // [SerializeField] private float m_driftStiffness = 1f;
        
        // ==================== Required Components ====================
        [Title("Required Components")] 
        // [SerializeField] private KartAnimator m_kartAnimator;
        [SerializeField] private AudioController m_audioController;
        [SerializeField, Required] private Transform m_cameraPositionTransform;
        [SerializeField, Required] private Transform m_cameraLookBehindPositionTransform;
        [SerializeField, Required] private Transform m_cameraLookAtTransform;
        [SerializeField, Required] private Camera m_camera;
        [SerializeField, Required] private AxleInfo[] m_axleInfos = new AxleInfo[2];
        
        // [SerializeField, Required] private Transform[] m_wheelObjects = new Transform[4];
        [ShowIf("m_isPlaying")] private Rigidbody m_rigidbody;
        [ShowIf("m_isPlaying")] private KartStabilization m_kartStabilization;
        public InputController InputController;
        
        // ==================== Particles ====================
        [Title("Particles")]
        [SerializeField, Required] private ParticleSystem[] m_rearDustParticles;
        
        [SerializeField, Required] private Transform m_dustDriftParticlesLeft;
        [SerializeField, Required] private Transform m_dustDriftParticlesRight;
        
        [SerializeField, Required] private Transform m_driftSparkParticlesLeft;
        [SerializeField, Required] private Transform m_driftSparkParticlesRight;
        [SerializeField, Required] private Transform m_driftBumpParticles;
        [SerializeField, ShowIf("m_isPlaying")] private ParticleSystem[] m_primaryParticles;
        [SerializeField, ShowIf("m_isPlaying")] private ParticleSystem[] m_secondaryParticles;

        [SerializeField] private Transform m_boostingParticlesTransform;
        [SerializeField] private Transform m_boostBurstParticlesTransform;
        
        [SerializeField, ShowIf("m_isPlaying")] private ParticleSystem[] m_boostingParticles; 
        [SerializeField, ShowIf("m_isPlaying")] private ParticleSystem[] m_boostBurstParticles; 

        [SerializeField, Required] private ParticleSystem m_speedLines;
        // ==================== Lights ====================
        [Title("Lights")] 
        [SerializeField, Required] private Light[] m_antiGravityLights;
        
        // ==================== Runtime Information ====================
        [Title("Runtime Information")]
        [SerializeField, ShowIf("m_isPlaying")] public RuntimeInformation RuntimeInfo;

        // ==================== Smooth Damp References ====================
        private Vector3 m_maxVelocityRef = Vector3.zero;
        private Vector3 m_cameraFollowVelocityRef = Vector3.zero;
        private Vector3 m_cameraRotationVelocityRef = Vector3.zero;
        
        // ==================== Misc ====================
        private const int c_frontWheelsIndex = 0;
        private const int c_rearWheelsIndex = 1;

        private void Awake()
        {
            m_isPlaying = true;
            m_audioController = GetComponent<AudioController>();
            m_kartStabilization = GetComponent<KartStabilization>();
            InputController = new InputController();
            SetupRigidbody();
            SetupRuntimeInformation();
            m_dustDriftParticlesLeft.gameObject.Disable();
            m_dustDriftParticlesRight.gameObject.Disable();
            
            // Just setup to be exactly on the target position
            m_camera.transform.position = m_cameraPositionTransform.position;

            var primaryLeft = m_driftSparkParticlesLeft.GetComponentsInChildren<ParticleSystem>().ToList();
            var primaryRight = m_driftSparkParticlesRight.GetComponentsInChildren<ParticleSystem>();
            primaryLeft.AddRange(primaryRight);
            m_primaryParticles = primaryLeft.ToArray();
            m_secondaryParticles = m_driftBumpParticles.GetComponentsInChildren<ParticleSystem>();
            m_speedLines.Stop();

            m_boostingParticles = m_boostingParticlesTransform.GetComponentsInChildren<ParticleSystem>();
            m_boostBurstParticles = m_boostBurstParticlesTransform.GetComponentsInChildren<ParticleSystem>();
            foreach (var boost in m_boostingParticles)
            {
                boost.Stop();
            }

            foreach (var burst in m_boostBurstParticles)
            {
                burst.Stop();
            }

            RuntimeInfo.AxleInfos = m_axleInfos;

            foreach (var l in m_antiGravityLights)
            {
                l.gameObject.Disable();
            }
        }

        private void Update()
        {
            var steering = InputController.GetAxisRaw(InputAction.Steering);
            AckermannSteering(steering * m_maxSteeringAngle);

            if (RuntimeInfo.IsDrifting)
            {
                Debug.Log(RuntimeInfo.DriftDirection);
            }
            
            if (RuntimeInfo.CurrentSpeed > 0f)
            {
                foreach (var dustParticle in m_rearDustParticles)
                {
                    dustParticle.Play();
                }
            }
            else
            {
                foreach (var dustParticle in m_rearDustParticles)
                {
                    dustParticle.Stop();
                }
            }
            if (RuntimeInfo.IsDrifting)
            {
                RuntimeInfo.DriftTimer += Time.deltaTime;
            }
            else
            {
                RuntimeInfo.DriftTimer = 0f;
            }
            
            switch (RuntimeInfo.KartMode)
            {
                case KartMode.AntiGravity:
                    m_gravity = 15f;
                    m_axleInfos[0].LeftWheel.suspensionDistance = 0.35f;
                    m_axleInfos[0].RightWheel.suspensionDistance = 0.35f;
                    m_axleInfos[1].LeftWheel.suspensionDistance = 0.35f;
                    m_axleInfos[1].RightWheel.suspensionDistance = 0.35f; // 0.35f
                    EnableAntiGravLights();
                    break;
                case KartMode.InAir:
                case KartMode.Normal:
                default:
                    m_gravity = 12f;
                    m_axleInfos[0].LeftWheel.suspensionDistance = 0.3f; // 0.25f
                    m_axleInfos[0].RightWheel.suspensionDistance = 0.3f;
                    m_axleInfos[1].LeftWheel.suspensionDistance = 0.3f;
                    m_axleInfos[1].RightWheel.suspensionDistance = 0.3f;
                    DisableAntiGravLights();
                    break;
            }
            
            if (RuntimeInfo.IsDrifting)
            {
                var powerControl = (RuntimeInfo.DriftDirection == DriftDirection.Right)
                    ? steering.Remap(-1f, 1f, 0.2f, 1f)
                    : steering.Remap(-1f, 1f, 1f, 0.2f);

                RuntimeInfo.DriftPower += powerControl;
                ColorDrift();
            }

            if (RuntimeInfo.IsBoosting)
            {
                if (!m_speedLines.isPlaying)
                {
                    m_speedLines.Play();
                }

                foreach (var boostParticles in m_boostingParticles)
                {
                    boostParticles.Play();
                }
                
                m_camera.DOFieldOfView(m_boostFOV, 0.5f);
                m_camera.DOShakePosition(0.1f, 0.02f);
            }
            else
            {
                if (m_speedLines.isPlaying)
                {
                    m_speedLines.Stop();
                }

                foreach (var boostParticles in m_boostingParticles)
                {
                    boostParticles.Stop();
                }

                foreach (var burstParticles in m_boostBurstParticles)
                {
                    burstParticles.Stop();
                }
                
                m_camera.DOFieldOfView(m_baseFOV, 0.5f);
            }
            
            // SetCameraPositionAndRotation();
        }

        private void FixedUpdate()
        {
            // Debug.Log(RuntimeInfo.GroundNormal);
            var cachedTransform = RuntimeInfo.Transform;
            var currentPosition = cachedTransform.position;
            var currentRotation = cachedTransform.rotation;
            var currentRelativeUp = cachedTransform.up;
            var currentRelativeRight = cachedTransform.right;
            RuntimeInfo.CurrentVelocity = cachedTransform.InverseTransformDirection(m_rigidbody.velocity);

            RuntimeInfo.CurrentSpeed = RuntimeInfo.CurrentVelocity.z; 

            // var raycastPosition = currentPosition;
            // raycastPosition.y += 1;
            var raycastPosition = m_raycastPositionTransform.position;
            Physics.Raycast(raycastPosition, -currentRelativeUp, out RuntimeInfo.RaycastHit);
            
            Debug.DrawRay(raycastPosition, -currentRelativeUp, Color.red, 0.1f);
            SetGroundNormal();
            TickGravity();
            Hop();
            
            // var steering = InputController.GetAxisRaw(InputAction.Steering) * m_maxSteeringAngle;
            // var motor = InputController.GetThrottle() * m_maxMotorTorque;

            var throttle = InputController.GetThrottle();
            
            m_medRpm = (m_axleInfos[0].LeftWheel.rpm + m_axleInfos[0].RightWheel.rpm +
                        m_axleInfos[1].LeftWheel.rpm + m_axleInfos[1].RightWheel.rpm) / 4f;
            
            m_velocityMagnitude = m_rigidbody.velocity.magnitude;
            
            foreach (var axleInfo in m_axleInfos)
            { 
                SetDriftStatus(axleInfo);
                SetWheelFriction(axleInfo);

                /*if (axleInfo.Steering)
                {
                    // axleInfo.SetSteerAngle(steering);
                }*/

                if (axleInfo.Motor)
                {
                    // axleInfo.SetMotorTorque(motor);
                    if (m_medRpm > 0f)
                    {
                        var torque = throttle * m_rigidbody.mass * 4f;
                        axleInfo.LeftWheel.motorTorque = torque;
                        axleInfo.RightWheel.motorTorque = torque;
                    }
                    else
                    {
                        var torque = throttle * m_rigidbody.mass * 1.5f;
                        axleInfo.LeftWheel.motorTorque = torque;
                        axleInfo.RightWheel.motorTorque = torque;
                    }
                }

                if (axleInfo.IsRear)
                {
                    if (Math.Abs(axleInfo.LeftWheel.rpm) > 10000f && Math.Abs(axleInfo.RightWheel.rpm) > 10000f)
                    {
                        axleInfo.LeftWheel.motorTorque = 0f;
                        axleInfo.RightWheel.motorTorque = 0f;
                        axleInfo.LeftWheel.brakeTorque = m_rigidbody.mass * 5f;
                        axleInfo.RightWheel.brakeTorque = m_rigidbody.mass * 5f;
                    }
                }

                if (m_velocityMagnitude < 1f && Math.Abs(throttle) < 0.1f)
                {
                    var brakeTorque = m_rigidbody.mass * 2f;
                    axleInfo.LeftWheel.brakeTorque = brakeTorque;
                    axleInfo.RightWheel.brakeTorque = brakeTorque;
                }
                else
                {
                    axleInfo.LeftWheel.brakeTorque = 0f;
                    axleInfo.RightWheel.brakeTorque = 0f;
                }

                /*if (RuntimeInfo.KartMode is KartMode.Normal or KartMode.InAir)
                {
                    AnimateWheelObjects(axleInfo);
                }*/
            }

            SetDriftDirection(RuntimeInfo.CurrentVelocity);
            SetDustParticles();

            // cachedTransform.up = RuntimeInfo.GroundNormal;
            
            // AlignToGround();
            
            if (!IsGrounded())
            {
                // HandleAirRotation(currentRelativeUp);
                Stabilize();
                // m_kartStabilization.Stabilize(m_rigidbody, RuntimeInfo.RaycastHit);
            }

            if (RuntimeInfo.IsBoosting)
            {
                m_rigidbody.AddForce(cachedTransform.forward * RuntimeInfo.BoostPower, ForceMode.VelocityChange);
            }
            
            ClampVelocity();
        }

        private void AckermannSteering(float steerAngle)
        {
            var front = m_axleInfos[0];
            var rear = m_axleInfos[1];
            rear.LeftWheel.steerAngle = 0f;
            rear.RightWheel.steerAngle = 0f;
            var halfVelocity = m_maxVelocity * 0.5f;
            if (m_velocityMagnitude >= halfVelocity && !RuntimeInfo.IsDrifting)
            {
                steerAngle *= 0.7f;
            }

            var axleSeparation = (front.LeftWheel.transform.position - rear.LeftWheel.transform.position).magnitude;
            var axleWidth = (front.LeftWheel.transform.position - front.RightWheel.transform.position).magnitude;
            var frontRightRadius = axleSeparation / Math.Sin(Math.Abs(front.RightWheel.steerAngle));
            var frontLeftRadius = axleSeparation / Math.Sin(Math.Abs(front.LeftWheel.steerAngle));

            var farAngle = AckermannUtilities.GetSecondaryAngle(steerAngle, axleSeparation, axleWidth);

            var smoothing = 10f * Time.deltaTime;
            if (steerAngle < 0f)
            {
                front.LeftWheel.steerAngle = Mathf.Lerp(front.LeftWheel.steerAngle, steerAngle, smoothing);
                front.RightWheel.steerAngle = Mathf.Lerp(front.RightWheel.steerAngle, farAngle, smoothing);
            }
            else
            {
                front.LeftWheel.steerAngle = Mathf.Lerp(front.LeftWheel.steerAngle, farAngle, smoothing);
                front.RightWheel.steerAngle = Mathf.Lerp(front.RightWheel.steerAngle, steerAngle, smoothing);
            }
        }

        private void Stabilize()
        {
            var axisFromRotate = Vector3.Cross(transform.up, RuntimeInfo.GroundNormal);
            var torqueForce = axisFromRotate.normalized * (axisFromRotate.magnitude * 50f);
            torqueForce.x *= 0.4f;
            torqueForce -= m_rigidbody.angularVelocity; // 0.02f
            m_rigidbody.AddTorque(torqueForce * (m_rigidbody.mass * 0.05f), ForceMode.Impulse);
     
            if (m_velocityMagnitude > 1.0f)
            {
                HandleAirRotation(transform.up);
            }
        }

        private void AlignToGround()
        {
            var body = transform;
            var gravityUp = RuntimeInfo.GroundNormal;
            var bodyUp = body.up;
            var bodyRotation = body.rotation;
            var targetRotation = Quaternion.FromToRotation(bodyUp, gravityUp) * bodyRotation;
            body.rotation = Quaternion.Slerp(bodyRotation, targetRotation, 50f * Time.deltaTime);
        }

        private void LateUpdate()
        {
            TickCameraFollow();
        }

        private void SetCameraPositionAndRotation()
        {
            var cameraTransform = m_cameraPositionTransform;
            var cameraPosition = cameraTransform.localPosition;
            var cameraRotation = cameraTransform.localRotation;
            var smoothing = 10f * Time.deltaTime;
            if (RuntimeInfo.IsDrifting)
            {
                cameraTransform.localPosition = Vector3.Lerp(cameraPosition, m_driftingCamPosition, smoothing);
                cameraTransform.localRotation = Quaternion.Slerp(cameraRotation, Quaternion.Euler(m_driftingCamRotation), smoothing);
            }
            else
            {
                cameraTransform.localPosition = Vector3.Lerp(cameraPosition, m_baseCamPosition, smoothing);
                cameraTransform.localRotation = Quaternion.Slerp(cameraRotation, Quaternion.Euler(m_baseCamRotation), smoothing);
            }
        }

        private void ColorDrift()
        {
            if (!RuntimeInfo.FirstStageDrift)
            {
                m_currentParticleColor = Color.clear;
            }

            var driftPower = RuntimeInfo.DriftPower;
            if (driftPower is > 30f and < 60f && !RuntimeInfo.FirstStageDrift)
            {
                RuntimeInfo.FirstStageDrift = true;
                m_currentParticleColor = m_driftColors[0];
                RuntimeInfo.DriftMode = 1;
                
                //m_audioController.PlayDriftSparks();
                PlayFlashParticle(m_currentParticleColor);
            }

            if (driftPower is > 60f and < 100f && !RuntimeInfo.SecondStageDrift)
            {
                RuntimeInfo.SecondStageDrift = true;
                m_currentParticleColor = m_driftColors[1];
                RuntimeInfo.DriftMode = 2;
                
                //m_audioController.PlayDriftSparks();
                PlayFlashParticle(m_currentParticleColor);
            }

            if (driftPower > 120f && !RuntimeInfo.ThirdStageDrift)
            {
                RuntimeInfo.ThirdStageDrift = true;
                m_currentParticleColor = m_driftColors[2];
                RuntimeInfo.DriftMode = 3;
                
                //m_audioController.PlayDriftSparks();
                PlayFlashParticle(m_currentParticleColor);
            }
            
            foreach (var primaryParticle in m_primaryParticles)
            {
                var main = primaryParticle.main;
                main.startColor = m_currentParticleColor;
            }

            foreach (var secondaryParticle in m_secondaryParticles)
            {
                var main = secondaryParticle.main;
                main.startColor = m_currentParticleColor;
            }
        }

        private void PlayFlashParticle(Color color)
        {
            foreach (var particle in m_secondaryParticles)
            {
                var main = particle.main;
                main.startColor = color;
                particle.Play();
            }
        }

        private void SetDustParticles()
        {
            switch (RuntimeInfo.DriftDirection)
            {
                case DriftDirection.Left:
                    m_dustDriftParticlesLeft.gameObject.Enable();
                    m_dustDriftParticlesRight.gameObject.Disable();
                    break;
                case DriftDirection.Right:
                    m_dustDriftParticlesRight.gameObject.Enable();
                    m_dustDriftParticlesLeft.gameObject.Disable();
                    break;
                default:
                case DriftDirection.None:
                    m_dustDriftParticlesLeft.gameObject.Disable();
                    m_dustDriftParticlesRight.gameObject.Disable();
                    break;
            }
        }

        private void SetDriftDirection(Vector3 currentVelocity)
        {
            var xVelocity = currentVelocity.x;
            if (RuntimeInfo.IsDrifting && m_axleInfos[c_rearWheelsIndex].IsGrounded())
            {
                if (xVelocity < 0f)
                {
                    RuntimeInfo.DriftDirection = DriftDirection.Right;
                }
                else if (xVelocity > 0f)
                {
                    RuntimeInfo.DriftDirection = DriftDirection.Left;
                }
            }
            else
            {
                RuntimeInfo.DriftDirection = DriftDirection.None;
            }
        }

        private void TickGravity()
        {
            if (RuntimeInfo.RaycastHit.transform is not null)
            {
                m_rigidbody.AddForce(RuntimeInfo.GroundNormal * -m_gravity, ForceMode.Acceleration);
            }
            else
            {
                m_rigidbody.AddForce(Vector3.up * -m_gravity, ForceMode.Acceleration);
            }
        }

        private void ClampVelocity()
        {
            var maxVelocity = m_maxVelocity;
            if (RuntimeInfo.CoinCount > 0)
            {
                maxVelocity += RuntimeInfo.CoinCount * 0.5f;
            }
            
            var currentVelocity = m_rigidbody.velocity;
            // On road velocity will always be the highest available velocity
            currentVelocity = Vector3.ClampMagnitude(currentVelocity, maxVelocity);
            
            // Since off road velocity is lower, I can simplify this to one if check
            if (!IsOnRoad() && IsGrounded() && !RuntimeInfo.IsBoosting)
            {
                var targetVelocity = Vector3.ClampMagnitude(currentVelocity, m_maxOffRoadVelocity);
                currentVelocity = Vector3.SmoothDamp(currentVelocity, targetVelocity, ref m_maxVelocityRef, 0.2f);
            }
            else if (RuntimeInfo.IsBoosting)
            {
                currentVelocity = Vector3.ClampMagnitude(currentVelocity, m_maxBoostVelocity);
            }
            else if (!RuntimeInfo.IsBoosting && InputController.GetThrottle() == 0f)
            {
                currentVelocity = Vector3.SmoothDamp(currentVelocity, Vector3.zero, ref m_maxVelocityRef, 0.6f);
            }

            m_rigidbody.velocity = currentVelocity;
        }

        private void HandleAirRotation(Vector3 up)
        {
            m_rigidbody.AddRelativeTorque(up * (InputController.GetAxisRaw(InputAction.Steering) * 0.5f), ForceMode.VelocityChange);
        }
        
        private void Hop()
        {
            if (InputController.GetButtonDown(InputAction.Drift) && RuntimeInfo.CanHop && IsGrounded())
            {
                if (RuntimeInfo.KartMode == KartMode.AntiGravity)
                {
                    m_rigidbody.AddForce(RuntimeInfo.GroundNormal * (m_hopForce * 2f), ForceMode.VelocityChange);
                }
                else
                {
                    m_rigidbody.AddForce(RuntimeInfo.GroundNormal * m_hopForce, ForceMode.VelocityChange);
                }

                RuntimeInfo.CanHop = false;
                Invoke(nameof(ResetCanHop), m_hopCooldown);
            }
        }

        private void TickCameraFollow()
        {
            var cameraTransform = m_camera.transform;
            var smoothing = m_normalCameraFollowSmoothing;
            if (RuntimeInfo.IsBoosting)
            {
                smoothing = m_boostCameraFollowSmoothing;
            }

            if (m_isFirstPerson)
            {
                // drifting left look at right
                var targetPosition = m_fpCameraPositionTransform.position;
                // cameraTransform.DOMove(targetPosition, smoothing * 0.02f);
                cameraTransform.position = targetPosition;
                if (RuntimeInfo.IsDrifting)
                {
                    switch (RuntimeInfo.DriftDirection)
                    {
                        case DriftDirection.Left:
                            cameraTransform.DOLookAt(m_fpCameraLookAtRight.position, m_cameraLookAtSmoothing, up: transform.up);
                            break;
                        case DriftDirection.Right:
                            cameraTransform.DOLookAt(m_fpCameraLookAtLeft.position, m_cameraLookAtSmoothing, up: transform.up);
                            break;
                    }
                }
                else
                {
                    cameraTransform.DOLookAt(m_fpCameraLookAtForward.position, m_cameraLookAtSmoothing, up: transform.up);
                }
                
            }
            else
            {
                var targetPosition = m_cameraPositionTransform.position;
                
                if (InputController.GetButtonHeld(InputAction.LookBehind))
                {
                    targetPosition = m_cameraLookBehindPositionTransform.position;
                }
            
                cameraTransform.DOMove(targetPosition, smoothing);
                cameraTransform.DOLookAt(m_cameraLookAtTransform.position, m_cameraLookAtSmoothing, up: transform.up);
            }
        }

        private void SetGroundNormal()
        {
            // "Usually" we will get a hit
            if (RuntimeInfo.RaycastHit.transform is not null)
            {
                RuntimeInfo.GroundNormal = RuntimeInfo.RaycastHit.normal;
            }
            else // but on the rare off chance something freaks out, we want world up 
            {
                RuntimeInfo.GroundNormal = Vector3.up;
            }
        }
        
        private void SetDriftStatus(AxleInfo axleInfo)
        {
            if (InputController.GetButtonHeld(InputAction.Drift) && IsOnRoad())
            {
                // m_runtimeInfo.IsDrifting = true;
                // We only care about the slip of the rear wheels
                if (axleInfo.IsRear && (axleInfo.LeftWheel.GetGroundHit(out var hit) || 
                                        axleInfo.RightWheel.GetGroundHit(out hit)))
                {
                    if (hit.sidewaysSlip is > 0.2f or < -0.2f)
                    {
                        if (RuntimeInfo.CurrentSpeed >= m_driftVelocityThreshold)
                        {
                            if (!RuntimeInfo.IsDrifting)
                            {
                                foreach (var particle in m_primaryParticles)
                                {
                                    var main = particle.main;
                                    main.startColor = Color.clear;
                                    particle.Play();
                                }
                            }
                        
                            RuntimeInfo.IsDrifting = true;
                        }
                    }
                    else if (hit.sidewaysSlip is < 0.05f or > -0.05f)
                    {
                        if (RuntimeInfo.DriftMode > 0 && !RuntimeInfo.IsBoosting && RuntimeInfo.DriftTimer > 0.25f)
                        {
                            RuntimeInfo.BoostPower = RuntimeInfo.DriftMode * 0.3f; 
                            RuntimeInfo.IsBoosting = true;
                            foreach (var burst in m_boostBurstParticles)
                            {
                                burst.Play();
                            }

                            var boostDuration = 0f;
                            if (RuntimeInfo.FirstStageDrift)
                            {
                                boostDuration = m_stageOneBoostDuration;
                            }
                            if (RuntimeInfo.SecondStageDrift)
                            {
                                boostDuration = m_stageTwoBoostDuration;
                            }
                            if (RuntimeInfo.ThirdStageDrift)
                            {
                                boostDuration = m_stageThreeBoostDuration;
                            }
                            Invoke(nameof(ResetIsBoosting), boostDuration);
                        }
                        
                        // TODO: use boost first
                        RuntimeInfo.IsDrifting = false;
                        DisablePrimaryParticles();
                        
                        ResetDrift();
                    }
                }
                
                if (axleInfo.IsFront && RuntimeInfo.IsDrifting)
                {
                    axleInfo.Motor = true;
                }
            }
            else
            {
                RuntimeInfo.IsDrifting = false;
                if (RuntimeInfo.DriftMode > 0 && !RuntimeInfo.IsBoosting && RuntimeInfo.DriftTimer > 0.25f)
                {
                    RuntimeInfo.BoostPower = RuntimeInfo.DriftMode * 0.3f;
                    RuntimeInfo.IsBoosting = true;
                    foreach (var burst in m_boostBurstParticles)
                    {
                        burst.Play();
                    }
                    
                    var boostDuration = 0f;
                    if (RuntimeInfo.FirstStageDrift)
                    {
                        boostDuration = m_stageOneBoostDuration;
                    }
                    if (RuntimeInfo.SecondStageDrift)
                    {
                        boostDuration = m_stageTwoBoostDuration;
                    }
                    if (RuntimeInfo.ThirdStageDrift)
                    {
                        boostDuration = m_stageThreeBoostDuration;
                    }
                    Invoke(nameof(ResetIsBoosting), boostDuration);
                }
                
                DisablePrimaryParticles();
                ResetDrift();

                if (axleInfo.IsFront)
                {
                    axleInfo.Motor = false;
                    axleInfo.LeftWheel.motorTorque = 0f;
                    axleInfo.RightWheel.motorTorque = 0f;
                    
                    /*var leftFriction = axleInfo.LeftWheel.sidewaysFriction;
                    leftFriction.stiffness = 1f;
                    var rightFriction = axleInfo.RightWheel.sidewaysFriction;
                    rightFriction.stiffness = 1f;

                    axleInfo.LeftWheel.sidewaysFriction = leftFriction;
                    axleInfo.RightWheel.sidewaysFriction = rightFriction;
                */
                }
            }
        }

        /*private void AnimateAntiGravWheelObjects(AxleInfo axleInfo)
        {
            if (!RuntimeInfo.AntiGravTransitionOver)
            {
                // left wheel z rotates to -90
                // right wheel z rotates to 90
                var leftRotation = axleInfo.LeftWheelMesh.localEulerAngles;
                // leftRotation.y = 0f;
                leftRotation.z = 90f;
                // axleInfo.LeftWheelMesh.localEulerAngles = leftRotation;
                axleInfo.LeftWheelMesh.DOLocalRotate(leftRotation, 0.35f).onComplete += () =>
                {
                    RuntimeInfo.AntiGravTransitionOver = true;
                    var angles = axleInfo.LeftWheelMesh.localEulerAngles;
                    angles.y = 0f;
                    axleInfo.LeftWheelMesh.localEulerAngles = angles;
                };
                
                var rightRotation = axleInfo.RightWheelMesh.localEulerAngles;
                // rightRotation.y = 0f;
                rightRotation.z = -90f;
                // axleInfo.RightWheelMesh.localEulerAngles = rightRotation;
                axleInfo.RightWheelMesh.DOLocalRotate(rightRotation, 0.35f).onComplete += () =>
                {
                    RuntimeInfo.AntiGravTransitionOver = true;
                    var angles = axleInfo.RightWheelMesh.localEulerAngles;
                    angles.y = 0f;
                    axleInfo.RightWheelMesh.localEulerAngles = angles;
                };
            }
            
            axleInfo.LeftWheelMesh.Rotate(RuntimeInfo.Transform.up, 150f * Time.deltaTime, Space.World);
            axleInfo.RightWheelMesh.Rotate(RuntimeInfo.Transform.up, -150f * Time.deltaTime, Space.World);
        }*/

        private void DisablePrimaryParticles()
        {
            foreach (var particle in m_primaryParticles)
            {
                particle.Stop();
            }
        }

        private void EnablePrimaryParticles()
        {
            foreach (var particle in m_primaryParticles)
            {
                particle.Play();
            }
        }

        private void EnableAntiGravLights()
        {
            foreach (var l in m_antiGravityLights)
            {
                l.gameObject.Enable();
            }
        }
        
        private void DisableAntiGravLights()
        {
            foreach (var l in m_antiGravityLights)
            {
                l.gameObject.Disable();
            }
        }


        private void ResetDrift()
        {
            RuntimeInfo.DriftPower = 0f;
            RuntimeInfo.DriftMode = 0;
            RuntimeInfo.FirstStageDrift = false;
            RuntimeInfo.SecondStageDrift = false;
            RuntimeInfo.ThirdStageDrift = false;
        }

        public void CollectCoin(int quantity)
        {
            RuntimeInfo.CoinCount += quantity;
            RuntimeInfo.CoinCount = Math.Clamp(RuntimeInfo.CoinCount, 0, 10);
        }
        
        private void SetWheelFriction(AxleInfo axleInfo)
        {
            var smoothing = 10f * Time.deltaTime;
            if (RuntimeInfo.IsDrifting)
            {
                // axleInfo.SetForwardsFriction(axleInfo.DriftingForwardsFriction);
                // axleInfo.SetSidewaysFriction(axleInfo.DriftingSidewaysFriction);
                
                // 0.6f baseline
                var leftFriction = axleInfo.LeftWheel.sidewaysFriction;
                // leftFriction.stiffness = Mathf.Lerp(leftFriction.stiffness, 0.5f, 10f * Time.deltaTime);
                
                var rightFriction = axleInfo.RightWheel.sidewaysFriction;
                // rightFriction.stiffness = Mathf.Lerp(rightFriction.stiffness, 0.5f, 10f * Time.deltaTime);
                // rightFriction.stiffness = Mathf.Lerp(rightFriction.stiffness, (0.2) 0.6f, 10f * Time.deltaTime);

                if (axleInfo.IsFront)
                {
                    leftFriction.stiffness = Mathf.Lerp(leftFriction.stiffness, 0.5f, smoothing);
                    rightFriction.stiffness = Mathf.Lerp(rightFriction.stiffness, 0.5f, smoothing);
                }
                if (axleInfo.IsRear)
                {
                    leftFriction.extremumValue = 1.5f;
                    rightFriction.extremumValue = 1.5f;
                    leftFriction.stiffness = Mathf.Lerp(leftFriction.stiffness, 0.75f, smoothing);
                    rightFriction.stiffness = Mathf.Lerp(rightFriction.stiffness, 0.75f, smoothing);
                    // rightFriction.extremumValue = 1.5f;
                }
                    
                axleInfo.LeftWheel.sidewaysFriction = leftFriction;
                axleInfo.RightWheel.sidewaysFriction = rightFriction;
            }
            else
            {
                var leftFriction = axleInfo.LeftWheel.sidewaysFriction;
                // leftFriction.stiffness = Mathf.Lerp(leftFriction.stiffness, 1.5f, 8f * Time.deltaTime);
                        
                var rightFriction = axleInfo.RightWheel.sidewaysFriction;
                // rightFriction.stiffness = Mathf.Lerp(rightFriction.stiffness, 1.5f, 8f * Time.deltaTime);

                if (axleInfo.IsFront)
                {
                    leftFriction.stiffness = 2f;
                    rightFriction.stiffness = 2f;
                }
                if (axleInfo.IsRear)
                {
                    leftFriction.extremumValue = 2f;
                    rightFriction.extremumValue = 2f; // 3f
                    // 2f
                    var thirdVelocity = m_maxVelocity / 3f;
                    if (m_velocityMagnitude >= thirdVelocity)
                    {
                        leftFriction.stiffness = 3.5f;
                        rightFriction.stiffness = 3.5f;
                    }
                    else
                    {
                        leftFriction.stiffness = 2f;
                        rightFriction.stiffness = 2f;
                    }
                }
                                        
                axleInfo.LeftWheel.sidewaysFriction = leftFriction;
                axleInfo.RightWheel.sidewaysFriction = rightFriction;
                
                // axleInfo.SetForwardsFriction(axleInfo.BaseForwardsFriction);
                // axleInfo.SetSidewaysFriction(axleInfo.BaseSidewaysFriction);
                
                /*if (axleInfo.IsRear)
                {
                    var leftFriction = axleInfo.LeftWheel.sidewaysFriction;
                    leftFriction.stiffness = Mathf.Lerp(leftFriction.stiffness, m_baseStiffness, 8f * Time.deltaTime);
                    // leftFriction.stiffness = Mathf.Lerp(leftFriction.stiffness, 1.5f, 8f * Time.deltaTime);
                        
                    var rightFriction = axleInfo.RightWheel.sidewaysFriction;
                    rightFriction.stiffness = Mathf.Lerp(rightFriction.stiffness, m_baseStiffness, 8f * Time.deltaTime);
                    // rightFriction.stiffness = Mathf.Lerp(rightFriction.stiffness, 1.5f, 8f * Time.deltaTime);

                    leftFriction.extremumValue = m_rearBaseExtremumValue;
                                        
                    rightFriction.extremumValue = m_rearBaseExtremumValue;
                                            
                    axleInfo.LeftWheel.sidewaysFriction = leftFriction;
                    axleInfo.RightWheel.sidewaysFriction = rightFriction;
                }*/
            }
        }

        /*private void AnimateWheelObjects(AxleInfo axleInfo)
        {
            axleInfo.LeftWheel.GetWorldPose(out var leftPosition, out var leftRotation);
            axleInfo.LeftWheelMesh.SetPositionAndRotation(leftPosition, leftRotation);

            
            axleInfo.RightWheel.GetWorldPose(out var rightPosition, out var rightRotation);
            axleInfo.RightWheelMesh.SetPositionAndRotation(rightPosition, rightRotation);

        }*/

        // On trigger exit has the best results
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

        // ==================== Checks ====================
        private bool IsGrounded()
        {
            return m_axleInfos[0].IsGrounded() && m_axleInfos[1].IsGrounded();
        }
        
        private bool IsOnRoad()
        {
            if (RuntimeInfo.RaycastHit.transform is not null)
            {
                return RuntimeInfo.RaycastHit.transform.CompareTag(m_roadTag);
            }

            return false;
        }
        // ==================== Invoke Resets ====================
        private void ResetCanSwitchModes()
        {
            RuntimeInfo.CanSwitchModes = true;
        }
        
        private void ResetCanHop()
        {
            RuntimeInfo.CanHop = true;
        }

        private void ResetIsBoosting()
        {
            RuntimeInfo.IsBoosting = false;
        }
        
        // ==================== Setup ====================
        private void SetupRuntimeInformation()
        {
            RuntimeInfo = new RuntimeInformation
            {
                Transform = transform,
            };
        }
        
        private void SetupRigidbody()
        {
            m_rigidbody = GetComponent<Rigidbody>();
            m_rigidbody.mass = m_kartMass;
            m_rigidbody.drag = 0f;
            m_rigidbody.angularDrag = 0f;
            m_rigidbody.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
            m_rigidbody.interpolation = RigidbodyInterpolation.Interpolate;
            m_rigidbody.useGravity = false;
            m_rigidbody.isKinematic = false;
        }

        private void OnDrawGizmos()
        {
            foreach (var axleInfo in m_axleInfos)
            {
                var leftTransform = axleInfo.LeftWheel.transform;
                var rightTransform = axleInfo.RightWheel.transform;
                
                var leftPos = leftTransform.position;
                var rightPos = rightTransform.position;
                var leftUp = leftTransform.up;
                var leftForward = leftTransform.forward;
                var rightUp = rightTransform.up;
                var rightForward = rightTransform.forward;
                
                var color = Gizmos.color;
                Gizmos.color = Color.magenta;
                
                leftPos.y -= 0.15f;
                rightPos.y -= 0.15f;
                
                Gizmos.DrawRay(leftPos, leftUp * axleInfo.LeftWheel.radius);
                Gizmos.DrawRay(leftPos, -leftUp * axleInfo.LeftWheel.radius);
                Gizmos.DrawRay(leftPos, leftForward * axleInfo.LeftWheel.radius);
                Gizmos.DrawRay(leftPos, -leftForward * axleInfo.LeftWheel.radius);
                
                Gizmos.DrawRay(rightPos, rightUp * axleInfo.RightWheel.radius);
                Gizmos.DrawRay(rightPos, -rightUp * axleInfo.RightWheel.radius);
                Gizmos.DrawRay(rightPos, rightForward * axleInfo.RightWheel.radius);
                Gizmos.DrawRay(rightPos, -rightForward * axleInfo.RightWheel.radius);
                
                Gizmos.color = Color.red;
                Gizmos.DrawWireSphere(leftPos, axleInfo.LeftWheel.radius);
                Gizmos.DrawWireSphere(rightPos, axleInfo.RightWheel.radius);
                Gizmos.color = color;
            }
        }
    }
}
