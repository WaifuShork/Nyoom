using System;
using DG.Tweening;
using UnityEngine;
using Sirenix.OdinInspector;

namespace Nyoom
{
    public class KartAnimator : MonoBehaviour
    {
        [Title("Required Components")]
        [SerializeField] private KartController m_kartController;
        [SerializeField, Required] private Transform m_kartTransform;
        [SerializeField, Required] private Transform m_steeringWheel;
        
        [Title("Engine Vibration Specs")]
        [SerializeField, SuffixLabel("(offset)")] private Vector3 m_engineVibrationStrength = new(0f, 0.05f, 0f);
        [SerializeField, SuffixLabel("(seconds)")] private float m_engineVibrationDuration = 0.1f;

        private Vector3 m_originalKartPosition;
        private bool m_canShake = true;
        
        private void Awake()
        {
            m_kartController = GetComponent<KartController>();
            m_originalKartPosition = m_kartTransform.localPosition;
        }
        
        private void Update()
        {
            AnimateEngineVibration();

            AnimateSteeringWheel();
            
            foreach (var axleInfo in m_kartController.RuntimeInfo.AxleInfos)
            {
                switch (m_kartController.RuntimeInfo.KartMode)
                {
                    case KartMode.AntiGravity:
                        AnimateAntiGravWheels(axleInfo);
                        break;
                    case KartMode.Normal:
                    case KartMode.InAir:
                    default:
                        AnimateNormalWheels(axleInfo);
                        break;
                }
            }
        }

        private void AnimateSteeringWheel()
        {
            var steerAngle = m_kartController.RuntimeInfo.AxleInfos[0].LeftWheel.steerAngle;
            var localAngles = m_steeringWheel.localEulerAngles;
            var newRot = Quaternion.Euler(new Vector3(localAngles.x, localAngles.y, -steerAngle));
            m_steeringWheel.localRotation = Quaternion.Slerp(m_steeringWheel.localRotation, newRot, 10f * Time.deltaTime);
        }

        private void AnimateEngineVibration()
        {
            if (m_canShake)
            {
                m_canShake = false;
                m_kartTransform.DOShakePosition(m_engineVibrationDuration, m_engineVibrationStrength).onComplete += () =>
                {
                    m_canShake = true;
                };
            }
        }

        private void AnimateAntiGravWheels(AxleInfo axleInfo)
        {
            if (!m_kartController.RuntimeInfo.TransitionToAntiGravOver)
            {
                // left wheel z rotates to -90
                // right wheel z rotates to 90
                // var leftAngles = axleInfo.LeftWheelMesh.localEulerAngles;
         
                var leftAngles = axleInfo.LeftWheelMesh.localEulerAngles;
                leftAngles.x = 0f;
                leftAngles.y = 0f;
                leftAngles.z = 90f;
                axleInfo.LeftWheelMesh.DOLocalRotate(leftAngles, 0.35f).onComplete += () =>
                {
                    m_kartController.RuntimeInfo.TransitionToAntiGravOver = true;
                    m_kartController.RuntimeInfo.TransitionToNormalOver = false;
                };
                
                var rightAngles= axleInfo.RightWheelMesh.localEulerAngles;
                rightAngles.x = 0f;
                rightAngles.y = 0f;
                rightAngles.z = -90f;
                axleInfo.RightWheelMesh.DOLocalRotate(rightAngles, 0.35f).onComplete += () =>
                {
                    m_kartController.RuntimeInfo.TransitionToAntiGravOver = true;
                    m_kartController.RuntimeInfo.TransitionToNormalOver = false;
                };
            }

            var throttle = m_kartController.InputController.GetThrottle();
            if (throttle == 0f)
            {
                axleInfo.LeftWheelMesh.Rotate(m_kartController.RuntimeInfo.Transform.up, 100f * Time.deltaTime, Space.World);
                axleInfo.RightWheelMesh.Rotate(m_kartController.RuntimeInfo.Transform.up, -100f * Time.deltaTime, Space.World);
            }
            else
            {
                axleInfo.LeftWheelMesh.Rotate(m_kartController.RuntimeInfo.Transform.up, (throttle * 300f) * Time.deltaTime, Space.World);
                axleInfo.RightWheelMesh.Rotate(m_kartController.RuntimeInfo.Transform.up, (throttle -300f) * Time.deltaTime, Space.World);
            }
            
            axleInfo.LeftWheel.GetWorldPose(out var leftPosition, out _);
            axleInfo.LeftWheelMesh.position = leftPosition;
            
            axleInfo.RightWheel.GetWorldPose(out var rightPosition, out _);
            axleInfo.RightWheelMesh.position = rightPosition;
        }

        private void AnimateNormalWheels(AxleInfo axleInfo)
        {
            if (!m_kartController.RuntimeInfo.TransitionToNormalOver)
            {
                axleInfo.LeftWheelMesh.DOLocalRotate(Vector3.zero, 0.5f).onComplete += () =>
                {
                    m_kartController.RuntimeInfo.TransitionToNormalOver = true;
                    m_kartController.RuntimeInfo.TransitionToAntiGravOver = false;
                };
                
                axleInfo.RightWheelMesh.DOLocalRotate(Vector3.zero, 0.5f).onComplete += () =>
                {
                    m_kartController.RuntimeInfo.TransitionToNormalOver = true;
                    m_kartController.RuntimeInfo.TransitionToAntiGravOver = false;
                };
            }
            
            axleInfo.LeftWheel.GetWorldPose(out var leftPosition, out var leftRotation);
            axleInfo.LeftWheelMesh.SetPositionAndRotation(leftPosition, leftRotation);
            
            axleInfo.RightWheel.GetWorldPose(out var rightPosition, out var rightRotation);
            axleInfo.RightWheelMesh.SetPositionAndRotation(rightPosition, rightRotation);
        }
    }
}