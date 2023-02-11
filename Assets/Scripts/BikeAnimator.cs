using System;
using System.Linq;
using DG.Tweening;
using Rewired.Data.Mapping;
using Sirenix.OdinInspector;
using UnityEngine;

namespace Nyoom
{
    public class BikeAnimator : MonoBehaviour
    {
        private BikeController m_bikeController;
        [SerializeField] private Transform m_bikeMesh;
        [SerializeField] private WheelCollider[] m_wheelColliders;
        [SerializeField] private Transform[] m_wheelMeshes;
        
        [Title("Engine Vibration Specs")]
        [SerializeField, SuffixLabel("(offset)")] private Vector3 m_engineVibrationStrength = new(0f, 0.05f, 0f);
        [SerializeField, SuffixLabel("(seconds)")] private float m_engineVibrationDuration = 0.1f;

        private InputController m_inputController;
        private bool m_canShake = true;

        private void Awake()
        {
            m_inputController = new InputController();
            m_bikeController = GetComponent<BikeController>();
        }

        private void Update()
        {
            AnimateEngineVibration();
            AnimateBikeBody();

            switch (m_bikeController.RuntimeInfo.KartMode)
            {
                case KartMode.AntiGravity:
                    AnimateAntiGravWheels();
                    break;
                case KartMode.Normal:
                case KartMode.InAir:
                default:
                    AnimateNormalWheels();
                    break;
            }
        }

        private void AnimateEngineVibration()
        {
            if (m_canShake)
            {
                m_canShake = false;
                m_bikeMesh.DOShakePosition(m_engineVibrationDuration, m_engineVibrationStrength).onComplete += () =>
                {
                    m_canShake = true;
                };
            }
        }

        private void AnimateBikeBody()
        {
            var steering = m_inputController.GetAxisRaw(InputAction.Steering);
            var angles = m_bikeMesh.localEulerAngles;
            if (steering > 0f)
            {
                angles.z = -20f;
            }
            else if (steering < 0f)
            {
                angles.z = 20f;
            }
            else
            {
                angles.z = 0f;
            }

            var front = m_wheelMeshes[0].parent;
            var rear = m_wheelMeshes[1].parent;
            
            front.DOLocalRotate(angles, 0.65f);
            rear.DOLocalRotate(angles, 0.65f);
            m_bikeMesh.DOLocalRotate(angles, 0.65f);
        }

        private void AnimateAntiGravWheels()
        {
            if (!m_bikeController.RuntimeInfo.TransitionToAntiGravOver)
            {
                var frontAngles = m_wheelMeshes[0].localEulerAngles;
                frontAngles.x = 0f;
                frontAngles.y = 0f;
                frontAngles.z = 90f;
    
                m_wheelMeshes[0].DOLocalRotate(frontAngles, 0.35f).onComplete += () =>
                {
                    m_bikeController.RuntimeInfo.TransitionToAntiGravOver = true;
                    m_bikeController.RuntimeInfo.TransitionToNormalOver = false;
                };
                
                var rearAngles = m_wheelMeshes[1].localEulerAngles;
                rearAngles.x = 0f;
                rearAngles.y = 0f;
                rearAngles.z = 90f;
    
                m_wheelMeshes[1].DOLocalRotate(rearAngles, 0.35f).onComplete += () =>
                {
                    m_bikeController.RuntimeInfo.TransitionToAntiGravOver = true;
                    m_bikeController.RuntimeInfo.TransitionToNormalOver = false;
                };
            }

            var throttle = m_inputController.GetThrottle();
            if (throttle == 0f)
            {
                m_wheelMeshes[0].Rotate(transform.up, 100f * Time.deltaTime, Space.World);
                m_wheelMeshes[1].Rotate(transform.up, -100f * Time.deltaTime, Space.World);
            }
            else
            {
                m_wheelMeshes[0].Rotate(transform.up, (throttle * 300) *  Time.deltaTime, Space.World);
                m_wheelMeshes[1].Rotate(transform.up, (throttle * -300) *  Time.deltaTime, Space.World);
            }

            m_wheelColliders[0].GetWorldPose(out var frontPosition, out _);
            m_wheelMeshes[0].position = frontPosition;
            
            m_wheelColliders[1].GetWorldPose(out var rearPosition, out _);
            m_wheelMeshes[1].position = rearPosition;
        }

        private void AnimateNormalWheels()
        {
            if (!m_bikeController.RuntimeInfo.TransitionToNormalOver)
            {
                m_wheelMeshes[0].DOLocalRotate(Vector3.zero, 0.5f).onComplete += () =>
                {
                    m_bikeController.RuntimeInfo.TransitionToNormalOver = true;
                    m_bikeController.RuntimeInfo.TransitionToAntiGravOver = false;
                };
                
                m_wheelMeshes[1].DOLocalRotate(Vector3.zero, 0.5f).onComplete += () =>
                {
                    m_bikeController.RuntimeInfo.TransitionToNormalOver = true;
                    m_bikeController.RuntimeInfo.TransitionToAntiGravOver = false;
                };
            }
            
            m_wheelColliders[0].GetWorldPose(out var leftPosition, out var leftRotation);
            m_wheelMeshes[0].SetPositionAndRotation(leftPosition, leftRotation);
            
            
            m_wheelColliders[1].GetWorldPose(out var rightPosition, out var rightRotation);
            m_wheelMeshes[1].SetPositionAndRotation(rightPosition, rightRotation);
        }
    }
}