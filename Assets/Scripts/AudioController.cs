using UnityEngine;
using UnityEngine.Serialization;

namespace Nyoom
{
    public class AudioController : MonoBehaviour
    {
        [SerializeField] private AudioSource m_kartIdle;
        [FormerlySerializedAs("m_kartRev")] [SerializeField] private AudioSource m_kartSound;
        [SerializeField] private AudioSource m_kartBump;
        
        [SerializeField] private AudioSource m_kartDrift;
        [SerializeField] private AudioSource m_kartDriftSparks;

        private KartController m_kartController;

        private void Start()
        {
            m_kartController = GetComponent<KartController>();
        }

        private void Update()
        {
            // KartSounds();

            // m_kartIdle.volume = Mathf.Lerp(m_kartIdle.volume, 1f, 0.5f * Time.deltaTime);
        }
        
        private void KartSounds()
        {
            var currentSpeed = m_kartController.RuntimeInfo.CurrentVelocity.magnitude;
            Debug.Log(currentSpeed);
            
            if (currentSpeed < 10f && currentSpeed >= -10f)
            {
                if (!m_kartIdle.isPlaying)
                {
                    m_kartIdle.Play();
                }
            }

            if (currentSpeed < -10f)
            {
                m_kartIdle.Stop();
            }

            if (currentSpeed >= 2.5f)
            {
                if (!m_kartSound.isPlaying)
                {
                    m_kartSound.Play();
                }

                var smoothing = 4f * Time.deltaTime;
                if (currentSpeed is > 5f and <= 10f)
                {
                    m_kartSound.time = Mathf.Lerp(m_kartSound.time, 0.6f, smoothing);
                    m_kartIdle.Stop();
                }
                if (currentSpeed is > 10f and < 15f)
                {
                    m_kartSound.time = Mathf.Lerp(m_kartSound.time, 1f, smoothing);
                    m_kartIdle.Stop();
                }
                else if (currentSpeed is > 15f and < 20f)
                {
                    m_kartSound.time = Mathf.Lerp(m_kartSound.time, 1.4f, smoothing);
                    m_kartIdle.Stop();
                }
                else if (currentSpeed is >= 20f and < 25f)
                {
                    m_kartSound.time = Mathf.Lerp(m_kartSound.time, 1.8f, smoothing);
                    m_kartIdle.Stop();
                } 
                else if (currentSpeed is >= 25f and < 30f)
                {
                    m_kartSound.time = Mathf.Lerp(m_kartSound.time, 2.2f, smoothing);
                    m_kartIdle.Stop();
                }
            }

            if (m_kartController.RuntimeInfo.IsDrifting)
            {
                if (!m_kartDrift.isPlaying)
                {
                    m_kartDrift.Play();
                }

                /*if (m_kartController.RuntimeInfo.DriftMode > 0)
                {
                    if (!m_kartDriftSparks.isPlaying)
                    {
                        m_kartDriftSparks.Play();
                    }
                }*/
            }
            else
            {
                if (m_kartDrift.isPlaying)
                {
                    m_kartDrift.Stop();
                }

                if (m_kartDriftSparks.isPlaying)
                {
                    m_kartDriftSparks.Stop();
                }
            }
        }

        public void PlayDriftSparks()
        {
            m_kartDriftSparks.Play();
        }
    }
}