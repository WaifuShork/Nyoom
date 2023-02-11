using UnityEngine;

namespace Nyoom
{
    public class LookAt : MonoBehaviour
    {
        [SerializeField] private Transform m_target;

        private void Awake()
        {
            transform.LookAt(m_target, transform.up);
        }
    }
}