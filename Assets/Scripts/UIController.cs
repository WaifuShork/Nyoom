using UnityEngine;
using UnityEngine.UI;

namespace Nyoom
{
    public class UIController : MonoBehaviour
    {
        [SerializeField] private Text m_coinCount;
        
        private KartController m_kartController;

        private void Awake()
        {
            m_kartController = GetComponent<KartController>();
        }

        private void Update()
        {
            m_coinCount.text = m_kartController.RuntimeInfo.CoinCount.ToString();
        }
    }
}