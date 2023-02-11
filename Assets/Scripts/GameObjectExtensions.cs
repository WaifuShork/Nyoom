using UnityEngine;

namespace Nyoom
{
    public static class GameObjectExtensions
    {
        public static void Enable(this GameObject gameObject)
        {
            if (!gameObject.IsEnabled())
            {
                gameObject.SetActive(true);
            }
        }

        public static void Disable(this GameObject gameObject)
        {
            if (gameObject.IsEnabled())
            {
                gameObject.SetActive(false);
            }
        }
        
        public static bool IsEnabled(this GameObject gameObject)
        {
            return gameObject.activeSelf;
        }
    }
}