using System;
using System.Text;
using UnityEngine;

namespace Nyoom
{
    public class ConsoleToGUI : MonoBehaviour
    {
        private static string s_log = string.Empty;
        private static string s_output;
        private static string s_stack;
        private GUIStyle m_style;

        private void OnEnable()
        {
            Application.logMessageReceived += OnLogMessageReceived;
        }

        private void OnDisable()
        {
            Application.logMessageReceived -= OnLogMessageReceived;
        }

        private static void OnLogMessageReceived(string logString, string stack, LogType type)
        {
            s_output = logString;
            s_stack = stack;
            s_log = $"{s_output}\n{s_log}";
            if (s_log.Length > 5000)
            {
                s_log = s_log[..4000];
            }
        }

        private Vector3 m_scrollPosition;

        private void OnGUI()
        {
            #if !UNITY_EDITOR
            if (m_style is null)
            {
                m_style = new(GUI.skin.box)
                {
                    normal =
                    {
                        background = MakeTexture(350, 200, new Color(0f, 0f, 0f, 0.5f))
                    },
                    alignment = TextAnchor.UpperLeft,
                    padding = new RectOffset(10, 0, 0, 0)
                };
            }
            
            if (GUILayout.Button("Clear"))
            {
                s_log = string.Empty;
            }
                        
            m_scrollPosition = GUILayout.BeginScrollView(m_scrollPosition, GUILayout.Width(350), GUILayout.Height(200));
            GUILayout.Box(s_log, m_style);
            
            GUILayout.EndScrollView();
#endif
        }

        private static Texture2D MakeTexture(int width, int height, Color color)
        {
            var pix = new Color[width * height];
            for (var i = 0; i < pix.Length; i++)
            {
                pix[i] = color;
            }

            var result = new Texture2D(width, height);
            result.SetPixels(pix);
            result.Apply();
            return result;
        }
    }
}