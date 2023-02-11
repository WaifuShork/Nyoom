using System;
using Rewired;

namespace Nyoom
{
    public enum InputAction
    {
        Steering,
        Forward,
        Backward,
        Drift,
        Use,
        LookBehind,
    }

    public class InputController
    {
        private readonly Player m_player;

        public InputController()
        {
            m_player = ReInput.players.GetPlayer(0);
        }
        
        public bool GetButtonDown(InputAction action)
        {
            return m_player.GetButtonDown(ToName(action));
        }

        public bool GetButtonHeld(InputAction action)
        {
            return m_player.GetButton(ToName(action));
        }

        public float GetAxis(InputAction action)
        {
            return m_player.GetAxis(ToName(action));
        }

        public float GetAxisRaw(InputAction action)
        {
            return m_player.GetAxisRaw(ToName(action));
        }

        public float GetThrottle()
        {
            return GetAxisRaw(InputAction.Forward) + GetAxisRaw(InputAction.Backward);
        }
        
        private static string ToName(InputAction action)
        {
            return action switch
            {
                InputAction.Steering => "Steer",
                InputAction.Forward => "Forward",
                InputAction.Backward => "Backward",
                InputAction.Drift => "Drift",
                InputAction.Use => "Use",
                InputAction.LookBehind => "Look Behind",
                _ => throw new ArgumentOutOfRangeException(nameof(action), action, null)
            };
        }
    }
}