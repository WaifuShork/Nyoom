using UnityEngine;
using Sirenix.OdinInspector;

namespace Nyoom
{
    [HideMonoScript]
    public class KartStabilization : MonoBehaviour
    {
        public void Stabilize(Rigidbody rb, RaycastHit raycastHit)
        {
            var normal = raycastHit.distance < 10f ? raycastHit.normal : Vector3.up;
            var t = rb.transform;
            var rotation = t.rotation;
            var targetRotation = Quaternion.FromToRotation(t.up, normal) * rotation;
            rb.MoveRotation(Quaternion.Slerp(rotation, targetRotation, 5f * Time.deltaTime));
            
            /*var t = rb.transform;
            var angles = new Vector3
            {
                x = groundNormal.x,
                y = t.localEulerAngles.y,
                z = groundNormal.z,
            };
            
            var targetRotation = Quaternion.Euler(angles);
            var smoothing = 5f * Time.smoothDeltaTime;
            // t.localRotation = Quaternion.Slerp(t.localRotation, targetRotation, smoothing);
            t.localRotation = Quaternion.Slerp(t.localRotation, Quaternion.FromToRotation(transform.up, angles), 10f * Time.deltaTime);
            
            rb.angularVelocity = Vector3.Lerp(rb.angularVelocity, Vector3.zero, smoothing);
        */
        }
    }
}