using UnityEngine;
using UnityEngine.UI;
using UnityEngine.SceneManagement;
using System.Collections.Generic;
using NWH.WheelController3D;

namespace NWH.WheelController3D
{
    public class GUIController : MonoBehaviour
    {

        public GameObject currentVehicle;
        [SerializeField]
        public List<GameObject> vehicles;
        private WheelController[] wcs;

        public Text speedText;
        private int speed;

        private int vehicleSelector;

        public GameObject canvas;

        [HideInInspector]
        public CarController cc;

        void Start()
        {
            currentVehicle = vehicles[vehicleSelector];
            currentVehicle.GetComponent<CarController>().Active(true);
            wcs = currentVehicle.GetComponentsInChildren<WheelController>();

            cc = currentVehicle.GetComponent<CarController>();
        }

        void Update()
        {
            currentVehicle = vehicles[vehicleSelector];
            wcs = currentVehicle.GetComponentsInChildren<WheelController>();
            cc = currentVehicle.GetComponent<CarController>();
            Camera.main.GetComponent<CameraDefault>().TargetLookAt = currentVehicle.transform;

            SetMeter();
            SetSpeed(cc.velocity * 3.6f);
        }

        public void ChangeVehicle()
        {
            var nextSelector = vehicleSelector + 1;
            if (nextSelector >= vehicles.Count)
            {
                nextSelector = 0;
            }

            if (nextSelector != vehicleSelector)
            {
                vehicles[vehicleSelector].GetComponent<CarController>().Active(false);
                vehicles[nextSelector].GetComponent<CarController>().Active(true);
            }

            vehicleSelector = nextSelector;
        }

        private void SetMeter()
        {
            speedText.text = Mathf.Abs(speed).ToString();
        }

        public void SetSpeed(float currentSpeed)
        {
            speed = Mathf.RoundToInt(currentSpeed);
        }

        public void LevelReset()
        {
            Scene scene = SceneManager.GetActiveScene();
            SceneManager.LoadScene(scene.name);
        }

        public void SurfaceTarmacDry()
        {
            AdjustFriction(WheelController.FrictionPreset.TarmacDry);
        }

        public void SurfaceTarmacWet()
        {
            AdjustFriction(WheelController.FrictionPreset.TarmacWet);
        }

        public void SurfaceGravel()
        {
            AdjustFriction(WheelController.FrictionPreset.Gravel);
        }

        public void SurfaceSnow()
        {
            AdjustFriction(WheelController.FrictionPreset.Snow);
        }

        public void SurfaceGeneric()
        {
            AdjustFriction(WheelController.FrictionPreset.Generic);
        }

        public void SurfaceIce()
        {
            AdjustFriction(WheelController.FrictionPreset.Ice);
        }

        public void IncreaseSpringLength()
        {
            foreach (WheelController w in wcs)
            {
                w.springLength += w.springLength * 0.1f;
                w.springLength = Mathf.Clamp(w.springLength, 0.15f, 0.6f);
            }
        }

        public void DecreaseSpringLength()
        {
            foreach (WheelController w in wcs)
            {
                w.springLength -= w.springLength * 0.1f;
                w.springLength = Mathf.Clamp(w.springLength, 0.15f, 0.6f);
            }
        }

        public void IncreaseSpringStrength()
        {
            foreach (WheelController w in wcs)
            {
                w.springMaximumForce += w.springMaximumForce * 0.1f;
                w.springMaximumForce = Mathf.Clamp(w.springMaximumForce, 14000f, 45000f);
            }
        }

        public void DecreaseSpringStrength()
        {
            foreach (WheelController w in wcs)
            {
                w.springMaximumForce -= w.springMaximumForce * 0.1f;
                w.springMaximumForce = Mathf.Clamp(w.springMaximumForce, 14000f, 45000f);
            }
        }

        public void IncreaseRebound()
        {
            foreach (WheelController w in wcs)
            {
                w.damperUnitReboundForce += w.damperUnitReboundForce * 0.1f;
                w.damperUnitReboundForce = Mathf.Clamp(w.damperUnitReboundForce, 300f, 2500f);
            }
        }

        public void DecreaseRebound()
        {
            foreach (WheelController w in wcs)
            {
                w.damperUnitReboundForce -= w.damperUnitReboundForce * 0.1f;
                w.damperUnitReboundForce = Mathf.Clamp(w.damperUnitReboundForce, 300f, 2500f);
            }
        }

        public void IncreaseBump()
        {
            foreach (WheelController w in wcs)
            {
                w.damperUnitBumpForce += w.damperUnitBumpForce * 0.1f;
                w.damperUnitBumpForce = Mathf.Clamp(w.damperUnitBumpForce, 300f, 2500f);
            }
        }

        public void DecreaseBump()
        {
            foreach (WheelController w in wcs)
            {
                w.damperUnitBumpForce -= w.damperUnitBumpForce * 0.1f;
                w.damperUnitBumpForce = Mathf.Clamp(w.damperUnitBumpForce, 300f, 2500f);
            }
        }

        public void IncreaseRimOffset()
        {
            foreach (WheelController w in wcs)
            {
                w.rimOffset += 0.05f;
                w.rimOffset = Mathf.Clamp(w.rimOffset, -0.2f, 0.2f);
            }
        }

        public void DecreaseRimOffset()
        {
            foreach (WheelController w in wcs)
            {
                w.rimOffset -= 0.05f;
                w.rimOffset = Mathf.Clamp(w.rimOffset, -0.2f, 0.2f);
            }
        }

        public void IncreaseCamber()
        {
            foreach (WheelController w in wcs)
            {
                float camber = w.camber + 2f;
                camber = Mathf.Clamp(camber, -15f, 15f);
                w.SetCamber(camber);
            }
        }

        public void DecreaseCamber()
        {
            foreach (WheelController w in wcs)
            {
                float camber = w.camber - 2f;
                camber = Mathf.Clamp(camber, -15f, 15f);
                w.SetCamber(camber);
            }
        }

        public void AdjustFriction(WheelController.FrictionPreset p)
        {
            foreach (WheelController w in wcs)
            {
                w.SetActiveFrictionPreset(p);
            }
        }
    }

}
