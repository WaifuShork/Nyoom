using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Nyoom
{
    
 
    [RequireComponent(typeof(Rigidbody))][RequireComponent(typeof(BoxCollider))]
    public class Motorcycle : MonoBehaviour { //MS Motorcycle test - Marcos Schultz (www.schultzgames.com)
     
        WheelCollider frontWheel;
        WheelCollider rearWheel;
        GameObject meshFront;
        GameObject meshRear;
        Rigidbody ms_Rigidbody;
     
        float rbVelocityMagnitude;
        float horizontalInput;
        float verticalInput;
        float medRPM;
     
        void Awake () {
            transform.rotation = Quaternion.identity;
            ms_Rigidbody = GetComponent<Rigidbody> ();
            ms_Rigidbody.mass = 400;
            ms_Rigidbody.interpolation = RigidbodyInterpolation.Extrapolate;
            ms_Rigidbody.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
     
            //centerOfMass
            GameObject centerOfmassOBJ = new GameObject ("centerOfmass");
            centerOfmassOBJ.transform.parent = transform;
            centerOfmassOBJ.transform.localPosition = new Vector3 (0.0f, -0.3f, 0.0f);
            ms_Rigidbody.centerOfMass = transform.InverseTransformPoint(centerOfmassOBJ.transform.position);
            //
            BoxCollider collider = GetComponent<BoxCollider>();
            collider.size = new Vector3 (0.5f, 1.0f, 3.0f);
            //
            GenerateWheels();
        }
     
        void OnEnable(){
            WheelCollider WheelColliders = GetComponentInChildren<WheelCollider>();
            WheelColliders.ConfigureVehicleSubsteps(1000, 30, 30);
        }
     
        void FixedUpdate () {
            horizontalInput = Input.GetAxis ("Horizontal");
            verticalInput = Input.GetAxis ("Vertical");
            medRPM = (frontWheel.rpm + rearWheel.rpm) / 2;
            rbVelocityMagnitude = ms_Rigidbody.velocity.magnitude;
     
            //motorTorque
            if (medRPM > 0) {
                rearWheel.motorTorque = verticalInput * ms_Rigidbody.mass * 4.0f;
            } else {
                rearWheel.motorTorque = verticalInput * ms_Rigidbody.mass * 1.5f;
            }
     
            //steerAngle
            float nextAngle = horizontalInput * 35.0f;
            frontWheel.steerAngle = Mathf.Lerp (frontWheel.steerAngle, nextAngle, 0.125f);
     
     
            if(Mathf.Abs(rearWheel.rpm) > 10000){
                rearWheel.motorTorque = 0.0f;
                rearWheel.brakeTorque = ms_Rigidbody.mass * 5;
            }
            //
            if (rbVelocityMagnitude < 1.0f && Mathf.Abs (verticalInput) < 0.1f) {
                rearWheel.brakeTorque = frontWheel.brakeTorque = ms_Rigidbody.mass * 2.0f;
            } else {
                rearWheel.brakeTorque = frontWheel.brakeTorque = 0.0f;
            }
            //
            Stabilizer();
        }
     
        void Update(){
            //update wheel meshes
            Vector3 temporaryVector;
            Quaternion temporaryQuaternion;
            //
            frontWheel.GetWorldPose(out temporaryVector, out temporaryQuaternion);
            meshFront.transform.position = temporaryVector;
            meshFront.transform.rotation = temporaryQuaternion;
            //
            rearWheel.GetWorldPose(out temporaryVector, out temporaryQuaternion);
            meshRear.transform.position = temporaryVector;
            meshRear.transform.rotation = temporaryQuaternion;
        }
     
        void Stabilizer(){
            Vector3 axisFromRotate = Vector3.Cross (transform.up, Vector3.up);
            Vector3 torqueForce = axisFromRotate.normalized * axisFromRotate.magnitude * 50;
            torqueForce.x = torqueForce.x * 0.4f;
            torqueForce -= ms_Rigidbody.angularVelocity;
            ms_Rigidbody.AddTorque (torqueForce * ms_Rigidbody.mass * 0.02f, ForceMode.Impulse);
     
            float rpmSign = Mathf.Sign (medRPM) * 0.02f;
            if (rbVelocityMagnitude > 1.0f && frontWheel.isGrounded && rearWheel.isGrounded) {
                ms_Rigidbody.angularVelocity += new Vector3 (0, horizontalInput * rpmSign, 0);
            }
        }
     
        void GenerateWheels(){
            GameObject frontWeelObject = new GameObject ("frontWheel");
            frontWeelObject.transform.parent = transform;
            frontWeelObject.transform.localPosition = new Vector3 (0, -0.5f, 1.0f);
            frontWheel = frontWeelObject.gameObject.AddComponent<WheelCollider> () as WheelCollider;
            //
            GameObject rearWheelObject = new GameObject ("rearWheel");
            rearWheelObject.transform.parent = transform;
            rearWheelObject.transform.localPosition = new Vector3 (0, -0.5f, -1.0f);
            rearWheel = rearWheelObject.gameObject.AddComponent<WheelCollider> () as WheelCollider;
     
            //settings
            frontWheel.mass = rearWheel.mass = 40;
            frontWheel.radius = rearWheel.radius = 0.5f;
            frontWheel.wheelDampingRate = rearWheel.wheelDampingRate = 0.75f;
            frontWheel.suspensionDistance = rearWheel.suspensionDistance = 0.35f;
            frontWheel.forceAppPointDistance = rearWheel.forceAppPointDistance = 0;
     
            //spring
            JointSpring suspensionSpringg = new JointSpring ();
            suspensionSpringg.spring = 15000;        
            suspensionSpringg.damper = 4000;        
            suspensionSpringg.targetPosition = 0.5f;
            frontWheel.suspensionSpring = rearWheel.suspensionSpring = suspensionSpringg;
     
            //Friction
            WheelFrictionCurve wheelFrictionCurveFW = new WheelFrictionCurve(); //friction FW
            wheelFrictionCurveFW.extremumSlip = 2.0f;
            wheelFrictionCurveFW.extremumValue = 4.0f;
            wheelFrictionCurveFW.asymptoteSlip = 4.0f;
            wheelFrictionCurveFW.asymptoteValue = 2.0f;
            wheelFrictionCurveFW.stiffness = 2.0f;
            frontWheel.forwardFriction = rearWheel.forwardFriction = wheelFrictionCurveFW;
     
            WheelFrictionCurve wheelFrictionCurveSW = new WheelFrictionCurve(); //friction SW
            wheelFrictionCurveSW.extremumSlip = 0.2f;
            wheelFrictionCurveSW.extremumValue = 1.0f;
            wheelFrictionCurveSW.asymptoteSlip = 0.5f;
            wheelFrictionCurveSW.asymptoteValue = 0.75f;
            wheelFrictionCurveSW.stiffness = 2.0f;
            frontWheel.sidewaysFriction = rearWheel.sidewaysFriction = wheelFrictionCurveSW;
     
     
            //generateMeshes
            GameObject wheelFrontMesh = new GameObject ("wheelFrontMesh");
            GameObject meshFrontTemp = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            Destroy (meshFrontTemp.GetComponent<CapsuleCollider> ());
            meshFrontTemp.transform.parent = wheelFrontMesh.transform;
            meshFrontTemp.transform.localPosition = new Vector3 (0.0f, 0.0f, 0.0f);
            meshFrontTemp.transform.localEulerAngles = new Vector3 (0.0f, 0.0f, 90.0f);
            meshFrontTemp.transform.localScale = new Vector3 (1.0f, 0.1f, 1.0f);
            meshFront = wheelFrontMesh;
            meshFront.transform.parent = transform;
            meshFront.transform.localPosition = new Vector3 (0, -0.5f, 1.0f);
            //
            GameObject wheelRearMesh = new GameObject ("wheelRearMesh");
            GameObject meshRearTemp = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            Destroy (meshRearTemp.GetComponent<CapsuleCollider> ());
            meshRearTemp.transform.parent = wheelRearMesh.transform;
            meshRearTemp.transform.localPosition = new Vector3 (0.0f, 0.0f, 0.0f);
            meshRearTemp.transform.localEulerAngles = new Vector3 (0.0f, 0.0f, 90.0f);
            meshRearTemp.transform.localScale = new Vector3 (1.0f, 0.1f, 1.0f);
            meshRear = wheelRearMesh;
            meshRear.transform.parent = transform;
            meshRear.transform.localPosition = new Vector3 (0, -0.5f, -1.0f);
            //
            GameObject meshCube = GameObject.CreatePrimitive(PrimitiveType.Cube);
            Destroy (meshCube.GetComponent<BoxCollider> ());
            meshCube.transform.parent = transform;
            meshCube.transform.localPosition = new Vector3 (0.0f, 0.0f, 0.0f);
            meshCube.transform.localScale = new Vector3 (0.5f, 1.0f, 3.0f);
        }
    }
}