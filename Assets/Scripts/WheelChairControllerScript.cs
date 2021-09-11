using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WheelChairControllerScript : MonoBehaviour
{
    /// <note>
    /// The camera is facing down along the z angle, thus all forward directions are transform.up not transform.forward!
    /// </note>

    public float IsRadius;
    public float ForwardTarget;
    public float  SetRadius;

    public GameObject TargetObject;
    public GameObject Projected;


    public GameObject Center;
    public GameObject Wheelchair;

    public GameObject WheelChairLeft;
    public GameObject WheelChairRight;


    [Range(0, 1)]
    public float forward;
    [Range(-1, 1)]
    public float steer;

    public float steeringAngle;

    [Range(0, 100)]
    public float speedFactor;
    
    [Range(0, 1)]
    public float throttleLeft = 0;
    
    [Range(0, 1)]
    public float throttleRight = 0;

    // the set point is the angle between the target and the forward vector, this should be 0
    public float SetPoint;



    private Rigidbody rbl;
    private Rigidbody rbr;

    //https://github.com/ms-iot/pid-controller
    private PidController.PidController pid;



    public float pGain;
    public float iGain;
    public float dGain;


    public float min;
    public float max;

    public float gainFactor;

    public float p;
    public float i;
    public float c;

    public Vector3 target = new Vector3();




    void Start()
    {
        rbr = WheelChairRight.GetComponent<Rigidbody>();
        rbl = WheelChairLeft.GetComponent<Rigidbody>();
        pid = new PidController.PidController(pGain * gainFactor, iGain * gainFactor, dGain * gainFactor, max, min);
        IsRadius = Vector3.Distance(Center.transform.position, Wheelchair.transform.position);

    }

    // Update is called once per frame
    void Update()
    {
        
    }

    void FixedUpdate(){

        // this is a target ForwardTarget m in front of the wheelchair
        Vector3 forwardFromWheelchair = Wheelchair.transform.position + Wheelchair.transform.up * ForwardTarget;

        Projected.transform.position = forwardFromWheelchair;
        // get the vector from the center to that target
        target = forwardFromWheelchair - Center.transform.position; 
        // normalize it to have the direction
        target.Normalize();
        // multiply with radius to have it on the circle
        target *= SetRadius;
        // this is the point we want to steer
        TargetObject.transform.position = target;
        // now calculate the steering angle

        // first we get the target direction:
        Vector3 targetDir = target - Wheelchair.transform.position;
               
        Vector2 v0 = new Vector2(Wheelchair.transform.up.x, Wheelchair.transform.up.y);
        Vector2 v1 = new Vector2(targetDir.x, targetDir.y);

        steeringAngle = Vector2.SignedAngle(v0,v1);



        pid.GainDerivative = dGain;
        pid.GainIntegral = iGain;
        pid.GainProportional = pGain;

        pid.SetPoint = SetPoint;
        pid.ProcessVariable = steeringAngle;

        c = (float) pid.ControlVariable(Time.deltaTime);
        p = (float) pid.ProcessVariable;
        i = (float) pid.IntegralTerm;

        

        steer = c;


        float lrAbs = Mathf.Abs(steer);
        float brL = 0;
        float brR = 0;

        float br = forward * lrAbs;


        if (steer < 0)
        {
            brR = br;
        }
        else if (steer > 0)
        {
            brL = br;
        }
        

        throttleLeft = forward - brL;
        throttleRight = forward - brR;

        rbl.AddRelativeForce(Vector3.up * throttleLeft * speedFactor * Time.deltaTime);
        rbr.AddRelativeForce(Vector3.up * throttleRight * speedFactor * Time.deltaTime);

        IsRadius = Vector3.Distance(Center.transform.position, Wheelchair.transform.position);


    }
}
