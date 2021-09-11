using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WheelChairControllerScript : MonoBehaviour
{


    ///
    /// <summary>
    /// A simple PID controlled wheelchair simulation to drive in a circle
    /// </summary>
    /// 
    /// <note>
    /// The camera is facing down along the z angle, thus all forward directions are transform.up not transform.forward!
    /// </note>

    

    
    [Header("Control Inputs")]
    [Range(0, 1)]
    public float forward;           // joystick forward
    public float ForwardTarget;     // defines the meter to our projected target (see Projected)

    [Range(5, 10)]
    public float SetRadius;         // the radius of the circle

    
    private float speedFactor;       // multiplicator for the wheels, should be fixed. 100 is a nice value

    [Header("Control Outputs")]
    public float IsRadius;          // the actual radius the chair drives
    [Range(-1, 1)]
    public float steer;             // joystick left/right

    public float steeringAngle;     // the angle to steer towards the target

    [Range(0, 1)]
    public float throttleLeft = 0;  // motor power ratio left
    [Range(0, 1)]
    public float throttleRight = 0; // motor power ratio right

    [Header("Simulation Components")]
    public GameObject TargetObject; // the target the chair aims for (for visualiziation only)
    public GameObject Projected;    // a target in front of the chair ( see ForwardTarget)


    public GameObject Center;       // the center of the circle
    public GameObject Wheelchair;   // the wheelchair itself

    public GameObject WheelChairLeft; // the left motor
    public GameObject WheelChairRight; // the right motor


    [Header("PID Inputs")]
    // the set point is the angle between the target and the forward vector, this should be 0
    public float SetPoint;  

    public float pGain; 
    public float iGain;
    public float dGain;

    public float gainFactor; // multiplied with each Gain, should be 1


    public float min;        // min value the PID generates
    public float max;        // max value the PID generates

    [Header("PID Outputs")]
    public float p;         // the process variable (steering angle)
    public float i;         // the integral value of the PID
    public float c;         // the control variable (= steer; joystick left/right)





    private Rigidbody rbl; // rigid nody of the left motor
    private Rigidbody rbr; // rigid nody of the right motor


    //https://github.com/ms-iot/pid-controller
    private PidController.PidController pid;





    private Vector3 target = new Vector3(); // position of the target




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


        // calculating the steering angle
        // we project a virtual target in front of the chair, then finding the nearest point on our circle and steer towards this point

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


        // Setting up pid vars

        pid.GainDerivative = dGain;
        pid.GainIntegral = iGain;
        pid.GainProportional = pGain;

        pid.SetPoint = SetPoint;
        pid.ProcessVariable = steeringAngle;

        c = (float) pid.ControlVariable(Time.deltaTime);
        p = (float) pid.ProcessVariable;
        i = (float) pid.IntegralTerm;

        

        // this part is for controlling the wheels
        // we simply move/steer both wheels by adding relative forces to each wheel and subtracting a factor from it when steering in a direction
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
