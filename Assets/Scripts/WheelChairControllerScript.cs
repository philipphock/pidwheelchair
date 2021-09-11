using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WheelChairControllerScript : MonoBehaviour
{

    public float  Radius = 10;

    public GameObject Center;
    public GameObject WheelChairLeft;
    public GameObject WheelChairRight;


    [Range(0, 1)]
    public float forward;
    [Range(-1, 1)]
    public float steer;

    [Range(0, 100)]
    public float speedFactor;
    
    [Range(0, 1)]
    public float throttleLeft = 0;
    
    [Range(0, 1)]
    public float throttleRight = 0;




    private Rigidbody rbl;
    private Rigidbody rbr;

    void Start()
    {
        rbr = WheelChairRight.GetComponent<Rigidbody>();
        rbl = WheelChairLeft.GetComponent<Rigidbody>();

    }

    // Update is called once per frame
    void Update()
    {
        
    }

    void FixedUpdate(){

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

        
    }
}
