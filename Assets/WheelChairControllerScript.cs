using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WheelChairControllerScript : MonoBehaviour
{

    public float  Radius = 10;

    public GameObject Center;
    public GameObject WheelChair;

    public float mpsx = 0;
    public float mpsy = 0;

    private RigidBody rb;

    private void Move()
    {
        float steerX;
        float steerY;

    }


    void Start()
    {
        rb = WheelChair.GetComponent<RigidBody>();
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
