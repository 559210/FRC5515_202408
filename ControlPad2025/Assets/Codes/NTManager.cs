using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using NetworkTablesSharp;

// Default values shown here
//Nt4Source Source = new Nt4Source("10.55.15.2", "5515ControlPad", true, 5810);
//// Not needed if you leave connectAutomatically as true
//Source.Connect();

//if (Source.Connected())
//{
//    Debug.LogError("Nt4 is connected");
//    Source.Subscribe("Key");

//    string latestValue = Source.GetValue<string>("Key");
//    string specificValue = Source.GetValue<string>("Key", (long)Source.GetServerTimeUs());

//    Source.PublishTopic("key", "type" /* ex. string, int */);
//    Source.PublishValue("key", "value");

//    Source.Disconnect();
//}
//else
//{
//    Debug.LogError("Nt4 not connected");
//}

public class NTManager
{
    readonly string TOPIC_NAME = "5515ControlPad";
    readonly string ROBOT_POS_ENTRY_NAME = "RobotPos";
    readonly string CONTROL_PAD_INFO_ENTERY_NAME = "ControlPadInfo";

    Nt4Source nt = null;
    public bool init()
    {
        nt = new("10.55.15.2", TOPIC_NAME, true, 5810);
        // Not needed if you leave connectAutomatically as true
        nt.Connect();

        return check();
    }

    private bool check()
    {
        if (nt == null) { return false; }
        return nt.Connected();
    }

    public bool publishControlPadInfo(long aprilTagId, long level, long branch)
    {
        Debug.LogErrorFormat("publish control pad info===> aprilTagId: {0}, level: {1}, branch: {2} ", aprilTagId, level, branch);
        if (!check())
        {
            return false;
        }

        nt.PublishTopic(CONTROL_PAD_INFO_ENTERY_NAME, "int[]");
        nt.PublishValue(CONTROL_PAD_INFO_ENTERY_NAME, new long[] {aprilTagId, level, branch});
        return true;
    }

    public void getRobotPos()
    {
        Debug.LogError("getRobotPos");
        if(!check()) 
        { 
            return; 
        }
        nt.Subscribe(ROBOT_POS_ENTRY_NAME);
        double[] posList = nt.GetValue<double[]>(ROBOT_POS_ENTRY_NAME);
        double x = posList[0];
        double y = posList[1];
        double degree = posList[2];
        Debug.LogErrorFormat("Got robot pos: x->{0}, y->{1}, degree->{2}", x, y, degree);
        return;
    }
}
