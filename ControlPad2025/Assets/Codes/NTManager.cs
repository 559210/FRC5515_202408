using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using NetworkTablesSharp;
using System.Threading;
using static NTManager;

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
    public class RobotPos {
        public float x;
        public float y;
        public float degree;
    }
    public class AprilTagTargetInfo
    {
        public long aprilTagId = -1;
        public long level;
        public long branch;
    }
    readonly static string ID_NAME = "5515ControlPad"; 
    readonly string ROBOT_POS_ENTRY_NAME = $"/{ID_NAME}/RobotPos";
    readonly string CONTROL_PAD_INFO_ENTRY_NAME = $"/{ID_NAME}/ControlPadInfo";
    readonly string CONTROL_PAD_INFO_RECALL_ENTRY_NAME = $"/{ID_NAME}/ControlPadInfoRecall";
    //readonly string CONTROL_PAD_INFO_RECALL_ENTRY_NAME = $"/{ID_NAME}/aaa";
    readonly string VIRTUAL_CONTROL_ENTRY_NAME = $"/{ID_NAME}/VirtualControl";
    readonly string GO_TARGET_ENTRY_NAME = $"/{ID_NAME}/GoTarget";

    RobotPos robotPos = new();
    AprilTagTargetInfo aprilTagTargetInfo = new();
    Nt4Source nt = null;

    bool isFirstConnect = true;

    public bool Connected
    {
        get
        {
            if (nt == null)
            {
                return false;
            }

            return nt.Connected();
        }
    }
    public bool init()
    {
        //nt = new("10.55.15.2", TOPIC_NAME, false, 5810);
        nt = new("127.0.0.1", ID_NAME, false, 5810);

        startGuard();
        return true;
    }

    protected void startGuard()
    {
        Main.inst.StartCoroutine(guardNT());
    }

    protected IEnumerator guardNT()
    {
        while (true)
        {
            if (nt.Connected())
            {
                isFirstConnect = false;
                yield return new WaitForSeconds(0.2f);
                continue;
            }

            if (!isFirstConnect)
                Debug.LogError("断线 ，开始重连");
            yield return new WaitForEndOfFrame();
            ConnectNT();
            if (nt.Connected())
            {
                Debug.LogError("连接成功");
            }
            else
            {
                Debug.LogError("连接失败,重试...");
            }
            yield return new WaitForSeconds(1);
        }
    }

    public void ConnectNT()
    {
        if (nt.Connected())
        {
            return;
        }

        // Not needed if you leave connectAutomatically as true

        nt.Connect();

        if (nt.Connected()) {
            nt.Subscribe(ROBOT_POS_ENTRY_NAME);
            nt.Subscribe(CONTROL_PAD_INFO_RECALL_ENTRY_NAME);
            nt.PublishTopic(CONTROL_PAD_INFO_ENTRY_NAME, "int[]");
            nt.PublishTopic(VIRTUAL_CONTROL_ENTRY_NAME, "double[]");
            nt.PublishTopic(GO_TARGET_ENTRY_NAME, "int[]");

            Main.inst.StartCoroutine(arrangeRefreshAprilTagTargetInfoRecall());

            Debug.LogError("NT connected");
        }
    }

    public void stop() {
        nt.Disconnect();
    }

    private bool check()
    {
        if (nt == null) { return false; }
        return nt.Connected();
    }

    public bool publishControlPadInfo(long aprilTagId, long level, long branch)
    {
        aprilTagTargetInfo.aprilTagId = aprilTagId;
        aprilTagTargetInfo.level = level;
        aprilTagTargetInfo.branch = branch;
        Debug.LogErrorFormat("publish control pad info===> aprilTagId: {0}, level: {1}, branch: {2} ", aprilTagId, level, branch);
        if (!check())
        {
            Debug.LogError("publishControlPadInfo check fail");
            return false;
        }

        
        nt.PublishValue(CONTROL_PAD_INFO_ENTRY_NAME, new long[] {aprilTagId, level, branch});

        return true;
    }

    public bool publishVirtualControl(bool isTapOn, float tapPosX, float tapPosY)
    {
        if (!check())
        {
            Debug.LogError("publishVirtualControl check fail");
            return false;
        }
        nt.PublishValue(VIRTUAL_CONTROL_ENTRY_NAME, new double[] { isTapOn ? 1 : 0, tapPosX, tapPosY });
        return true;
    }

    public bool publishGoTarget(bool isGo)
    {
        if (!check())
        {
            Debug.LogError("publishGoTarget check fail");
            return false;
        }
        nt.PublishValue(GO_TARGET_ENTRY_NAME, new long[] { isGo ? 1 : 0, Time.frameCount });
        return true;
    }

    public RobotPos getRobotPos()
    {
        // Debug.LogError("getRobotPos");
        if(!check()) 
        { 
            //Debug.LogError("getRobotPos check fail");
            return null; 
        }
        
        double[] posList = nt.GetValue<double[]>(ROBOT_POS_ENTRY_NAME);
        if (posList == null) {
            return null;
        }
        double x = posList[0];
        double y = posList[1];
        double degree = posList[2];
        

        robotPos.x = (float)x;
        robotPos.y = (float)y;
        robotPos.degree = (float)degree;
        return robotPos;
    }

    protected IEnumerator arrangeRefreshAprilTagTargetInfoRecall()
    {
        yield return new WaitForSeconds(1);
        refreshAprilTagTargetInfoRecall();
    }
    public AprilTagTargetInfo refreshAprilTagTargetInfoRecall()
    {
        if (!check())
        {
            return null;
        }
        long[] rawData = nt.GetValue<long[]>(CONTROL_PAD_INFO_RECALL_ENTRY_NAME);
        if (rawData == null)
        {
            return null;
        }
        aprilTagTargetInfo.aprilTagId = rawData[0];
        aprilTagTargetInfo.level = rawData[1];
        aprilTagTargetInfo.branch = rawData[2];
        return aprilTagTargetInfo;
    }

    public AprilTagTargetInfo getAprilTagTargetInfo()
    {
        return aprilTagTargetInfo;
    }
}
