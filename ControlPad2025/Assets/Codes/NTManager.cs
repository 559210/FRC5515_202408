using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using NetworkTablesSharp;
using System.Text.RegularExpressions;
using System;
using UnityEngine.Windows;


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

    public enum DebugPanelInfo
    {
        NONE = 0,
        ZERO,
        READY_FOR_LOAD_CORAL,
        L1,
        L2,
        L3,
        L4,
        BALL1,
        BALL2,
        UP_ARROW,
        DOWN_ARROW,
        LEFT_ARROW,
        RIGHT_ARROW,
    }
    readonly static string ID_NAME = "5515ControlPad"; 
    readonly string ROBOT_POS_ENTRY_NAME = $"/{ID_NAME}/RobotPos";
    readonly string CONTROL_PAD_INFO_ENTRY_NAME = $"/{ID_NAME}/ControlPadInfo";
    readonly string CONTROL_PAD_INFO_RECALL_ENTRY_NAME = $"/{ID_NAME}/ControlPadInfoRecall";
    //readonly string CONTROL_PAD_INFO_RECALL_ENTRY_NAME = $"/{ID_NAME}/aaa";
    readonly string VIRTUAL_CONTROL_ENTRY_NAME = $"/{ID_NAME}/VirtualControl";
    readonly string GO_TARGET_ENTRY_NAME = $"/{ID_NAME}/GoTarget";
    readonly string DEBUG_PANEL_ENTRY_NAME = $"/{ID_NAME}/Debug";
    readonly string UPPER_SYSTEM_STATE = "/SmartDashboard/US2025Cmd_State";
    readonly string UPPER_SYSTEM_CARRYING_STATE = "/SmartDashboard/US2025Cmd_CarryingState";

    RobotPos robotPos = new();
    AprilTagTargetInfo aprilTagTargetInfo = new();
    NetworkTablesSharpEx.Nt4Source nt = null;

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
        nt = new("10.55.15.2", ID_NAME, false, 5810);
        //nt = new("127.0.0.1", ID_NAME, false, 5810);

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
            yield return new WaitForEndOfFrame();
            ConnectNT();
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
            nt.Subscribe(UPPER_SYSTEM_STATE);
            nt.Subscribe(UPPER_SYSTEM_CARRYING_STATE);
            nt.PublishTopic(CONTROL_PAD_INFO_ENTRY_NAME, "int[]");
            nt.PublishTopic(VIRTUAL_CONTROL_ENTRY_NAME, "double[]");
            nt.PublishTopic(GO_TARGET_ENTRY_NAME, "int[]");
            nt.PublishTopic(DEBUG_PANEL_ENTRY_NAME, "int[]");

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

    public class UpperSystemState
    {
        public string state = "UNKOWN";
        public string runningState = "UNKOWN";
        public string isCarryingCoral = "UNKOWN";
        public string isCarryingBall = "UNKOWN";
    }

    UpperSystemState upperSystemState = new();
    public UpperSystemState getUpperSystemStates()
    {
        if (!check())
        {
            return null;
        }

        string stateStr = nt.GetValue<string>(UPPER_SYSTEM_STATE);
        // state: ZERO running state: DONE
        if (stateStr == null)
        {
            return null;
        }

        string pattern1 = @"state:\s+(\w+)\s+running state:\s+(\w+)";

        Match match1 = Regex.Match(stateStr, pattern1);
        if (match1.Success)
        {
            upperSystemState.state = match1.Groups[1].Value; // ��ȡZERO
            upperSystemState.runningState = match1.Groups[2].Value; // ��ȡDONE
        }

        string carryingStateStr = nt.GetValue<string>(UPPER_SYSTEM_CARRYING_STATE);
        // isCarryingCoral: false isCarryingBall: false
        if (carryingStateStr == null)
        {
            return null;
        }

        string pattern2 = @"isCarryingCoral:\s+(\w+)\s+isCarryingBall:\s+(\w+)";
        Match match2 = Regex.Match(carryingStateStr, pattern2);
        if (match2.Success)
        {
            upperSystemState.isCarryingCoral = match2.Groups[1].Value; // ��ȡZERO
            upperSystemState.isCarryingBall = match2.Groups[2].Value; // ��ȡDONE
        }

        return upperSystemState;
    }

    public AprilTagTargetInfo getAprilTagTargetInfo()
    {
        return aprilTagTargetInfo;
    }

    public bool publishDebugPanelInfo(int debugBtnVal, bool isLoadCoral, bool isLoadBall)
    {
        if (!check())
        {
            Debug.LogError("publishGoTarget check fail");
            return false;
        }
        nt.PublishValue(DEBUG_PANEL_ENTRY_NAME, new long[] { debugBtnVal, isLoadCoral ? 1 : 0, isLoadBall ? 1 : 0, Time.frameCount });
        return true;
    }

}
