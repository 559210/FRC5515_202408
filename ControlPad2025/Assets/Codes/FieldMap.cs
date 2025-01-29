using FairyGUI;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;
using static NTManager;

public class Grid
{
    public Grid(int gridPosX, int gridPosY, float width, GameObject proto, Transform parent)
    {
        // gridpos 左下为原点，右+，上+
        this.gridPosX = gridPosX;
        this.gridPosY = gridPosY;
        go = GameObject.Instantiate(proto);
        trans = go.transform;
        if (parent != null)
        {
            trans.parent = parent;
        }
        Vector3 pos = parent.position;
        trans.localPosition = new Vector3 (gridPosX * width + width / 2, gridPosY * width + width / 2, 0);
        sr = go.GetComponent<SpriteRenderer>();
        size = new Vector2(width, width);
        sr.size = size;

    }
    public readonly int gridPosX;
    public readonly int gridPosY;
    public readonly Vector2 size;
    public readonly GameObject go;
    public readonly Transform trans;
    public readonly SpriteRenderer sr;
}

public class InfoPanel
{

    public enum SIDE
    {
        BLUE = 0,
        RED = 1
    }
    GComponent root;


    GButton toRedBtn;
    GButton toBlueBtn;
    GButton showBtn;
    GButton hideBtn;
    GButton goTargetBtn;

    Controller hideCtrl;
    Controller sideCtrl;
    Controller netCtrl;

    GTextField aprilTagSelectedInfoLbl;
    GTextField robotPosLbl;
    public InfoPanel(GComponent comp)
    {
        root = comp;

        toRedBtn = root.GetChild("toRedBtn").asButton;
        toRedBtn.onClick.Set(toRedClicked);
        toBlueBtn = root.GetChild("toBlueBtn").asButton;
        toBlueBtn.onClick.Set(toBlueClicked);
        showBtn = root.GetChild("toShowBtn").asButton;
        showBtn.onClick.Set(toShowClicked);
        hideBtn = root.GetChild("toHideBtn").asButton;
        hideBtn.onClick.Set(toHideClicked);

        goTargetBtn = root.GetChild("goTargetBtn").asButton;
        goTargetBtn.onClick.Set(onGoTargetClicked);

        hideCtrl = root.GetController("HIDE");
        sideCtrl = root.GetController("SIDE");
        netCtrl = root.GetController("NET");

        aprilTagSelectedInfoLbl = root.GetChild("ArpilTagSelectedInfo").asTextField;
        robotPosLbl = root.GetChild("RobotPos").asTextField;
    }

    public void showNetInfo(bool isConnected)
    {
        if (isConnected)
        {
            netCtrl.selectedIndex = 0;
        }
        else
        {
            netCtrl.selectedIndex = 1;
        }
    }

    public void showRobotPos(RobotPos rp)
    {
        if (rp == null)
        {
            robotPosLbl.text = "Waiting data...";
        }
        else
        {
            robotPosLbl.text = String.Format("X:{0:0.00}  Y:{1:0.00} Degree:{2:0.00}°", rp.x, rp.y, rp.degree);
        }
        
    }

    public void showAprilTagTarget(AprilTagTargetInfo ti)
    {
        if (ti.aprilTagId == -1)
        {
            aprilTagSelectedInfoLbl.text = "Waiting data...";
            return;
        }
        string levelName = "ERR";
        switch (ti.level)
        {
            case -1:
                levelName = "LEFT";
                break;
            case 0:
                levelName = "BOTTOM";
                break;
            case 1:
                levelName = "RIGHT";
                break;
        }
        aprilTagSelectedInfoLbl.text = String.Format("AprilTag {0}, {1}, {2}", ti.aprilTagId, levelName, ti.branch);
    }

    public void toggleSide()
    {
        toggleSide(sideCtrl.selectedIndex == 0 ? SIDE.RED : SIDE.BLUE);
    }

    public void toggleSide(SIDE side)
    {
        sideCtrl.selectedIndex = (int)side;
        if (side == SIDE.BLUE)
        {
            // 靠左
            root.Center();
            root.x = root.parent.width - root.width;
        }
        else if (side == SIDE.RED)
        {
            // 靠右
            root.Center();
            root.x = 0;
        }
    }

    protected void toRedClicked()
    {
        toggleSide(SIDE.RED);
    }

    protected void toBlueClicked()
    {
        toggleSide(SIDE.BLUE);
    }

    public void showPanel(bool isShow)
    {
        hideCtrl.selectedIndex = isShow ? 0 : 1;
    }

    protected void toShowClicked()
    {
        showPanel(true);
    }

    protected void toHideClicked()
    {
        showPanel(false);
    }

    protected void onGoTargetClicked()
    {

    }
}
public class FieldMap : MonoBehaviour
{
    public GameObject gridPrefab;
    public Transform gridParentNode;
    public float scale = 1f;

    public Grid[][] grids;

    GComponent fieldRoot;
    GComponent field;
    GComponent apMenu;

    GComponent robot;

    public string[] aprilTagNames = new string[]
    {
        "AP6",
        "AP7",
        "AP8",
        "AP9",
        "AP10",
        "AP11",
        "AP17",
        "AP18",
        "AP19",
        "AP20",
        "AP21",
        "AP22",
    };

    public string[] apMenuBtnNames = new string[]
    {
        "btn0",
        "btnL1",
        "btnL2",
        "btnL3",
        "btnR1",
        "btnR2",
        "btnR3"
    };

    public Tuple<long, long>[] apMenuData = new Tuple<long, long>[] {
        new (0, 0),
        new (-1, 1),
        new (-1, 2),
        new (-1, 3),
        new (1, 1),
        new (1, 2),
        new (1, 3)
    };

    long aprilTagId = -1;
    long level = 0;
    long branch = -1;

    GButton[] apBtns;
    GButton[] apMenuBtns;

    InfoPanel infoPanel;

    // Start is called before the first frame update
    void Start()
    {
        // 场地尺寸，米单位 "field_size":{"x":17.548,"y":8.052}
        // 每个grid的缩放比 "nodeSizeMeters":0.3
        UIPackage.AddPackage("ui/main/main");
        fieldRoot = UIPackage.CreateObject("main", "main").asCom;
        GRoot.inst.AddChild(fieldRoot);

        field = fieldRoot.GetChild("field").asCom;
        var touch = field.GetChild("touch");
        touch.onTouchBegin.Set(onFieldTouchBegin);
        touch.onTouchEnd.Set(onFieldTouchEnd);


        GButton testBtn = fieldRoot.GetChild("testBtn").asButton;
        testBtn.onClick.Set(onTestClick);

        apBtns = new GButton[aprilTagNames.Length];
        for (int i = 0; i < apBtns.Length; i++)
        {
            apBtns[i] =  field.GetChild(aprilTagNames[i]).asButton;
            apBtns[i].onClick.Set(onApBtnClick);
            apBtns[i].data = i;
        }
        
        apMenu = field.GetChild("ApMenu").asCom;
        apMenu.visible = false;

        apMenuBtns = new GButton[apMenuBtnNames.Length];
        for (int i = 0; i < apMenuBtns.Length; i++)
        {
            apMenuBtns[i] = apMenu.GetChild(apMenuBtnNames[i]).asButton;
            apMenuBtns[i].onClick.Set(onApMenuBtnClick);
            apMenuBtns[i].data = i;
        }

        robot = field.GetChild("robot").asCom;


        infoPanel = new(fieldRoot.GetChild("infoPanel").asCom);
        infoPanel.toggleSide(InfoPanel.SIDE.BLUE);
    }

    void onApBtnClick(EventContext context)
    {
        GButton btn = context.sender as GButton;
        aprilTagId = long.Parse(aprilTagNames[(int)btn.data].Replace("AP", ""));
        Debug.LogErrorFormat("April tag clicked: {0}, {1}", aprilTagNames[(int)btn.data], aprilTagId);

        apMenu.visible = true;
    }

    void onApMenuBtnClick(EventContext context)
    {
        GButton btn = context.sender as GButton;
        var data = apMenuData[(int)btn.data];
        level = data.Item1;
        branch = data.Item2;

        Main.inst.NT.publishControlPadInfo(aprilTagId, level, branch);

        apMenu.visible = false;
    }


    // Update is called once per frame
    void Update()
    {
        NTManager.RobotPos pos = Main.inst.NT.getRobotPos();
        if (pos != null) {
            robot.SetXY(meter2Pixel(pos.x), field.height - meter2Pixel(pos.y));
            robot.rotation = 360 - pos.degree;
        }

        infoPanel.showNetInfo(Main.inst.NT.Connected);
        infoPanel.showRobotPos(pos);
        infoPanel.showAprilTagTarget(Main.inst.NT.getAprilTagTargetInfo());
    }

    protected float meter2Pixel(float value) {
        return value * field.width / 17.548f;
    }

    protected float pixel2Meter(float value)
    {
        return value / (field.width / 17.548f);
    }
    //void createMap(int w, int h)
    //{
    //    grids = new Grid[w][];
    //    for (int x = 0; x < w; ++x)
    //    {
    //        grids[x] = new Grid[h];
    //        for (int y = 0; y < h; ++y)
    //        {
    //            grids[x][y] = new Grid(x, y, scale, gridPrefab, this.gridParentNode);
    //        }
    //    }
    //}

    protected void onFieldTouchBegin(EventContext context)
    {
        FairyGUI.InputEvent evt = (FairyGUI.InputEvent)context.data;

        var p = Stage.inst.TransformPoint(evt.position, field.displayObject);
        var fieldPos = pixelPoint2FieldPoint(p);
        Debug.LogError(p.ToString() + " -> " + fieldPos.ToString());
        Main.inst.NT.publishVirtualControl(true, fieldPos.x, fieldPos.y);
    }

    protected void onFieldTouchEnd(EventContext context)
    {
        FairyGUI.InputEvent evt = (FairyGUI.InputEvent)context.data;

        var p = Stage.inst.TransformPoint(evt.position, field.displayObject);
        var fieldPos = pixelPoint2FieldPoint(p);
        Debug.LogError(p.ToString() + " -> " + fieldPos.ToString());
        Main.inst.NT.publishVirtualControl(false, fieldPos.x, fieldPos.y);
    }

    protected Vector2 pixelPoint2FieldPoint(Vector2 pos)
    {
        return new Vector2(pixel2Meter(pos.x), pixel2Meter(field.height - pos.y));
    }

    protected void onTestClick()
    {
        Main.inst.NT.getRobotPos();
        Main.inst.NT.refreshAprilTagTargetInfoRecall();
    }
}
