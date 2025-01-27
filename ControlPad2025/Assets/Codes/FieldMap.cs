using FairyGUI;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;

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
    GButton roboPosBtn;

    // Start is called before the first frame update
    void Start()
    {
        // 场地尺寸，米单位 "field_size":{"x":17.548,"y":8.052}
        // 每个grid的缩放比 "nodeSizeMeters":0.3

        

        UIPackage.AddPackage("ui/main/main");
        fieldRoot = UIPackage.CreateObject("main", "main").asCom;
        GRoot.inst.AddChild(fieldRoot);

        field = fieldRoot.GetChild("field").asCom;

        roboPosBtn = fieldRoot.GetChild("RobPosBtn").asButton;
        roboPosBtn.onClick.Set(onRoboPosClick);

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

    void onRoboPosClick()
    {
        NTManager.RobotPos pos = Main.inst.NT.getRobotPos();
        Debug.LogErrorFormat("Got robot pos: x->{0}, y->{1}, degree->{2}", pos.x, pos.y, pos.degree);
    }

    // Update is called once per frame
    void Update()
    {
        NTManager.RobotPos pos = Main.inst.NT.getRobotPos();
        if (pos != null) {
            robot.SetXY(meter2Pixel(pos.x), field.height - meter2Pixel(pos.y));
            robot.rotation = 360 - pos.degree;
        }
    }

    protected float meter2Pixel(float value) {
        return value * field.width / 17.548f;
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

}
