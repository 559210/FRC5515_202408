using FairyGUI;
using NetworkTablesSharp;
using System.Collections;
using System.Collections.Generic;
using UnityEditor.Experimental.GraphView;
using UnityEngine;

public class Main : MonoBehaviour
{
    protected static Main _inst;
    public static Main inst
    {
        get
        {
            return _inst;
        }
    }
    private void Awake()
    {
        _inst = this;
    }

    NTManager _NT = new();

    public NTManager NT
    {
        get { return _NT; }
    }

    // Start is called before the first frame update
    void Start()
    {
        _NT.init();
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    void OnApplicationQuit() {
        Debug.LogError("OnApplicationQuit");
        NT.stop();
    }
}
