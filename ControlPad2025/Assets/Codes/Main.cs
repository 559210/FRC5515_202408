using System.Collections;
using UnityEngine;
using System.IO;

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
        // read file that store ip to connect.
        string contents = File.ReadAllText(@"ip.txt");
        Debug.LogError("ip: " + contents);
        _NT.init(contents);
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    void OnApplicationQuit() {
        Debug.LogError("OnApplicationQuit");
        NT.stop();
    }

    public void callCBLater(System.Action cb, float delay) {
        StartCoroutine(callCBLaterCoroutine(cb, delay));
    }

    private IEnumerator callCBLaterCoroutine(System.Action cb, float delay) {
        if (delay <= 0) {
            yield return null; // Wait for the next frame
            cb();
            yield break;
        }
        yield return new WaitForSeconds(delay);
        cb();
    }
}
