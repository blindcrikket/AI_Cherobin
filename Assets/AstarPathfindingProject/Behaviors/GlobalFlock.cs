using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GlobalFlock : MonoBehaviour
{
    public GameObject fishPrefab;
    public GameObject goalPrefab;
    public static int lakesize = 5;
    static int numfish = 10;
   
    public static GameObject[] allFish = new GameObject[numfish];

    public static Vector3 goalPos = Vector3.zero;
    // Start is called before the first frame update
    void Start()
    {
        for (int i = 0; i < numfish; i++)
        {
            Vector3 pos = new Vector3(Random.Range(-lakesize, lakesize),
                Random.Range(-lakesize, lakesize), Random.Range(-lakesize, lakesize));

            allFish[i]= (GameObject)Instantiate(fishPrefab, pos, Quaternion.identity);
        }
    }

    // Update is called once per frame
    void Update()
    {
        if (Random.Range(0, 10000) < 50)
        {
            goalPos = new Vector3(Random.Range(-lakesize, lakesize),
                Random.Range(-lakesize, lakesize), Random.Range(-lakesize, lakesize));
        }
        goalPrefab.transform.position = goalPos;
    }
}
