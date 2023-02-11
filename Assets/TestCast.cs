using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TestCast : MonoBehaviour
{
    private void Update()
    {
        if (Physics.Raycast(transform.position, -transform.up, out var hit))
        {
            Debug.Log(hit.normal);
        }
    }
}
