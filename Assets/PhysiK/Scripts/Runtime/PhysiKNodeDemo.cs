using System;
using UnityEngine;

public class PhysiKNodeDemo : MonoBehaviour
{
    [Header("PhysiK")]
    [SerializeField] private int substeps = 1;
    [SerializeField] private bool useImplicitEuler = true;
    [SerializeField] private Vector3 gravity = new Vector3(0.0f, -9.81f, 0.0f);

    [Header("Unity Visual")]
    [SerializeField] private Transform visualSphere;

    private IntPtr world = IntPtr.Zero;
    private int nodeIndex = -1;

    private void Awake()
    {
        world = PhysiKNative.PHYSIK_CreateWorld();

        if (world == IntPtr.Zero)
        {
            Debug.LogError("PhysiK world creation failed.");
            enabled = false;
            return;
        }

        PhysiKNative.PHYSIK_SetSubstepCount(world, Mathf.Max(1, substeps));
        PhysiKNative.PHYSIK_SetSolverMode(world, useImplicitEuler ? 1 : 0);
        PhysiKNative.PHYSIK_SetGravity(world, gravity.x, gravity.y, gravity.z);

        nodeIndex = PhysiKNative.PHYSIK_AddNode(
            world,
            transform.position.x,
            transform.position.y,
            transform.position.z); // inverse mass = 1 means mass = 1

        Debug.Log($"PhysiK world created. Node index: {nodeIndex}");
    }

    private void FixedUpdate()
    {
        if (world == IntPtr.Zero || nodeIndex < 0)
        {
            return;
        }

        PhysiKNative.PHYSIK_Step(world, Time.fixedDeltaTime);

        PhysiKNative.PHYSIK_GetNodePosition(
            world,
            nodeIndex,
            out float x,
            out float y,
            out float z);

        Vector3 position = new Vector3(x, y, z);

        if (visualSphere != null)
        {
            visualSphere.position = position;
        }
        else
        {
            transform.position = position;
        }
    }

    private void OnDestroy()
    {
        if (world != IntPtr.Zero)
        {
            PhysiKNative.PHYSIK_DestroyWorld(world);
            world = IntPtr.Zero;
        }
    }
}
