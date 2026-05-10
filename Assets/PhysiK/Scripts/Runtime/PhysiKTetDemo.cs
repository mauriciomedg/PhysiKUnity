using PhysiK.Unity;
using System;
using UnityEngine;

public sealed class PhysiKTetDemo : MonoBehaviour
{
    [Header("PhysiK")]
    [SerializeField] private int substeps = 5;
    [SerializeField] private bool useImplicitEuler = false;
    [SerializeField] private Vector3 gravity = new Vector3(0.0f, 100.0f, 0.0f);

    [Header("Visual")]
    [SerializeField] private float nodeRadius = 0.08f;

    private IntPtr world = IntPtr.Zero;
    private PhysiKComponentHandle tetMesh;
    private int[] nodes;
    public PhysikMaterialAsset material;
    
    private Transform[] nodeVisuals;
    private LineRenderer[] edgeVisuals;

    private Vector3 anchor0;
    private Vector3 anchor1;
    private Vector3 anchor2;

    private static readonly int[,] TetEdges =
    {
        { 0, 1 },
        { 0, 2 },
        { 0, 3 },
        { 1, 2 },
        { 1, 3 },
        { 2, 3 }
    };

    private void Awake()
    {
        world = PhysiKNative.PHYSIK_CreateWorld();

        if (world == IntPtr.Zero)
        {
            Debug.LogError("Failed to create PhysiK world.");
            enabled = false;
            return;
        }

        PhysiKNative.PHYSIK_SetSubstepCount(world, Mathf.Max(1, substeps));
        PhysiKNative.PHYSIK_SetSolverMode(world, useImplicitEuler ? 1 : 0);
        PhysiKNative.PHYSIK_SetGravity(world, gravity.x, gravity.y, gravity.z);

        CreateTet();
        CreateVisuals();

        Debug.Log("PhysiK tet demo created.");
    }

    private void CreateTet()
    {
        Vector3 origin = transform.position;

        nodes = new int[4];

        // fixed nodes: inverseMass = 0
        nodes[0] = PhysiKNative.PHYSIK_AddNode(world, origin.x + 0.0f, origin.y + 0.0f, origin.z + 0.0f);
        nodes[1] = PhysiKNative.PHYSIK_AddNode(world, origin.x + 1.0f, origin.y + 0.0f, origin.z + 0.0f);

        // dynamic nodes: inverseMass = 1
        nodes[2] = PhysiKNative.PHYSIK_AddNode(world, origin.x + 0.0f, origin.y + 0.0f, origin.z + 1.0f);
        nodes[3] = PhysiKNative.PHYSIK_AddNode(world, origin.x + 0.3f, origin.y + 1.0f, origin.z + 0.3f);

        anchor0 = new Vector3(origin.x + 0.0f, origin.y + 0.0f, origin.z + 0.0f);
        anchor1 = new Vector3(origin.x + 1.0f, origin.y + 0.0f, origin.z + 0.0f);
        anchor2 = new Vector3(origin.x + 0.0f, origin.y + 0.0f, origin.z + 1.0f);

        int[] tetNodeIndices =
        {
            nodes[0], nodes[1], nodes[2], nodes[3]
        };

        PhysikMaterialDesc nativeMaterial = material.ToNative();
        tetMesh = PhysiKNative.PHYSIK_CreateTetMeshComponentWithMaterialDesc(
            world,
            nodes,
            nodes.Length,
            tetNodeIndices,
            1,
            ref nativeMaterial);

        PhysiKNative.PHYSIK_SetNodePosition(
            world,
            nodes[3],
            origin.x + 0.3f,
            origin.y + 1.5f,
            origin.z + 0.3f);

        int valid = PhysiKNative.PHYSIK_IsComponentHandleValid(world, tetMesh);

        if (valid == 0)
        {
            Debug.LogError("Tet mesh component creation failed.");
            enabled = false;
        }
    }

    private void CreateVisuals()
    {
        nodeVisuals = new Transform[4];

        for (int i = 0; i < nodeVisuals.Length; ++i)
        {
            GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.name = $"PhysiK_Node_{i}";
            sphere.transform.localScale = Vector3.one * nodeRadius;
            nodeVisuals[i] = sphere.transform;
        }

        edgeVisuals = new LineRenderer[6];

        for (int i = 0; i < edgeVisuals.Length; ++i)
        {
            GameObject edge = new GameObject($"PhysiK_Tet_Edge_{i}");
            LineRenderer line = edge.AddComponent<LineRenderer>();

            line.positionCount = 2;
            line.widthMultiplier = 0.02f;
            line.useWorldSpace = true;

            edgeVisuals[i] = line;
        }
    }

    private void FixedUpdate()
    {
        if (world == IntPtr.Zero)
        {
            return;
        }

        AddAnchorConnection(nodes[0], anchor0);
        //AddAnchorConnection(nodes[1], anchor1);
        //AddAnchorConnection(nodes[2], anchor2);

        PhysiKNative.PHYSIK_Step(world, Time.fixedDeltaTime);
        UpdateVisuals();
    }

    private void AddAnchorConnection(int node, Vector3 target)
    {
        PhysiKNative.PHYSIK_AddPointConnection(
            world,
            node, node, node, node,
            1.0f, 0.0f, 0.0f, 0.0f,
            target.x, target.y, target.z,
            500.0f,
            5.0f);
    }

    private void UpdateVisuals()
    {
        Vector3[] positions = new Vector3[4];

        for (int i = 0; i < nodes.Length; ++i)
        {
            PhysiKNative.PHYSIK_GetNodePosition(
                world,
                nodes[i],
                out float x,
                out float y,
                out float z);

            positions[i] = new Vector3(x, y, z);
            nodeVisuals[i].position = positions[i];
        }

        for (int i = 0; i < edgeVisuals.Length; ++i)
        {
            int a = TetEdges[i, 0];
            int b = TetEdges[i, 1];

            edgeVisuals[i].SetPosition(0, positions[a]);
            edgeVisuals[i].SetPosition(1, positions[b]);
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