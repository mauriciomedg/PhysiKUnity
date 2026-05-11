using System;
using System.Collections.Generic;
using UnityEngine;
using PhysiK.Unity;

public sealed class PhysiKTetDemo : MonoBehaviour
{
    [Header("PhysiK")]
    [SerializeField] private int substeps = 8;
    [SerializeField] private bool useImplicitEuler = false;

    // Unity convention: Y up, so gravity is usually negative Y.
    // If your native engine uses the opposite sign, flip this to +9.81.
    [SerializeField] private Vector3 gravity = new Vector3(0.0f, -9.81f, 0.0f);

    [Header("Cube Tet Mesh")]
    [SerializeField] private float cubeSize = 1.0f;
    [SerializeField] private PhysikMaterialAsset material;

    [Header("Bottom Anchor Point Connections")]
    [SerializeField] private float anchorStiffness = 5000.0f;
    [SerializeField] private float anchorDamping = 50.0f;

    [Header("Visual")]
    [SerializeField] private float nodeRadius = 0.06f;
    [SerializeField] private float edgeWidth = 0.02f;

    private IntPtr world = IntPtr.Zero;
    private PhysiKComponentHandle tetMesh;

    private int[] nodes;
    private int[] bottomNodeLocalIndices;
    private Vector3[] bottomAnchors;

    private int[] tetNodeIndices;
    private int[,] visualEdges;

    private Transform[] nodeVisuals;
    private LineRenderer[] edgeVisuals;

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

        CreateCubeTetMesh();
        CreateVisuals();
        UpdateVisuals();

        Debug.Log("PhysiK cube tet demo created.");
    }

    private void CreateCubeTetMesh()
    {
        if (material == null)
        {
            Debug.LogError("PhysiK material is not assigned.");
            enabled = false;
            return;
        }

        Vector3 origin = transform.position;
        float s = cubeSize;

        // Cube node layout:
        // Bottom: 0, 1, 2, 3
        // Top:    4, 5, 6, 7
        Vector3[] localPositions =
        {
            new Vector3(0.0f, 0.0f, 0.0f), // 0 bottom
            new Vector3(s,    0.0f, 0.0f), // 1 bottom
            new Vector3(0.0f, 0.0f, s),    // 2 bottom
            new Vector3(s,    0.0f, s),    // 3 bottom

            new Vector3(0.0f, s,    0.0f), // 4 top
            new Vector3(s,    s,    0.0f), // 5 top
            new Vector3(0.0f, s,    s),    // 6 top
            new Vector3(s,    s,    s),    // 7 top
        };

        nodes = new int[localPositions.Length];

        for (int i = 0; i < localPositions.Length; ++i)
        {
            Vector3 p = origin + localPositions[i];
            nodes[i] = PhysiKNative.PHYSIK_AddNode(world, p.x, p.y, p.z);
        }

        // Six positive-orientation tetrahedra filling the cube around diagonal 0 -> 7.
        // Flattened as groups of 4 node indices.
        tetNodeIndices = new[]
        {
            nodes[0], nodes[3], nodes[1], nodes[7],
            nodes[0], nodes[2], nodes[3], nodes[7],
            nodes[0], nodes[6], nodes[2], nodes[7],
            nodes[0], nodes[4], nodes[6], nodes[7],
            nodes[0], nodes[5], nodes[4], nodes[7],
            nodes[0], nodes[1], nodes[5], nodes[7],
        };

        PhysikMaterialDesc nativeMaterial = material.ToNative();

        tetMesh = PhysiKNative.PHYSIK_CreateTetMeshComponentWithMaterialDesc(
            world,
            nodes,
            nodes.Length,
            tetNodeIndices,
            tetNodeIndices.Length / 4,
            ref nativeMaterial);

        int valid = PhysiKNative.PHYSIK_IsComponentHandleValid(world, tetMesh);

        if (valid == 0)
        {
            Debug.LogError("Cube tet mesh component creation failed.");
            enabled = false;
            return;
        }

        // Anchor the whole bottom face using transient point connections every physics step.
        bottomNodeLocalIndices = new[] { 0, 1, 2, 3 };
        bottomAnchors = new Vector3[bottomNodeLocalIndices.Length];

        for (int i = 0; i < bottomNodeLocalIndices.Length; ++i)
        {
            bottomAnchors[i] = origin + localPositions[bottomNodeLocalIndices[i]];
        }

        visualEdges = BuildUniqueTetEdges(tetNodeIndices, nodes);
    }

    private static int[,] BuildUniqueTetEdges(int[] flattenedTetNodeIndices, int[] globalNodes)
    {
        Dictionary<int, int> globalToLocal = new Dictionary<int, int>();

        for (int i = 0; i < globalNodes.Length; ++i)
        {
            globalToLocal[globalNodes[i]] = i;
        }

        HashSet<(int, int)> uniqueEdges = new HashSet<(int, int)>();

        for (int t = 0; t < flattenedTetNodeIndices.Length; t += 4)
        {
            int a = globalToLocal[flattenedTetNodeIndices[t + 0]];
            int b = globalToLocal[flattenedTetNodeIndices[t + 1]];
            int c = globalToLocal[flattenedTetNodeIndices[t + 2]];
            int d = globalToLocal[flattenedTetNodeIndices[t + 3]];

            AddEdge(uniqueEdges, a, b);
            AddEdge(uniqueEdges, a, c);
            AddEdge(uniqueEdges, a, d);
            AddEdge(uniqueEdges, b, c);
            AddEdge(uniqueEdges, b, d);
            AddEdge(uniqueEdges, c, d);
        }

        int[,] edges = new int[uniqueEdges.Count, 2];
        int index = 0;

        foreach ((int a, int b) in uniqueEdges)
        {
            edges[index, 0] = a;
            edges[index, 1] = b;
            ++index;
        }

        return edges;
    }

    private static void AddEdge(HashSet<(int, int)> edges, int a, int b)
    {
        if (a > b)
        {
            (a, b) = (b, a);
        }

        edges.Add((a, b));
    }

    private void CreateVisuals()
    {
        nodeVisuals = new Transform[nodes.Length];

        for (int i = 0; i < nodeVisuals.Length; ++i)
        {
            GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.name = $"PhysiK_Cube_Node_{i}";
            sphere.transform.localScale = Vector3.one * nodeRadius;
            nodeVisuals[i] = sphere.transform;
        }

        edgeVisuals = new LineRenderer[visualEdges.GetLength(0)];

        for (int i = 0; i < edgeVisuals.Length; ++i)
        {
            GameObject edge = new GameObject($"PhysiK_Cube_Tet_Edge_{i}");
            LineRenderer line = edge.AddComponent<LineRenderer>();

            line.positionCount = 2;
            line.widthMultiplier = edgeWidth;
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

        // Point connections are transient: push them every physics step.
        for (int i = 0; i < bottomNodeLocalIndices.Length; ++i)
        {
            int localNodeIndex = bottomNodeLocalIndices[i];
            AddAnchorConnection(nodes[localNodeIndex], bottomAnchors[i]);
        }

        PhysiKNative.PHYSIK_Step(world, Time.fixedDeltaTime);
        UpdateVisuals();
    }

    private void AddAnchorConnection(int node, Vector3 target)
    {
        // For a direct node anchor, use the same node four times and barycentric (1, 0, 0, 0).
        PhysiKNative.PHYSIK_AddPointConnection(
            world,
            node, node, node, node,
            1.0f, 0.0f, 0.0f, 0.0f,
            target.x, target.y, target.z,
            anchorStiffness,
            anchorDamping);
    }

    private void UpdateVisuals()
    {
        if (nodes == null || nodeVisuals == null)
        {
            return;
        }

        Vector3[] positions = new Vector3[nodes.Length];

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
            int a = visualEdges[i, 0];
            int b = visualEdges[i, 1];

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
