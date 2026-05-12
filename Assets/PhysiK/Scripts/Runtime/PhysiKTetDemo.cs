using System;
using System.Collections.Generic;
using UnityEngine;
using PhysiK.Unity;

public sealed class PhysiKTetDemo : MonoBehaviour
{
    [Header("PhysiK")]
    [SerializeField] private int substeps = 12;
    [SerializeField] private bool useImplicitEuler = true;

    [Header("Gravity")]
    [SerializeField] private Vector3 gravity = new Vector3(0.0f, -9.81f, 0.0f);
    [SerializeField] private bool applyGravityEveryFixedUpdate = true;

    [Header("Beam Tet Mesh")]
    [SerializeField] private int beamSegments = 6;
    [SerializeField] private float beamLength = 4.0f;
    [SerializeField] private float beamHeight = 0.45f;
    [SerializeField] private float beamDepth = 0.45f;

    [Header("FEM")]
    [SerializeField] private PhysikMaterialAsset material;
    [SerializeField] private PhysiKFemModel femModel = PhysiKFemModel.Corotational;

    [Header("Left End Point Connections")]
    [SerializeField] private float anchorStiffness = 20000.0f;
    [SerializeField] private float anchorDamping = 200.0f;

    [Header("Visual")]
    [SerializeField] private float nodeRadius = 0.045f;
    [SerializeField] private float edgeWidth = 0.018f;

    private IntPtr world = IntPtr.Zero;
    private PhysiKComponentHandle tetMesh;

    private int[] nodes;
    private int[] leftEndLocalNodeIndices;
    private Vector3[] leftEndAnchors;

    private int[] tetNodeIndices;
    private int[,] visualEdges;

    private Transform[] nodeVisuals;
    private LineRenderer[] edgeVisuals;

    private void Awake()
    {
        world = PhysiKNative.PHYSIK_CreateWorld();

        if (world == IntPtr.Zero)
        {
            Debug.LogError("Failed to create PhysiK world.", this);
            enabled = false;
            return;
        }

        PhysiKNative.PHYSIK_SetSubstepCount(world, Mathf.Max(1, substeps));
        PhysiKNative.PHYSIK_SetSolverMode(world, useImplicitEuler ? 1 : 0);

        ApplyGravityToNative();

        CreateBeamTetMesh();
        CreateVisuals();
        UpdateVisuals();

        Debug.Log(
            $"PhysiK beam tet demo created. FEM={femModel}, segments={beamSegments}, tets={tetNodeIndices.Length / 4}",
            this);
    }

    private void CreateBeamTetMesh()
    {
        if (material == null)
        {
            Debug.LogError("PhysiK material is not assigned.", this);
            enabled = false;
            return;
        }

        beamSegments = Mathf.Max(1, beamSegments);

        Vector3 origin = transform.position;

        int stationCount = beamSegments + 1;
        int nodesPerStation = 4;

        nodes = new int[stationCount * nodesPerStation];

        float dx = beamLength / beamSegments;
        float halfHeight = beamHeight * 0.5f;
        float halfDepth = beamDepth * 0.5f;

        for (int station = 0; station < stationCount; ++station)
        {
            float x = station * dx;

            Vector3[] stationLocalPositions =
            {
                new Vector3(x, -halfHeight, -halfDepth), // 0 lower/back
                new Vector3(x, -halfHeight,  halfDepth), // 1 lower/front
                new Vector3(x,  halfHeight, -halfDepth), // 2 upper/back
                new Vector3(x,  halfHeight,  halfDepth), // 3 upper/front
            };

            for (int corner = 0; corner < nodesPerStation; ++corner)
            {
                int localIndex = StationNodeIndex(station, corner);
                Vector3 p = origin + stationLocalPositions[corner];

                nodes[localIndex] = PhysiKNative.PHYSIK_AddNode(world, p.x, p.y, p.z);
            }
        }

        List<int> tets = new List<int>();

        for (int segment = 0; segment < beamSegments; ++segment)
        {
            int a0 = nodes[StationNodeIndex(segment, 0)];
            int a1 = nodes[StationNodeIndex(segment, 1)];
            int a2 = nodes[StationNodeIndex(segment, 2)];
            int a3 = nodes[StationNodeIndex(segment, 3)];

            int b0 = nodes[StationNodeIndex(segment + 1, 0)];
            int b1 = nodes[StationNodeIndex(segment + 1, 1)];
            int b2 = nodes[StationNodeIndex(segment + 1, 2)];
            int b3 = nodes[StationNodeIndex(segment + 1, 3)];

            // Cube mapping:
            // old cube 0 = a0
            // old cube 1 = b0
            // old cube 2 = a1
            // old cube 3 = b1
            // old cube 4 = a2
            // old cube 5 = b2
            // old cube 6 = a3
            // old cube 7 = b3
            //
            // Six tets around diagonal a0 -> b3.
            AddTet(tets, a0, b1, b0, b3);
            AddTet(tets, a0, a1, b1, b3);
            AddTet(tets, a0, a3, a1, b3);
            AddTet(tets, a0, a2, a3, b3);
            AddTet(tets, a0, b2, a2, b3);
            AddTet(tets, a0, b0, b2, b3);
        }

        tetNodeIndices = tets.ToArray();

        PhysikMaterialDesc nativeMaterial = material.ToNative();

        Debug.Log(
            $"Creating beam TetMesh. FEM={femModel}, nodes={nodes.Length}, tets={tetNodeIndices.Length / 4}, gravity=({gravity.x:F4}, {gravity.y:F4}, {gravity.z:F4})",
            this);

        tetMesh = PhysiKNative.PHYSIK_CreateTetMeshComponent(
            world,
            nodes,
            nodes.Length,
            tetNodeIndices,
            tetNodeIndices.Length / 4,
            ref nativeMaterial,
            femModel);

        int valid = PhysiKNative.PHYSIK_IsComponentHandleValid(world, tetMesh);

        if (valid == 0)
        {
            Debug.LogError("Beam tet mesh component creation failed.", this);
            enabled = false;
            return;
        }

        // Anchor the left end of the beam using transient point connections.
        leftEndLocalNodeIndices = new[]
        {
            StationNodeIndex(0, 0),
            StationNodeIndex(0, 1),
            StationNodeIndex(0, 2),
            StationNodeIndex(0, 3)
        };

        leftEndAnchors = new Vector3[leftEndLocalNodeIndices.Length];

        for (int i = 0; i < leftEndLocalNodeIndices.Length; ++i)
        {
            int localNode = leftEndLocalNodeIndices[i];

            PhysiKNative.PHYSIK_GetNodePosition(
                world,
                nodes[localNode],
                out float x,
                out float y,
                out float z);

            leftEndAnchors[i] = new Vector3(x, y, z);
        }

        visualEdges = BuildUniqueTetEdges(tetNodeIndices, nodes);
    }

    private int StationNodeIndex(int station, int corner)
    {
        return station * 4 + corner;
    }

    private static void AddTet(List<int> tets, int n0, int n1, int n2, int n3)
    {
        tets.Add(n0);
        tets.Add(n1);
        tets.Add(n2);
        tets.Add(n3);
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
            sphere.name = $"PhysiK_Beam_Node_{i}";
            sphere.transform.localScale = Vector3.one * nodeRadius;
            nodeVisuals[i] = sphere.transform;
        }

        edgeVisuals = new LineRenderer[visualEdges.GetLength(0)];

        for (int i = 0; i < edgeVisuals.Length; ++i)
        {
            GameObject edge = new GameObject($"PhysiK_Beam_Tet_Edge_{i}");
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

        if (applyGravityEveryFixedUpdate)
        {
            ApplyGravityToNative();
        }

        // Point connections are transient: push them every physics step.
        for (int i = 0; i < leftEndLocalNodeIndices.Length; ++i)
        {
            int localNodeIndex = leftEndLocalNodeIndices[i];
            AddAnchorConnection(nodes[localNodeIndex], leftEndAnchors[i]);
        }

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
            anchorStiffness,
            anchorDamping);
    }

    private void ApplyGravityToNative()
    {
        if (world == IntPtr.Zero)
        {
            return;
        }

        PhysiKNative.PHYSIK_SetGravity(world, gravity.x, gravity.y, gravity.z);
    }

    [ContextMenu("Apply Gravity To Native")]
    private void ApplyGravityContextMenu()
    {
        if (!Application.isPlaying)
        {
            Debug.Log("Enter Play Mode first. Native world does not exist yet.", this);
            return;
        }

        ApplyGravityToNative();

        Debug.Log(
            $"Gravity applied to native: ({gravity.x:F6}, {gravity.y:F6}, {gravity.z:F6})",
            this);
    }

    [ContextMenu("Reset Gravity To Unity Default")]
    private void ResetGravityToUnityDefault()
    {
        gravity = new Vector3(0.0f, -9.81f, 0.0f);

        Debug.Log(
            $"Gravity reset to Unity default: ({gravity.x:F6}, {gravity.y:F6}, {gravity.z:F6})",
            this);
    }

    private void UpdateVisuals()
    {
        if (world == IntPtr.Zero || nodes == null || nodeVisuals == null)
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

        if (edgeVisuals == null || visualEdges == null)
        {
            return;
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

        DestroyVisuals();
    }

    private void DestroyVisuals()
    {
        if (nodeVisuals != null)
        {
            for (int i = 0; i < nodeVisuals.Length; ++i)
            {
                if (nodeVisuals[i] != null)
                {
                    Destroy(nodeVisuals[i].gameObject);
                }
            }
        }

        if (edgeVisuals != null)
        {
            for (int i = 0; i < edgeVisuals.Length; ++i)
            {
                if (edgeVisuals[i] != null)
                {
                    Destroy(edgeVisuals[i].gameObject);
                }
            }
        }
    }
}