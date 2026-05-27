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

    [Header("Cutting Test")]
    [SerializeField] private KeyCode cutKey = KeyCode.R;
    [SerializeField] private int segmentToCut = -1;
    [SerializeField] private bool cutOnlyOnce = true;

    [Header("Visual")]
    [SerializeField] private float nodeRadius = 0.045f;
    [SerializeField] private float edgeWidth = 0.018f;

    private IntPtr world = IntPtr.Zero;
    private PhysiKComponentHandle tetMesh;

    private int[] nodes;
    private int[] leftEndLocalNodeIndices;
    private Vector3[] leftEndAnchors;

    private int[] tetNodeIndices;

    private Transform[] nodeVisuals;
    private readonly List<TetEdgeVisual> tetEdgeVisuals = new List<TetEdgeVisual>();

    private bool hasCut;

    private struct TetEdgeVisual
    {
        public int tetIndex;
        public int localNodeA;
        public int localNodeB;
        public LineRenderer line;
    }

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
            $"PhysiK beam created. FEM={femModel}, segments={beamSegments}, tets={tetNodeIndices.Length / 4}. Press {cutKey} to cut.",
            this);
    }

    private void Update()
    {
        if (Input.GetKeyDown(cutKey))
        {
            if (cutOnlyOnce && hasCut)
            {
                Debug.Log("Cut already performed.", this);
                return;
            }

            CutMiddleSegment();
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

            // Six tets per beam segment.
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

        tetMesh = PhysiKNative.PHYSIK_CreateTetMeshPhysicsComponent(
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

        if (segmentToCut < 0)
        {
            segmentToCut = beamSegments / 2;
        }

        segmentToCut = Mathf.Clamp(segmentToCut, 0, beamSegments - 1);
    }

    private void CutMiddleSegment()
    {
        if (world == IntPtr.Zero || tetNodeIndices == null || beamSegments <= 0)
        {
            return;
        }

        int totalTetCount = tetNodeIndices.Length / 4;
        int tetsPerSegment = totalTetCount / beamSegments;

        if (tetsPerSegment <= 0)
        {
            Debug.LogError("Invalid tetsPerSegment. Cannot cut beam.", this);
            return;
        }

        int segment = segmentToCut >= 0
            ? Mathf.Clamp(segmentToCut, 0, beamSegments - 1)
            : beamSegments / 2;

        int firstTet = segment * tetsPerSegment;

        int deactivated = 0;

        for (int i = 0; i < tetsPerSegment; ++i)
        {
            int tetIndex = firstTet + i;

            if (tetIndex < 0 || tetIndex >= totalTetCount)
            {
                continue;
            }

            int wasActive = PhysiKNative.PHYSIK_IsTetActive(world, tetMesh, tetIndex);

            PhysiKNative.PHYSIK_DeactivateTet(world, tetMesh, tetIndex);

            if (wasActive != 0)
            {
                ++deactivated;
            }
        }

        hasCut = true;

        int activeCount = PhysiKNative.PHYSIK_GetActiveTetCount(world, tetMesh);

        Debug.Log(
            $"CUT: segment={segment}/{beamSegments - 1}, " +
            $"tetsPerSegment={tetsPerSegment}, " +
            $"deactivated={deactivated}, " +
            $"activeTets={activeCount}/{totalTetCount}",
            this);

        UpdateVisuals();
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

        CreateTetEdgeVisuals();
    }

    private void CreateTetEdgeVisuals()
    {
        tetEdgeVisuals.Clear();

        Dictionary<int, int> globalToLocal = new Dictionary<int, int>();

        for (int i = 0; i < nodes.Length; ++i)
        {
            globalToLocal[nodes[i]] = i;
        }

        int tetCount = tetNodeIndices.Length / 4;

        for (int tet = 0; tet < tetCount; ++tet)
        {
            int baseIndex = tet * 4;

            int a = globalToLocal[tetNodeIndices[baseIndex + 0]];
            int b = globalToLocal[tetNodeIndices[baseIndex + 1]];
            int c = globalToLocal[tetNodeIndices[baseIndex + 2]];
            int d = globalToLocal[tetNodeIndices[baseIndex + 3]];

            AddTetEdgeVisual(tet, a, b);
            AddTetEdgeVisual(tet, a, c);
            AddTetEdgeVisual(tet, a, d);
            AddTetEdgeVisual(tet, b, c);
            AddTetEdgeVisual(tet, b, d);
            AddTetEdgeVisual(tet, c, d);
        }
    }

    private void AddTetEdgeVisual(int tetIndex, int localA, int localB)
    {
        GameObject edge = new GameObject($"PhysiK_Beam_Tet_{tetIndex}_Edge");
        LineRenderer line = edge.AddComponent<LineRenderer>();

        line.positionCount = 2;
        line.widthMultiplier = edgeWidth;
        line.useWorldSpace = true;

        tetEdgeVisuals.Add(new TetEdgeVisual
        {
            tetIndex = tetIndex,
            localNodeA = localA,
            localNodeB = localB,
            line = line
        });
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

    [ContextMenu("Cut Middle Segment")]
    private void CutMiddleSegmentContextMenu()
    {
        if (!Application.isPlaying)
        {
            Debug.Log("Enter Play Mode first.", this);
            return;
        }

        CutMiddleSegment();
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

        for (int i = 0; i < tetEdgeVisuals.Count; ++i)
        {
            TetEdgeVisual edge = tetEdgeVisuals[i];

            bool active = PhysiKNative.PHYSIK_IsTetActive(world, tetMesh, edge.tetIndex) != 0;

            if (edge.line != null)
            {
                edge.line.enabled = active;

                if (active)
                {
                    edge.line.SetPosition(0, positions[edge.localNodeA]);
                    edge.line.SetPosition(1, positions[edge.localNodeB]);
                }
            }
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

        for (int i = 0; i < tetEdgeVisuals.Count; ++i)
        {
            if (tetEdgeVisuals[i].line != null)
            {
                Destroy(tetEdgeVisuals[i].line.gameObject);
            }
        }

        tetEdgeVisuals.Clear();
    }
}