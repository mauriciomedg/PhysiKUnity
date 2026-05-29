using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using PhysiK.Unity;

public class Physik_MechanicalTissue : MonoBehaviour
{
    [Header("PhysiK")]
    [SerializeField] private int substeps = 8;
    [SerializeField] private bool useImplicitEuler = true;

    [Header("Simulation Loop")]
    [SerializeField] private float simulationDt = 1.0f / 30.0f;
    [SerializeField] private int maxSimulationStepsPerFrame = 1;

    [Header("Gravity")]
    [SerializeField]
    private Vector3 gravity =
        new Vector3(0.0f, -9.81f, 0.0f);

    [SerializeField] private bool applyGravityEveryStep = true;

    [Header("Radial Circular Tissue Mesh")]
    [SerializeField] private int radialSegments = 6;
    [SerializeField] private int angularSegments = 30;
    [SerializeField] private float radius = 2.0f;
    [SerializeField] private float thickness = 0.12f;

    [Header("FEM")]
    [SerializeField] private PhysikMaterialAsset material;

    [Header("Boundary Point Connections")]
    [SerializeField] private float boundaryStiffness = 20000.0f;
    [SerializeField] private float boundaryDamping = 0.0f;
    [SerializeField] private float radialBoundaryOffset = 0.15f;

    [Header("Runtime Cutting")]
    [SerializeField] private int randomSeed = 12345;
    [SerializeField] private bool avoidBoundaryTetsWhenCutting = true;

    [Header("Visual Mesh")]
    [SerializeField] private bool drawSurface = true;
    [SerializeField] private Material surfaceMaterial;
    [SerializeField] private bool doubleSidedSurface = true;

    [SerializeField] private bool drawWireframe = true;
    [SerializeField] private Material wireframeMaterial;

    [SerializeField] private bool drawBoundaryMarkers = true;
    [SerializeField] private Material boundaryMarkerMaterial;
    [SerializeField] private float boundaryMarkerRadius = 0.035f;

    [Header("Point Connection Lines")]
    [SerializeField] private bool drawPointConnectionLines = true;
    [SerializeField] private Material pointConnectionLineMaterial;

    private bool initialized;

    private IntPtr world = IntPtr.Zero;
    private PhysiKComponentHandle tetMesh;

    // Generated local node index -> World global node index.
    private int[] nodes;

    // Existing debug-draw code still expects global node indices.
    private int[] tetNodeIndices;

    // Current world positions indexed by generated local node index.
    private Vector3[] nodeWorldPositions;

    private int[] boundaryLocalNodeIndices;
    private bool[] boundaryLocalNodeMask;
    private Vector3[] boundaryAnchorPositions;

    private readonly Dictionary<int, int> globalToLocalNode =
        new Dictionary<int, int>();

    private System.Random random;
    private float simulationAccumulator;
    private bool topologyDirty = true;

    private GameObject surfaceObject;
    private Mesh surfaceMesh;
    private MeshFilter surfaceMeshFilter;
    private MeshRenderer surfaceMeshRenderer;
    private int[] surfaceTriangles;

    private GameObject wireframeObject;
    private Mesh wireframeMesh;
    private MeshFilter wireframeMeshFilter;
    private MeshRenderer wireframeMeshRenderer;
    private int[] wireframeLineIndices;

    private GameObject boundaryMarkerObject;
    private Mesh boundaryMarkerMesh;
    private MeshFilter boundaryMarkerMeshFilter;
    private MeshRenderer boundaryMarkerMeshRenderer;
    private int[] boundaryMarkerTriangles;

    private GameObject pointConnectionLineObject;
    private Mesh pointConnectionLineMesh;
    private MeshFilter pointConnectionLineMeshFilter;
    private MeshRenderer pointConnectionLineMeshRenderer;
    private int[] pointConnectionLineIndices;
    private Vector3[] pointConnectionLineVertices;

    public bool IsInitialized => initialized;

    public IntPtr WorldHandle => world;

    public PhysiKComponentHandle TetMeshHandle => tetMesh;

    public float TissuePlaneY => transform.position.y;

    private readonly struct FaceKey : IEquatable<FaceKey>
    {
        public readonly int a;
        public readonly int b;
        public readonly int c;

        public FaceKey(int n0, int n1, int n2)
        {
            if (n0 > n1)
            {
                (n0, n1) = (n1, n0);
            }

            if (n1 > n2)
            {
                (n1, n2) = (n2, n1);
            }

            if (n0 > n1)
            {
                (n0, n1) = (n1, n0);
            }

            a = n0;
            b = n1;
            c = n2;
        }

        public bool Equals(FaceKey other)
        {
            return a == other.a &&
                b == other.b &&
                c == other.c;
        }

        public override bool Equals(object obj)
        {
            return obj is FaceKey other &&
                Equals(other);
        }

        public override int GetHashCode()
        {
            unchecked
            {
                int hash = a;
                hash = (hash * 397) ^ b;
                hash = (hash * 397) ^ c;
                return hash;
            }
        }
    }

    private readonly struct Face
    {
        public readonly int a;
        public readonly int b;
        public readonly int c;

        public Face(int a, int b, int c)
        {
            this.a = a;
            this.b = b;
            this.c = c;
        }
    }

    private void Awake()
    {
        random = new System.Random(randomSeed);

        world = PhysiKNative.PHYSIK_CreateWorld();

        if (world == IntPtr.Zero)
        {
            Debug.LogError("Failed to create PhysiK world.", this);
            enabled = false;
            return;
        }

        PhysiKNative.PHYSIK_SetSubstepCount(
            world,
            Mathf.Max(1, substeps));

        PhysiKNative.PHYSIK_SetSolverMode(
            world,
            useImplicitEuler ? 1 : 0);

        ApplyGravityToNative();

        if (!CreateRadialCircularTissueTetMesh())
        {
            enabled = false;
            return;
        }

        if (drawSurface)
        {
            CreateSurfaceVisual();
            RebuildSurfaceTopology();
            UpdateSurfaceVertices();
        }

        if (drawWireframe)
        {
            CreateWireframeVisual();
            RebuildWireframeTopology();
            UpdateWireframeVertices();
        }

        if (drawBoundaryMarkers)
        {
            CreateBoundaryMarkerVisual();
            RebuildBoundaryMarkerTopology();
            UpdateBoundaryMarkerVertices();
        }

        if (drawPointConnectionLines)
        {
            CreatePointConnectionLineVisual();
            RebuildPointConnectionLineTopology();
            UpdatePointConnectionLineVertices();
        }

        int totalTetCount = tetNodeIndices.Length / 4;

        int activeTetCount =
            PhysiKNative.PHYSIK_GetActiveTetCount(
                world,
                tetMesh);

        Debug.Log(
            $"Radial tissue created. " +
            $"nodes={nodes.Length}, " +
            $"totalTets={totalTetCount}, " +
            $"activeTets={activeTetCount}, " +
            $"boundaryNodes={boundaryLocalNodeIndices.Length}, " +
            $"radialSegments={radialSegments}, " +
            $"angularSegments={angularSegments}. " +
            $"Press R to remove one random interior tet.",
            this);

        initialized = true;
    }

    private void Update()
    {
        if (world == IntPtr.Zero)
        {
            return;
        }

        if (Keyboard.current != null &&
            Keyboard.current.rKey.wasPressedThisFrame)
        {
            RemoveOneRandomTet();
        }

        simulationAccumulator += Time.deltaTime;

        int steps = 0;

        while (simulationAccumulator >= simulationDt &&
               steps < maxSimulationStepsPerFrame)
        {
            StepSimulation(simulationDt);

            simulationAccumulator -= simulationDt;
            ++steps;
        }

        if (steps == maxSimulationStepsPerFrame)
        {
            simulationAccumulator = 0.0f;
        }

        UpdateNodeWorldPositions();

        if (drawSurface)
        {
            if (topologyDirty)
            {
                RebuildSurfaceTopology();
            }

            UpdateSurfaceVertices();
        }

        if (drawWireframe)
        {
            if (topologyDirty)
            {
                RebuildWireframeTopology();
            }

            UpdateWireframeVertices();
        }

        if (drawBoundaryMarkers)
        {
            UpdateBoundaryMarkerVertices();
        }

        if (drawPointConnectionLines)
        {
            UpdatePointConnectionLineVertices();
        }

        topologyDirty = false;
    }

    private void StepSimulation(float dt)
    {
        if (applyGravityEveryStep)
        {
            ApplyGravityToNative();
        }

        AddBoundaryPointConnections();

        PhysiKNative.PHYSIK_Step(
            world,
            dt);
    }

    private bool CreateRadialCircularTissueTetMesh()
    {
        if (material == null)
        {
            Debug.LogError(
                "PhysiK material is not assigned.",
                this);

            return false;
        }

        radialSegments = Mathf.Max(1, radialSegments);
        angularSegments = Mathf.Max(8, angularSegments);
        radius = Mathf.Max(0.01f, radius);
        thickness = Mathf.Max(0.001f, thickness);

        Vector3 origin = transform.position;
        float halfThickness = thickness * 0.5f;

        List<Vector3> rawPositions =
            new List<Vector3>();

        List<int> rawTetLocalNodeIndices =
            new List<int>();

        List<(int a, int b, int c)> triangles2D =
            new List<(int a, int b, int c)>();

        int[,] bottomRingNodes =
            new int[radialSegments + 1, angularSegments];

        int[,] topRingNodes =
            new int[radialSegments + 1, angularSegments];

        int CreateLocalNode(Vector3 worldPosition)
        {
            int localIndex = rawPositions.Count;

            rawPositions.Add(worldPosition);

            return localIndex;
        }

        int bottomCenter =
            CreateLocalNode(
                origin +
                new Vector3(
                    0.0f,
                    -halfThickness,
                    0.0f));

        int topCenter =
            CreateLocalNode(
                origin +
                new Vector3(
                    0.0f,
                    halfThickness,
                    0.0f));

        for (int ring = 1;
             ring <= radialSegments;
             ++ring)
        {
            float r =
                radius *
                ring /
                radialSegments;

            for (int segment = 0;
                 segment < angularSegments;
                 ++segment)
            {
                float angle =
                    2.0f *
                    Mathf.PI *
                    segment /
                    angularSegments;

                float x =
                    Mathf.Cos(angle) *
                    r;

                float z =
                    Mathf.Sin(angle) *
                    r;

                bottomRingNodes[ring, segment] =
                    CreateLocalNode(
                        origin +
                        new Vector3(
                            x,
                            -halfThickness,
                            z));

                topRingNodes[ring, segment] =
                    CreateLocalNode(
                        origin +
                        new Vector3(
                            x,
                            halfThickness,
                            z));
            }
        }

        // Center fan.
        for (int segment = 0;
             segment < angularSegments;
             ++segment)
        {
            int next =
                (segment + 1) %
                angularSegments;

            int a =
                bottomCenter;

            int b =
                bottomRingNodes[1, segment];

            int c =
                bottomRingNodes[1, next];

            triangles2D.Add((a, b, c));
        }

        // Ring bands. Each radial quad becomes two triangles.
        for (int ring = 1;
             ring < radialSegments;
             ++ring)
        {
            for (int segment = 0;
                 segment < angularSegments;
                 ++segment)
            {
                int next =
                    (segment + 1) %
                    angularSegments;

                int inner0 =
                    bottomRingNodes[ring, segment];

                int inner1 =
                    bottomRingNodes[ring, next];

                int outer0 =
                    bottomRingNodes[ring + 1, segment];

                int outer1 =
                    bottomRingNodes[ring + 1, next];

                triangles2D.Add(
                    (inner0, outer0, outer1));

                triangles2D.Add(
                    (inner0, outer1, inner1));
            }
        }

        int GetTopLocalNodeFromBottomLocalNode(
            int bottomLocalNode)
        {
            if (bottomLocalNode == bottomCenter)
            {
                return topCenter;
            }

            // Bottom and top nodes are created in pairs.
            return bottomLocalNode + 1;
        }

        foreach ((int a, int b, int c) in triangles2D)
        {
            int at =
                GetTopLocalNodeFromBottomLocalNode(a);

            int bt =
                GetTopLocalNodeFromBottomLocalNode(b);

            int ct =
                GetTopLocalNodeFromBottomLocalNode(c);

            AddPrismTets(
                rawTetLocalNodeIndices,
                rawPositions,
                a,
                b,
                c,
                at,
                bt,
                ct);
        }

        Vec3[] nativeRawPositions =
            new Vec3[rawPositions.Count];

        for (int i = 0;
             i < rawPositions.Count;
             ++i)
        {
            Vector3 position =
                rawPositions[i];

            nativeRawPositions[i] =
                new Vec3
                {
                    x = position.x,
                    y = position.y,
                    z = position.z
                };
        }

        int[] rawTets =
            rawTetLocalNodeIndices.ToArray();

        PhysiKGeneratedTetMeshHandle generatedTetMesh =
            PhysiKNative.PHYSIK_GenerateTetMesh(
                nativeRawPositions,
                nativeRawPositions.Length,
                rawTets,
                rawTets.Length / 4);

        if (PhysiKNative.PHYSIK_IsGeneratedTetMeshHandleValid(
                generatedTetMesh) == 0)
        {
            Debug.LogError(
                "Failed to generate clean tet mesh.",
                this);

            return false;
        }

        try
        {
            if (!TryReadGeneratedTetMesh(
                    generatedTetMesh,
                    out Vector3[] generatedPositions,
                    out int[] generatedTetLocalNodeIndices))
            {
                Debug.LogError(
                    "Failed to read generated tet mesh.",
                    this);

                return false;
            }

            PhysikMaterialDesc nativeMaterial =
                material.ToNative();

            tetMesh =
                PhysiKNative.PHYSIK_CreateTetMeshPhysicsComponent(
                    world,
                    generatedTetMesh,
                    ref nativeMaterial);

            if (PhysiKNative.PHYSIK_IsComponentHandleValid(
                    world,
                    tetMesh) == 0)
            {
                Debug.LogError(
                    "Radial circular tissue TetMesh physics component creation failed.",
                    this);

                return false;
            }

            nodeWorldPositions =
                generatedPositions;

            int globalNodeCount =
    PhysiKNative.PHYSIK_GetTetMeshGlobalNodeCount(
        world,
        tetMesh);

            if (globalNodeCount <= 0)
            {
                Debug.LogError("Tet mesh physics component has no global nodes.", this);
                return false;
            }

            nodes = new int[globalNodeCount];

            for (int localNode = 0;
                 localNode < globalNodeCount;
                 ++localNode)
            {
                nodes[localNode] =
                    PhysiKNative.PHYSIK_GetTetMeshGlobalNodeIndex(
                        world,
                        tetMesh,
                        localNode);

                if (nodes[localNode] < 0)
                {
                    Debug.LogError(
                        $"Failed to resolve global node for local node {localNode}.",
                        this);

                    return false;
                }
            }

            // The legacy debug rendering in this script still expects
            // global node IDs inside tetNodeIndices.
            tetNodeIndices =
                new int[generatedTetLocalNodeIndices.Length];

            for (int i = 0;
                 i < generatedTetLocalNodeIndices.Length;
                 ++i)
            {
                int localNode =
                    generatedTetLocalNodeIndices[i];

                if (localNode < 0 ||
                    localNode >= nodes.Length)
                {
                    Debug.LogError(
                        $"Generated tet contains invalid local node index {localNode}.",
                        this);

                    return false;
                }

                tetNodeIndices[i] =
                    nodes[localNode];
            }

            globalToLocalNode.Clear();

            for (int localNode = 0;
                 localNode < nodes.Length;
                 ++localNode)
            {
                globalToLocalNode[nodes[localNode]] =
                    localNode;
            }

            BuildRadialBoundaryNodes(
                generatedPositions,
                origin);

            int totalTetCount =
                tetNodeIndices.Length /
                4;

            int activeTetCount =
                PhysiKNative.PHYSIK_GetActiveTetCount(
                    world,
                    tetMesh);

            Debug.Log(
                $"Tissue generated and registered. " +
                $"rawNodes={rawPositions.Count}, " +
                $"generatedNodes={nodes.Length}, " +
                $"rawTets={rawTets.Length / 4}, " +
                $"generatedTets={totalTetCount}, " +
                $"activeTets={activeTetCount}, " +
                $"2DTriangles={triangles2D.Count}.",
                this);

            return true;
        }
        finally
        {
            PhysiKNative.PHYSIK_DestroyGeneratedTetMesh(
                generatedTetMesh);
        }
    }

    private static bool TryReadGeneratedTetMesh(
        PhysiKGeneratedTetMeshHandle generatedTetMesh,
        out Vector3[] generatedPositions,
        out int[] generatedTetLocalNodeIndices)
    {
        generatedPositions =
            Array.Empty<Vector3>();

        generatedTetLocalNodeIndices =
            Array.Empty<int>();

        int vertexCount =
            PhysiKNative.PHYSIK_GetGeneratedTetMeshVertexCount(
                generatedTetMesh);

        int tetIndexCount =
            PhysiKNative.PHYSIK_GetGeneratedTetMeshTetIndexCount(
                generatedTetMesh);

        if (vertexCount <= 0 ||
            tetIndexCount <= 0 ||
            tetIndexCount % 4 != 0)
        {
            return false;
        }

        generatedPositions =
            new Vector3[vertexCount];

        generatedTetLocalNodeIndices =
            new int[tetIndexCount];

        for (int vertexIndex = 0;
             vertexIndex < vertexCount;
             ++vertexIndex)
        {
            int ok =
                PhysiKNative.PHYSIK_GetGeneratedTetMeshVertex(
                    generatedTetMesh,
                    vertexIndex,
                    out float x,
                    out float y,
                    out float z);

            if (ok == 0)
            {
                return false;
            }

            generatedPositions[vertexIndex] =
                new Vector3(
                    x,
                    y,
                    z);
        }

        for (int index = 0;
             index < tetIndexCount;
             ++index)
        {
            int ok =
                PhysiKNative.PHYSIK_GetGeneratedTetMeshTetNodeIndex(
                    generatedTetMesh,
                    index,
                    out int nodeIndex);

            if (ok == 0)
            {
                return false;
            }

            generatedTetLocalNodeIndices[index] =
                nodeIndex;
        }

        return true;
    }

    private void BuildRadialBoundaryNodes(
        Vector3[] positions,
        Vector3 center)
    {
        List<int> boundary =
            new List<int>();

        float tolerance =
            Mathf.Max(
                1.0e-4f,
                radius * 1.0e-4f);

        for (int localNode = 0;
             localNode < positions.Length;
             ++localNode)
        {
            Vector3 position =
                positions[localNode];

            float radialDistance =
                new Vector2(
                    position.x - center.x,
                    position.z - center.z)
                .magnitude;

            if (Mathf.Abs(
                    radialDistance -
                    radius) <= tolerance)
            {
                boundary.Add(localNode);
            }
        }

        boundaryLocalNodeIndices =
            boundary.ToArray();

        Array.Sort(
            boundaryLocalNodeIndices);

        boundaryLocalNodeMask =
            new bool[positions.Length];

        boundaryAnchorPositions =
            new Vector3[boundaryLocalNodeIndices.Length];

        for (int i = 0;
             i < boundaryLocalNodeIndices.Length;
             ++i)
        {
            int localNode =
                boundaryLocalNodeIndices[i];

            boundaryLocalNodeMask[localNode] =
                true;

            Vector3 originalPosition =
                positions[localNode];

            Vector3 radial =
                new Vector3(
                    originalPosition.x - center.x,
                    0.0f,
                    originalPosition.z - center.z);

            if (radial.sqrMagnitude > 1.0e-8f)
            {
                radial.Normalize();
            }
            else
            {
                radial =
                    Vector3.zero;
            }

            boundaryAnchorPositions[i] =
                originalPosition +
                radial *
                radialBoundaryOffset;
        }
    }

    private static void AddPrismTets(
        List<int> tets,
        List<Vector3> positions,
        int a,
        int b,
        int c,
        int at,
        int bt,
        int ct)
    {
        // Triangular prism split into 3 tetrahedra.
        AddTetPositive(
            tets,
            positions,
            a,
            b,
            c,
            at);

        AddTetPositive(
            tets,
            positions,
            b,
            bt,
            c,
            at);

        AddTetPositive(
            tets,
            positions,
            c,
            bt,
            ct,
            at);
    }

    private static void AddTetPositive(
        List<int> tets,
        List<Vector3> positions,
        int n0,
        int n1,
        int n2,
        int n3)
    {
        float signedVolume6 =
            Vector3.Dot(
                Vector3.Cross(
                    positions[n1] -
                    positions[n0],
                    positions[n2] -
                    positions[n0]),
                positions[n3] -
                positions[n0]);

        if (Mathf.Abs(signedVolume6) <
            1.0e-10f)
        {
            return;
        }

        if (signedVolume6 < 0.0f)
        {
            (n1, n2) =
                (n2, n1);
        }

        // Store local generated-mesh indices.
        tets.Add(n0);
        tets.Add(n1);
        tets.Add(n2);
        tets.Add(n3);
    }

    private void AddBoundaryPointConnections()
    {
        if (boundaryLocalNodeIndices == null ||
            boundaryAnchorPositions == null)
        {
            return;
        }

        for (int i = 0;
             i < boundaryLocalNodeIndices.Length;
             ++i)
        {
            int localNodeIndex =
                boundaryLocalNodeIndices[i];

            int globalNodeIndex =
                nodes[localNodeIndex];

            Vector3 target =
                boundaryAnchorPositions[i];

            PhysiKNative.PHYSIK_AddPointConnection(
                world,
                globalNodeIndex,
                globalNodeIndex,
                globalNodeIndex,
                globalNodeIndex,
                1.0f,
                0.0f,
                0.0f,
                0.0f,
                target.x,
                target.y,
                target.z,
                boundaryStiffness,
                boundaryDamping);
        }
    }

    private bool TetTouchesBoundary(int tetIndex)
    {
        if (tetNodeIndices == null ||
            boundaryLocalNodeMask == null)
        {
            return true;
        }

        int baseIndex =
            tetIndex *
            4;

        if (baseIndex < 0 ||
            baseIndex + 3 >= tetNodeIndices.Length)
        {
            return true;
        }

        for (int i = 0;
             i < 4;
             ++i)
        {
            int globalNode =
                tetNodeIndices[baseIndex + i];

            if (!globalToLocalNode.TryGetValue(
                    globalNode,
                    out int localNode))
            {
                return true;
            }

            if (boundaryLocalNodeMask[localNode])
            {
                return true;
            }
        }

        return false;
    }

    public bool DeactivateTet(int tetIndex)
    {
        if (world == IntPtr.Zero ||
            tetNodeIndices == null)
        {
            return false;
        }

        if (tetIndex < 0 ||
            tetIndex >= tetNodeIndices.Length / 4)
        {
            return false;
        }

        if (PhysiKNative.PHYSIK_IsTetActive(
                world,
                tetMesh,
                tetIndex) == 0)
        {
            return false;
        }

        PhysiKNative.PHYSIK_DeactivateTet(
            world,
            tetMesh,
            tetIndex);

        topologyDirty = true;

        return true;
    }

    private void RemoveOneRandomTet()
    {
        if (world == IntPtr.Zero ||
            tetNodeIndices == null)
        {
            return;
        }

        int totalTetCount =
            tetNodeIndices.Length /
            4;

        if (totalTetCount <= 0)
        {
            return;
        }

        int activeBefore =
            PhysiKNative.PHYSIK_GetActiveTetCount(
                world,
                tetMesh);

        if (activeBefore <= 0)
        {
            Debug.Log(
                "No active tets left to remove.",
                this);

            return;
        }

        int selectedTet =
            -1;

        for (int attempt = 0;
             attempt < 128;
             ++attempt)
        {
            int candidate =
                random.Next(
                    0,
                    totalTetCount);

            if (PhysiKNative.PHYSIK_IsTetActive(
                    world,
                    tetMesh,
                    candidate) != 0 &&
                (!avoidBoundaryTetsWhenCutting ||
                 !TetTouchesBoundary(candidate)))
            {
                selectedTet =
                    candidate;

                break;
            }
        }

        if (selectedTet < 0)
        {
            for (int tet = 0;
                 tet < totalTetCount;
                 ++tet)
            {
                if (PhysiKNative.PHYSIK_IsTetActive(
                        world,
                        tetMesh,
                        tet) != 0 &&
                    (!avoidBoundaryTetsWhenCutting ||
                     !TetTouchesBoundary(tet)))
                {
                    selectedTet =
                        tet;

                    break;
                }
            }
        }

        if (selectedTet < 0)
        {
            Debug.Log(
                "Could not find an active interior tet to remove.",
                this);

            return;
        }

        PhysiKNative.PHYSIK_DeactivateTet(
            world,
            tetMesh,
            selectedTet);

        int activeAfter =
            PhysiKNative.PHYSIK_GetActiveTetCount(
                world,
                tetMesh);

        int removedCount =
            activeBefore -
            activeAfter;

        topologyDirty = true;

        Debug.Log(
            $"Removed random tet {selectedTet}. " +
            $"Removed={removedCount}, " +
            $"activeTets={activeAfter}, " +
            $"removedTets={totalTetCount - activeAfter}, " +
            $"totalTets={totalTetCount}",
            this);
    }

    private void UpdateNodeWorldPositions()
    {
        if (nodes == null ||
            nodeWorldPositions == null)
        {
            return;
        }

        for (int i = 0;
             i < nodes.Length;
             ++i)
        {
            PhysiKNative.PHYSIK_GetNodePosition(
                world,
                nodes[i],
                out float x,
                out float y,
                out float z);

            nodeWorldPositions[i] =
                new Vector3(
                    x,
                    y,
                    z);
        }
    }

    private void CreateSurfaceVisual()
    {
        surfaceObject =
            new GameObject(
                "PhysiK_Tissue_Surface");

        surfaceObject.transform.SetPositionAndRotation(
            Vector3.zero,
            Quaternion.identity);

        surfaceMeshFilter =
            surfaceObject.AddComponent<MeshFilter>();

        surfaceMeshRenderer =
            surfaceObject.AddComponent<MeshRenderer>();

        surfaceMesh =
            new Mesh
            {
                name =
                    "PhysiK_Tissue_Surface_Mesh",

                indexFormat =
                    UnityEngine.Rendering.IndexFormat.UInt32
            };

        surfaceMesh.MarkDynamic();

        surfaceMeshFilter.sharedMesh =
            surfaceMesh;

        if (surfaceMaterial != null)
        {
            surfaceMeshRenderer.sharedMaterial =
                surfaceMaterial;
        }
    }

    private void RebuildSurfaceTopology()
    {
        if (surfaceMesh == null ||
            tetNodeIndices == null ||
            nodes == null)
        {
            return;
        }

        Dictionary<FaceKey, int> faceCounts =
            new Dictionary<FaceKey, int>();

        Dictionary<FaceKey, Face> faceRepresentatives =
            new Dictionary<FaceKey, Face>();

        int tetCount =
            tetNodeIndices.Length /
            4;

        for (int tet = 0;
             tet < tetCount;
             ++tet)
        {
            if (PhysiKNative.PHYSIK_IsTetActive(
                    world,
                    tetMesh,
                    tet) == 0)
            {
                continue;
            }

            int baseIndex =
                tet *
                4;

            int a =
                globalToLocalNode[
                    tetNodeIndices[baseIndex + 0]];

            int b =
                globalToLocalNode[
                    tetNodeIndices[baseIndex + 1]];

            int c =
                globalToLocalNode[
                    tetNodeIndices[baseIndex + 2]];

            int d =
                globalToLocalNode[
                    tetNodeIndices[baseIndex + 3]];

            AddFace(
                faceCounts,
                faceRepresentatives,
                a,
                b,
                c);

            AddFace(
                faceCounts,
                faceRepresentatives,
                a,
                b,
                d);

            AddFace(
                faceCounts,
                faceRepresentatives,
                a,
                c,
                d);

            AddFace(
                faceCounts,
                faceRepresentatives,
                b,
                c,
                d);
        }

        List<int> triangles =
            new List<int>();

        foreach (KeyValuePair<FaceKey, int> pair
                 in faceCounts)
        {
            if (pair.Value != 1)
            {
                continue;
            }

            Face face =
                faceRepresentatives[pair.Key];

            triangles.Add(face.a);
            triangles.Add(face.b);
            triangles.Add(face.c);

            if (doubleSidedSurface)
            {
                triangles.Add(face.a);
                triangles.Add(face.c);
                triangles.Add(face.b);
            }
        }

        surfaceTriangles =
            triangles.ToArray();

        surfaceMesh.Clear();

        surfaceMesh.vertices =
            nodeWorldPositions;

        surfaceMesh.triangles =
            surfaceTriangles;

        surfaceMesh.RecalculateNormals();
        surfaceMesh.RecalculateBounds();
    }

    private static void AddFace(
        Dictionary<FaceKey, int> faceCounts,
        Dictionary<FaceKey, Face> faceRepresentatives,
        int a,
        int b,
        int c)
    {
        FaceKey key =
            new FaceKey(
                a,
                b,
                c);

        if (faceCounts.TryGetValue(
                key,
                out int count))
        {
            faceCounts[key] =
                count +
                1;
        }
        else
        {
            faceCounts.Add(
                key,
                1);

            faceRepresentatives.Add(
                key,
                new Face(
                    a,
                    b,
                    c));
        }
    }

    private void UpdateSurfaceVertices()
    {
        if (surfaceMesh == null ||
            nodeWorldPositions == null)
        {
            return;
        }

        surfaceMesh.vertices =
            nodeWorldPositions;

        if (surfaceTriangles != null)
        {
            surfaceMesh.triangles =
                surfaceTriangles;
        }

        surfaceMesh.RecalculateNormals();
        surfaceMesh.RecalculateBounds();
    }

    private void CreateWireframeVisual()
    {
        wireframeObject =
            new GameObject(
                "PhysiK_Tissue_Wireframe");

        wireframeObject.transform.SetPositionAndRotation(
            Vector3.zero,
            Quaternion.identity);

        wireframeMeshFilter =
            wireframeObject.AddComponent<MeshFilter>();

        wireframeMeshRenderer =
            wireframeObject.AddComponent<MeshRenderer>();

        wireframeMesh =
            new Mesh
            {
                name =
                    "PhysiK_Tissue_Wireframe_Mesh",

                indexFormat =
                    UnityEngine.Rendering.IndexFormat.UInt32
            };

        wireframeMesh.MarkDynamic();

        wireframeMeshFilter.sharedMesh =
            wireframeMesh;

        if (wireframeMaterial != null)
        {
            wireframeMeshRenderer.sharedMaterial =
                wireframeMaterial;
        }
    }

    private void RebuildWireframeTopology()
    {
        if (wireframeMesh == null ||
            tetNodeIndices == null ||
            nodes == null)
        {
            return;
        }

        HashSet<(int, int)> uniqueEdges =
            new HashSet<(int, int)>();

        int tetCount =
            tetNodeIndices.Length /
            4;

        for (int tet = 0;
             tet < tetCount;
             ++tet)
        {
            if (PhysiKNative.PHYSIK_IsTetActive(
                    world,
                    tetMesh,
                    tet) == 0)
            {
                continue;
            }

            int baseIndex =
                tet *
                4;

            int a =
                globalToLocalNode[
                    tetNodeIndices[baseIndex + 0]];

            int b =
                globalToLocalNode[
                    tetNodeIndices[baseIndex + 1]];

            int c =
                globalToLocalNode[
                    tetNodeIndices[baseIndex + 2]];

            int d =
                globalToLocalNode[
                    tetNodeIndices[baseIndex + 3]];

            AddWireEdge(
                uniqueEdges,
                a,
                b);

            AddWireEdge(
                uniqueEdges,
                a,
                c);

            AddWireEdge(
                uniqueEdges,
                a,
                d);

            AddWireEdge(
                uniqueEdges,
                b,
                c);

            AddWireEdge(
                uniqueEdges,
                b,
                d);

            AddWireEdge(
                uniqueEdges,
                c,
                d);
        }

        List<int> lines =
            new List<int>(
                uniqueEdges.Count *
                2);

        foreach ((int a, int b) in uniqueEdges)
        {
            lines.Add(a);
            lines.Add(b);
        }

        wireframeLineIndices =
            lines.ToArray();

        wireframeMesh.Clear();

        wireframeMesh.vertices =
            nodeWorldPositions;

        wireframeMesh.SetIndices(
            wireframeLineIndices,
            MeshTopology.Lines,
            0);

        wireframeMesh.RecalculateBounds();
    }

    private static void AddWireEdge(
        HashSet<(int, int)> edges,
        int a,
        int b)
    {
        if (a > b)
        {
            (a, b) =
                (b, a);
        }

        edges.Add(
            (a, b));
    }

    private void UpdateWireframeVertices()
    {
        if (wireframeMesh == null ||
            nodeWorldPositions == null)
        {
            return;
        }

        wireframeMesh.vertices =
            nodeWorldPositions;

        if (wireframeLineIndices != null)
        {
            wireframeMesh.SetIndices(
                wireframeLineIndices,
                MeshTopology.Lines,
                0);
        }

        wireframeMesh.RecalculateBounds();
    }

    private void CreateBoundaryMarkerVisual()
    {
        boundaryMarkerObject =
            new GameObject(
                "PhysiK_PointConnected_Boundary_Nodes");

        boundaryMarkerObject.transform.SetPositionAndRotation(
            Vector3.zero,
            Quaternion.identity);

        boundaryMarkerMeshFilter =
            boundaryMarkerObject.AddComponent<MeshFilter>();

        boundaryMarkerMeshRenderer =
            boundaryMarkerObject.AddComponent<MeshRenderer>();

        boundaryMarkerMesh =
            new Mesh
            {
                name =
                    "PhysiK_PointConnected_Boundary_Nodes_Mesh",

                indexFormat =
                    UnityEngine.Rendering.IndexFormat.UInt32
            };

        boundaryMarkerMesh.MarkDynamic();

        boundaryMarkerMeshFilter.sharedMesh =
            boundaryMarkerMesh;

        if (boundaryMarkerMaterial != null)
        {
            boundaryMarkerMeshRenderer.sharedMaterial =
                boundaryMarkerMaterial;
        }
    }

    private void RebuildBoundaryMarkerTopology()
    {
        if (boundaryMarkerMesh == null ||
            boundaryLocalNodeIndices == null)
        {
            return;
        }

        int markerCount =
            boundaryLocalNodeIndices.Length;

        Vector3[] vertices =
            new Vector3[
                markerCount *
                6];

        List<int> triangles =
            new List<int>(
                markerCount *
                8 *
                3);

        for (int i = 0;
             i < markerCount;
             ++i)
        {
            int baseVertex =
                i *
                6;

            int px =
                baseVertex +
                0;

            int nx =
                baseVertex +
                1;

            int py =
                baseVertex +
                2;

            int ny =
                baseVertex +
                3;

            int pz =
                baseVertex +
                4;

            int nz =
                baseVertex +
                5;

            triangles.Add(py);
            triangles.Add(px);
            triangles.Add(pz);

            triangles.Add(py);
            triangles.Add(pz);
            triangles.Add(nx);

            triangles.Add(py);
            triangles.Add(nx);
            triangles.Add(nz);

            triangles.Add(py);
            triangles.Add(nz);
            triangles.Add(px);

            triangles.Add(ny);
            triangles.Add(pz);
            triangles.Add(px);

            triangles.Add(ny);
            triangles.Add(nx);
            triangles.Add(pz);

            triangles.Add(ny);
            triangles.Add(nz);
            triangles.Add(nx);

            triangles.Add(ny);
            triangles.Add(px);
            triangles.Add(nz);
        }

        boundaryMarkerTriangles =
            triangles.ToArray();

        boundaryMarkerMesh.Clear();

        boundaryMarkerMesh.vertices =
            vertices;

        boundaryMarkerMesh.triangles =
            boundaryMarkerTriangles;

        boundaryMarkerMesh.RecalculateNormals();
        boundaryMarkerMesh.RecalculateBounds();
    }

    private void UpdateBoundaryMarkerVertices()
    {
        if (boundaryMarkerMesh == null ||
            boundaryLocalNodeIndices == null ||
            nodes == null)
        {
            return;
        }

        int markerCount =
            boundaryLocalNodeIndices.Length;

        Vector3[] vertices =
            boundaryMarkerMesh.vertices;

        if (vertices == null ||
            vertices.Length != markerCount * 6)
        {
            RebuildBoundaryMarkerTopology();

            vertices =
                boundaryMarkerMesh.vertices;
        }

        float r =
            Mathf.Max(
                0.001f,
                boundaryMarkerRadius);

        for (int i = 0;
             i < markerCount;
             ++i)
        {
            int localNode =
                boundaryLocalNodeIndices[i];

            Vector3 position =
                nodeWorldPositions[localNode];

            int baseVertex =
                i *
                6;

            vertices[baseVertex + 0] =
                position +
                new Vector3(
                    r,
                    0.0f,
                    0.0f);

            vertices[baseVertex + 1] =
                position +
                new Vector3(
                    -r,
                    0.0f,
                    0.0f);

            vertices[baseVertex + 2] =
                position +
                new Vector3(
                    0.0f,
                    r,
                    0.0f);

            vertices[baseVertex + 3] =
                position +
                new Vector3(
                    0.0f,
                    -r,
                    0.0f);

            vertices[baseVertex + 4] =
                position +
                new Vector3(
                    0.0f,
                    0.0f,
                    r);

            vertices[baseVertex + 5] =
                position +
                new Vector3(
                    0.0f,
                    0.0f,
                    -r);
        }

        boundaryMarkerMesh.vertices =
            vertices;

        if (boundaryMarkerTriangles != null)
        {
            boundaryMarkerMesh.triangles =
                boundaryMarkerTriangles;
        }

        boundaryMarkerMesh.RecalculateNormals();
        boundaryMarkerMesh.RecalculateBounds();
    }

    private void CreatePointConnectionLineVisual()
    {
        pointConnectionLineObject =
            new GameObject(
                "PhysiK_PointConnection_Lines");

        pointConnectionLineObject.transform.SetPositionAndRotation(
            Vector3.zero,
            Quaternion.identity);

        pointConnectionLineMeshFilter =
            pointConnectionLineObject.AddComponent<MeshFilter>();

        pointConnectionLineMeshRenderer =
            pointConnectionLineObject.AddComponent<MeshRenderer>();

        pointConnectionLineMesh =
            new Mesh
            {
                name =
                    "PhysiK_PointConnection_Lines_Mesh",

                indexFormat =
                    UnityEngine.Rendering.IndexFormat.UInt32
            };

        pointConnectionLineMesh.MarkDynamic();

        pointConnectionLineMeshFilter.sharedMesh =
            pointConnectionLineMesh;

        if (pointConnectionLineMaterial != null)
        {
            pointConnectionLineMeshRenderer.sharedMaterial =
                pointConnectionLineMaterial;
        }
    }

    private void RebuildPointConnectionLineTopology()
    {
        if (pointConnectionLineMesh == null ||
            boundaryLocalNodeIndices == null ||
            boundaryAnchorPositions == null)
        {
            return;
        }

        int connectionCount =
            boundaryLocalNodeIndices.Length;

        pointConnectionLineVertices =
            new Vector3[
                connectionCount *
                2];

        pointConnectionLineIndices =
            new int[
                connectionCount *
                2];

        for (int i = 0;
             i < connectionCount;
             ++i)
        {
            int baseIndex =
                i *
                2;

            pointConnectionLineIndices[baseIndex + 0] =
                baseIndex +
                0;

            pointConnectionLineIndices[baseIndex + 1] =
                baseIndex +
                1;
        }

        pointConnectionLineMesh.Clear();

        pointConnectionLineMesh.vertices =
            pointConnectionLineVertices;

        pointConnectionLineMesh.SetIndices(
            pointConnectionLineIndices,
            MeshTopology.Lines,
            0);

        pointConnectionLineMesh.RecalculateBounds();
    }

    private void UpdatePointConnectionLineVertices()
    {
        if (pointConnectionLineMesh == null ||
            boundaryLocalNodeIndices == null ||
            boundaryAnchorPositions == null)
        {
            return;
        }

        int connectionCount =
            boundaryLocalNodeIndices.Length;

        if (pointConnectionLineVertices == null ||
            pointConnectionLineVertices.Length !=
                connectionCount *
                2)
        {
            RebuildPointConnectionLineTopology();
        }

        for (int i = 0;
             i < connectionCount;
             ++i)
        {
            int localNodeIndex =
                boundaryLocalNodeIndices[i];

            Vector3 nodePosition =
                nodeWorldPositions[localNodeIndex];

            Vector3 anchorPosition =
                boundaryAnchorPositions[i];

            int baseIndex =
                i *
                2;

            pointConnectionLineVertices[baseIndex + 0] =
                nodePosition;

            pointConnectionLineVertices[baseIndex + 1] =
                anchorPosition;
        }

        pointConnectionLineMesh.vertices =
            pointConnectionLineVertices;

        if (pointConnectionLineIndices != null)
        {
            pointConnectionLineMesh.SetIndices(
                pointConnectionLineIndices,
                MeshTopology.Lines,
                0);
        }

        pointConnectionLineMesh.RecalculateBounds();
    }

    private void ApplyGravityToNative()
    {
        if (world == IntPtr.Zero)
        {
            return;
        }

        PhysiKNative.PHYSIK_SetGravity(
            world,
            gravity.x,
            gravity.y,
            gravity.z);
    }

    [ContextMenu("Remove One Random Tet")]
    private void RemoveOneRandomTetContextMenu()
    {
        if (!Application.isPlaying)
        {
            Debug.Log(
                "Enter Play Mode first.",
                this);

            return;
        }

        RemoveOneRandomTet();
    }

    [ContextMenu("Apply Gravity To Native")]
    private void ApplyGravityContextMenu()
    {
        if (!Application.isPlaying)
        {
            Debug.Log(
                "Enter Play Mode first. Native world does not exist yet.",
                this);

            return;
        }

        ApplyGravityToNative();

        Debug.Log(
            $"Gravity applied to native: " +
            $"({gravity.x:F6}, {gravity.y:F6}, {gravity.z:F6})",
            this);
    }

    [ContextMenu("Reset Gravity To Unity Default")]
    private void ResetGravityToUnityDefault()
    {
        gravity =
            new Vector3(
                0.0f,
                -9.81f,
                0.0f);

        Debug.Log(
            $"Gravity reset to Unity default: " +
            $"({gravity.x:F6}, {gravity.y:F6}, {gravity.z:F6})",
            this);
    }

    private void OnDestroy()
    {
        if (world != IntPtr.Zero)
        {
            PhysiKNative.PHYSIK_DestroyWorld(
                world);

            world =
                IntPtr.Zero;
        }

        if (surfaceObject != null)
        {
            Destroy(
                surfaceObject);
        }

        if (wireframeObject != null)
        {
            Destroy(
                wireframeObject);
        }

        if (boundaryMarkerObject != null)
        {
            Destroy(
                boundaryMarkerObject);
        }

        if (pointConnectionLineObject != null)
        {
            Destroy(
                pointConnectionLineObject);
        }
    }
}

