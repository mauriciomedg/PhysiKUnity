using System;
using System.Collections.Generic;
using UnityEngine;
using PhysiK.Unity;
using System.Diagnostics;
using UnityEngine.InputSystem;

public class PhysikCircularTissue : MonoBehaviour
{
    [Header("PhysiK")]
    [SerializeField] private int substeps = 8;
    [SerializeField] private bool useImplicitEuler = true;

    [Header("Simulation Loop")]
    [SerializeField] private float simulationDt = 1.0f / 30.0f;
    [SerializeField] private int maxSimulationStepsPerFrame = 1;

    [Header("Gravity")]
    [SerializeField] private Vector3 gravity = new Vector3(0.0f, -9.81f, 0.0f);
    [SerializeField] private bool applyGravityEveryStep = true;

    [Header("Circular Tissue / Pancake Mesh")]
    [SerializeField] private int gridResolution = 15;
    [SerializeField] private float radius = 2.0f;
    [SerializeField] private float thickness = 0.12f;

    [Header("FEM")]
    [SerializeField] private PhysikMaterialAsset material;
    [SerializeField] private PhysiKFemModel femModel = PhysiKFemModel.Corotational;

    [Header("Boundary Point Connections")]
    [SerializeField] private float boundaryStiffness = 20000.0f;
    [SerializeField] private float boundaryDamping = 0.0f;
    [SerializeField] private float radialBoundaryOffset = 0.15f;

    [Header("Runtime Cutting")]
    [SerializeField] private int randomSeed = 12345;

    [Header("Visual Mesh")]
    [SerializeField] private bool drawSurface = true;
    [SerializeField] private Material surfaceMaterial;
    [SerializeField] private bool doubleSidedSurface = true;

    private IntPtr world = IntPtr.Zero;
    private PhysiKComponentHandle tetMesh;

    private int[] nodes;
    private int[] tetNodeIndices;

    private Vector3[] nodeWorldPositions;
    private int[] boundaryLocalNodeIndices;
    private Vector3[] boundaryAnchorPositions;

    private readonly Dictionary<int, int> globalToLocalNode = new Dictionary<int, int>();

    private System.Random random;
    private float simulationAccumulator;

    private GameObject surfaceObject;
    private Mesh surfaceMesh;
    private MeshFilter surfaceMeshFilter;
    private MeshRenderer surfaceMeshRenderer;

    private int[] surfaceTriangles;
    private bool surfaceTopologyDirty = true;

    private readonly struct GridNodeKey : IEquatable<GridNodeKey>
    {
        public readonly int ix;
        public readonly int iz;
        public readonly int layer;

        public GridNodeKey(int ix, int iz, int layer)
        {
            this.ix = ix;
            this.iz = iz;
            this.layer = layer;
        }

        public bool Equals(GridNodeKey other)
        {
            return ix == other.ix && iz == other.iz && layer == other.layer;
        }

        public override bool Equals(object obj)
        {
            return obj is GridNodeKey other && Equals(other);
        }

        public override int GetHashCode()
        {
            unchecked
            {
                int hash = ix;
                hash = (hash * 397) ^ iz;
                hash = (hash * 397) ^ layer;
                return hash;
            }
        }
    }

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
            return a == other.a && b == other.b && c == other.c;
        }

        public override bool Equals(object obj)
        {
            return obj is FaceKey other && Equals(other);
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
            UnityEngine.Debug.LogError("Failed to create PhysiK world.", this);
            enabled = false;
            return;
        }

        PhysiKNative.PHYSIK_SetSubstepCount(world, Mathf.Max(1, substeps));
        PhysiKNative.PHYSIK_SetSolverMode(world, useImplicitEuler ? 1 : 0);

        ApplyGravityToNative();

        CreateCircularTissueTetMesh();

        if (drawSurface)
        {
            CreateSurfaceVisual();
            RebuildSurfaceTopology();
            UpdateSurfaceVertices();
        }

        int totalTetCount = tetNodeIndices.Length / 4;
        int activeTetCount = PhysiKNative.PHYSIK_GetActiveTetCount(world, tetMesh);

        UnityEngine.Debug.Log(
            $"Circular tissue created. FEM={femModel}, nodes={nodes.Length}, totalTets={totalTetCount}, activeTets={activeTetCount}, boundaryNodes={boundaryLocalNodeIndices.Length}. Press R to remove one random tet.",
            this);
    }

    private void Update()
    {
        if (world == IntPtr.Zero)
        {
            return;
        }

        if (Keyboard.current != null && Keyboard.current.rKey.wasPressedThisFrame)
        {
            RemoveOneRandomTet();
        }

        simulationAccumulator += Time.deltaTime;

        int steps = 0;

        while (simulationAccumulator >= simulationDt && steps < maxSimulationStepsPerFrame)
        {
            StepSimulation(simulationDt);
            simulationAccumulator -= simulationDt;
            ++steps;
        }

        if (steps == maxSimulationStepsPerFrame)
        {
            simulationAccumulator = 0.0f;
        }

        if (drawSurface)
        {
            if (surfaceTopologyDirty)
            {
                RebuildSurfaceTopology();
            }

            UpdateSurfaceVertices();
        }
    }

    private void StepSimulation(float dt)
    {
        if (applyGravityEveryStep)
        {
            ApplyGravityToNative();
        }

        AddBoundaryPointConnections();

        PhysiKNative.PHYSIK_Step(world, dt);
    }

    private void CreateCircularTissueTetMesh()
    {
        if (material == null)
        {
            UnityEngine.Debug.LogError("PhysiK material is not assigned.", this);
            enabled = false;
            return;
        }

        gridResolution = Mathf.Max(2, gridResolution);
        radius = Mathf.Max(0.01f, radius);
        thickness = Mathf.Max(0.001f, thickness);

        Vector3 origin = transform.position;

        float diameter = radius * 2.0f;
        float cellSize = diameter / gridResolution;
        float halfThickness = thickness * 0.5f;

        List<Vector2Int> selectedCells = new List<Vector2Int>();
        HashSet<Vector2Int> selectedCellSet = new HashSet<Vector2Int>();

        for (int iz = 0; iz < gridResolution; ++iz)
        {
            for (int ix = 0; ix < gridResolution; ++ix)
            {
                float cx = -radius + (ix + 0.5f) * cellSize;
                float cz = -radius + (iz + 0.5f) * cellSize;

                if ((cx * cx + cz * cz) <= radius * radius)
                {
                    Vector2Int cell = new Vector2Int(ix, iz);
                    selectedCells.Add(cell);
                    selectedCellSet.Add(cell);
                }
            }
        }

        Dictionary<GridNodeKey, int> gridToLocalNode = new Dictionary<GridNodeKey, int>();
        List<int> globalNodes = new List<int>();
        List<Vector3> initialWorldPositions = new List<Vector3>();
        List<int> tets = new List<int>();

        int GetOrCreateNode(int ix, int iz, int layer)
        {
            GridNodeKey key = new GridNodeKey(ix, iz, layer);

            if (gridToLocalNode.TryGetValue(key, out int existingLocalIndex))
            {
                return existingLocalIndex;
            }

            float x = -radius + ix * cellSize;
            float z = -radius + iz * cellSize;
            float y = layer == 0 ? -halfThickness : halfThickness;

            Vector3 worldPosition = origin + new Vector3(x, y, z);

            int globalNode = PhysiKNative.PHYSIK_AddNode(
                world,
                worldPosition.x,
                worldPosition.y,
                worldPosition.z);

            int localIndex = globalNodes.Count;

            gridToLocalNode.Add(key, localIndex);
            globalNodes.Add(globalNode);
            initialWorldPositions.Add(worldPosition);

            return localIndex;
        }

        foreach (Vector2Int cell in selectedCells)
        {
            int ix = cell.x;
            int iz = cell.y;

            int n0 = GetOrCreateNode(ix, iz, 0);
            int n1 = GetOrCreateNode(ix + 1, iz, 0);
            int n2 = GetOrCreateNode(ix, iz + 1, 0);
            int n3 = GetOrCreateNode(ix + 1, iz + 1, 0);

            int n4 = GetOrCreateNode(ix, iz, 1);
            int n5 = GetOrCreateNode(ix + 1, iz, 1);
            int n6 = GetOrCreateNode(ix, iz + 1, 1);
            int n7 = GetOrCreateNode(ix + 1, iz + 1, 1);

            int g0 = globalNodes[n0];
            int g1 = globalNodes[n1];
            int g2 = globalNodes[n2];
            int g3 = globalNodes[n3];
            int g4 = globalNodes[n4];
            int g5 = globalNodes[n5];
            int g6 = globalNodes[n6];
            int g7 = globalNodes[n7];

            AddTet(tets, g0, g3, g1, g7);
            AddTet(tets, g0, g2, g3, g7);
            AddTet(tets, g0, g6, g2, g7);
            AddTet(tets, g0, g4, g6, g7);
            AddTet(tets, g0, g5, g4, g7);
            AddTet(tets, g0, g1, g5, g7);
        }

        nodes = globalNodes.ToArray();
        nodeWorldPositions = initialWorldPositions.ToArray();
        tetNodeIndices = tets.ToArray();

        globalToLocalNode.Clear();

        for (int i = 0; i < nodes.Length; ++i)
        {
            globalToLocalNode[nodes[i]] = i;
        }

        BuildBoundaryNodes(
            selectedCells,
            selectedCellSet,
            gridToLocalNode,
            nodeWorldPositions,
            origin);

        PhysikMaterialDesc nativeMaterial = material.ToNative();

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
            UnityEngine.Debug.LogError("Circular tissue TetMesh component creation failed.", this);
            enabled = false;
            return;
        }

        int totalTetCount = tetNodeIndices.Length / 4;
        int activeTetCount = PhysiKNative.PHYSIK_GetActiveTetCount(world, tetMesh);

        UnityEngine.Debug.Log(
            $"Tissue created: totalTets={totalTetCount}, activeTets={activeTetCount}, nodes={nodes.Length}, gridResolution={gridResolution}",
            this);
    }

    private void BuildBoundaryNodes(
        List<Vector2Int> selectedCells,
        HashSet<Vector2Int> selectedCellSet,
        Dictionary<GridNodeKey, int> gridToLocalNode,
        Vector3[] initialPositions,
        Vector3 center)
    {
        HashSet<int> boundaryLocalNodes = new HashSet<int>();

        Vector2Int[] neighbors =
        {
            new Vector2Int(-1,  0),
            new Vector2Int( 1,  0),
            new Vector2Int( 0, -1),
            new Vector2Int( 0,  1)
        };

        foreach (Vector2Int cell in selectedCells)
        {
            bool isBoundaryCell = false;

            for (int i = 0; i < neighbors.Length; ++i)
            {
                Vector2Int neighbor = cell + neighbors[i];

                if (!selectedCellSet.Contains(neighbor))
                {
                    isBoundaryCell = true;
                    break;
                }
            }

            if (!isBoundaryCell)
            {
                continue;
            }

            AddBoundaryNodeIfExists(boundaryLocalNodes, gridToLocalNode, cell.x, cell.y, 0);
            AddBoundaryNodeIfExists(boundaryLocalNodes, gridToLocalNode, cell.x + 1, cell.y, 0);
            AddBoundaryNodeIfExists(boundaryLocalNodes, gridToLocalNode, cell.x, cell.y + 1, 0);
            AddBoundaryNodeIfExists(boundaryLocalNodes, gridToLocalNode, cell.x + 1, cell.y + 1, 0);

            AddBoundaryNodeIfExists(boundaryLocalNodes, gridToLocalNode, cell.x, cell.y, 1);
            AddBoundaryNodeIfExists(boundaryLocalNodes, gridToLocalNode, cell.x + 1, cell.y, 1);
            AddBoundaryNodeIfExists(boundaryLocalNodes, gridToLocalNode, cell.x, cell.y + 1, 1);
            AddBoundaryNodeIfExists(boundaryLocalNodes, gridToLocalNode, cell.x + 1, cell.y + 1, 1);
        }

        boundaryLocalNodeIndices = new int[boundaryLocalNodes.Count];
        boundaryLocalNodes.CopyTo(boundaryLocalNodeIndices);
        Array.Sort(boundaryLocalNodeIndices);

        boundaryAnchorPositions = new Vector3[boundaryLocalNodeIndices.Length];

        for (int i = 0; i < boundaryLocalNodeIndices.Length; ++i)
        {
            Vector3 originalPosition = initialPositions[boundaryLocalNodeIndices[i]];

            Vector3 radial = new Vector3(
                originalPosition.x - center.x,
                0.0f,
                originalPosition.z - center.z);

            if (radial.sqrMagnitude > 1.0e-8f)
            {
                radial.Normalize();
            }
            else
            {
                radial = Vector3.zero;
            }

            boundaryAnchorPositions[i] =
                originalPosition + radial * radialBoundaryOffset;
        }
    }

    private static void AddBoundaryNodeIfExists(
        HashSet<int> boundaryNodes,
        Dictionary<GridNodeKey, int> gridToLocalNode,
        int ix,
        int iz,
        int layer)
    {
        GridNodeKey key = new GridNodeKey(ix, iz, layer);

        if (gridToLocalNode.TryGetValue(key, out int localNodeIndex))
        {
            boundaryNodes.Add(localNodeIndex);
        }
    }

    private static void AddTet(List<int> tets, int n0, int n1, int n2, int n3)
    {
        tets.Add(n0);
        tets.Add(n1);
        tets.Add(n2);
        tets.Add(n3);
    }

    private void AddBoundaryPointConnections()
    {
        if (boundaryLocalNodeIndices == null || boundaryAnchorPositions == null)
        {
            return;
        }

        for (int i = 0; i < boundaryLocalNodeIndices.Length; ++i)
        {
            int localNodeIndex = boundaryLocalNodeIndices[i];
            int globalNodeIndex = nodes[localNodeIndex];
            Vector3 target = boundaryAnchorPositions[i];

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
        if (tetNodeIndices == null || boundaryLocalNodeIndices == null)
        {
            return true;
        }

        int baseIndex = tetIndex * 4;

        if (baseIndex < 0 || baseIndex + 3 >= tetNodeIndices.Length)
        {
            return true;
        }

        int g0 = tetNodeIndices[baseIndex + 0];
        int g1 = tetNodeIndices[baseIndex + 1];
        int g2 = tetNodeIndices[baseIndex + 2];
        int g3 = tetNodeIndices[baseIndex + 3];

        int l0 = globalToLocalNode[g0];
        int l1 = globalToLocalNode[g1];
        int l2 = globalToLocalNode[g2];
        int l3 = globalToLocalNode[g3];

        for (int i = 0; i < boundaryLocalNodeIndices.Length; ++i)
        {
            int boundaryNode = boundaryLocalNodeIndices[i];

            if (l0 == boundaryNode ||
                l1 == boundaryNode ||
                l2 == boundaryNode ||
                l3 == boundaryNode)
            {
                return true;
            }
        }

        return false;
    }
    private void RemoveOneRandomTet()
    {
        if (world == IntPtr.Zero || tetNodeIndices == null)
        {
            return;
        }

        int totalTetCount = tetNodeIndices.Length / 4;

        if (totalTetCount <= 0)
        {
            return;
        }

        int activeBefore = PhysiKNative.PHYSIK_GetActiveTetCount(world, tetMesh);

        if (activeBefore <= 0)
        {
            UnityEngine.Debug.Log("No active tets left to remove.", this);
            return;
        }

        int selectedTet = -1;

        for (int attempt = 0; attempt < 64; ++attempt)
        {
            int candidate = random.Next(0, totalTetCount);

            if (PhysiKNative.PHYSIK_IsTetActive(world, tetMesh, candidate) != 0 &&
                !TetTouchesBoundary(candidate))
            {
                selectedTet = candidate;
                break;
            }
        }

        if (selectedTet < 0)
        {
            for (int tet = 0; tet < totalTetCount; ++tet)
            {
                if (PhysiKNative.PHYSIK_IsTetActive(world, tetMesh, tet) != 0 &&
                    !TetTouchesBoundary(tet))
                {
                    selectedTet = tet;
                    break;
                }
            }
        }

        if (selectedTet < 0)
        {
            UnityEngine.Debug.Log("Could not find an active tet to remove.", this);
            return;
        }

        PhysiKNative.PHYSIK_DeactivateTet(world, tetMesh, selectedTet);

        int activeAfter = PhysiKNative.PHYSIK_GetActiveTetCount(world, tetMesh);
        int removedCount = activeBefore - activeAfter;

        surfaceTopologyDirty = true;

        UnityEngine.Debug.Log(
            $"Removed random tet {selectedTet}. " +
            $"Removed={removedCount}, " +
            $"activeTets={activeAfter}, " +
            $"removedTets={totalTetCount - activeAfter}, " +
            $"totalTets={totalTetCount}",
            this);
    }

    private void CreateSurfaceVisual()
    {
        surfaceObject = new GameObject("PhysiK_Tissue_Surface");
        surfaceObject.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);

        surfaceMeshFilter = surfaceObject.AddComponent<MeshFilter>();
        surfaceMeshRenderer = surfaceObject.AddComponent<MeshRenderer>();

        surfaceMesh = new Mesh
        {
            name = "PhysiK_Tissue_Surface_Mesh",
            indexFormat = UnityEngine.Rendering.IndexFormat.UInt32
        };

        surfaceMesh.MarkDynamic();

        surfaceMeshFilter.sharedMesh = surfaceMesh;

        if (surfaceMaterial != null)
        {
            surfaceMeshRenderer.sharedMaterial = surfaceMaterial;
        }
    }

    private void RebuildSurfaceTopology()
    {
        if (surfaceMesh == null || tetNodeIndices == null || nodes == null)
        {
            return;
        }

        Dictionary<FaceKey, int> faceCounts = new Dictionary<FaceKey, int>();
        Dictionary<FaceKey, Face> faceRepresentatives = new Dictionary<FaceKey, Face>();

        int tetCount = tetNodeIndices.Length / 4;

        for (int tet = 0; tet < tetCount; ++tet)
        {
            if (PhysiKNative.PHYSIK_IsTetActive(world, tetMesh, tet) == 0)
            {
                continue;
            }

            int baseIndex = tet * 4;

            int a = globalToLocalNode[tetNodeIndices[baseIndex + 0]];
            int b = globalToLocalNode[tetNodeIndices[baseIndex + 1]];
            int c = globalToLocalNode[tetNodeIndices[baseIndex + 2]];
            int d = globalToLocalNode[tetNodeIndices[baseIndex + 3]];

            AddFace(faceCounts, faceRepresentatives, a, b, c);
            AddFace(faceCounts, faceRepresentatives, a, b, d);
            AddFace(faceCounts, faceRepresentatives, a, c, d);
            AddFace(faceCounts, faceRepresentatives, b, c, d);
        }

        List<int> triangles = new List<int>();

        foreach (KeyValuePair<FaceKey, int> pair in faceCounts)
        {
            if (pair.Value != 1)
            {
                continue;
            }

            Face face = faceRepresentatives[pair.Key];

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

        surfaceTriangles = triangles.ToArray();

        surfaceMesh.Clear();
        surfaceMesh.vertices = nodeWorldPositions;
        surfaceMesh.triangles = surfaceTriangles;
        surfaceMesh.RecalculateNormals();
        surfaceMesh.RecalculateBounds();

        surfaceTopologyDirty = false;
    }

    private static void AddFace(
        Dictionary<FaceKey, int> faceCounts,
        Dictionary<FaceKey, Face> faceRepresentatives,
        int a,
        int b,
        int c)
    {
        FaceKey key = new FaceKey(a, b, c);

        if (faceCounts.TryGetValue(key, out int count))
        {
            faceCounts[key] = count + 1;
        }
        else
        {
            faceCounts.Add(key, 1);
            faceRepresentatives.Add(key, new Face(a, b, c));
        }
    }

    private void UpdateSurfaceVertices()
    {
        if (surfaceMesh == null || nodes == null || nodeWorldPositions == null)
        {
            return;
        }

        for (int i = 0; i < nodes.Length; ++i)
        {
            PhysiKNative.PHYSIK_GetNodePosition(
                world,
                nodes[i],
                out float x,
                out float y,
                out float z);

            nodeWorldPositions[i] = new Vector3(x, y, z);
        }

        surfaceMesh.vertices = nodeWorldPositions;

        if (surfaceTriangles != null)
        {
            surfaceMesh.triangles = surfaceTriangles;
        }

        surfaceMesh.RecalculateNormals();
        surfaceMesh.RecalculateBounds();
    }

    private void ApplyGravityToNative()
    {
        if (world == IntPtr.Zero)
        {
            return;
        }

        PhysiKNative.PHYSIK_SetGravity(world, gravity.x, gravity.y, gravity.z);
    }

    [ContextMenu("Remove One Random Tet")]
    private void RemoveOneRandomTetContextMenu()
    {
        if (!Application.isPlaying)
        {
            UnityEngine.Debug.Log("Enter Play Mode first.", this);
            return;
        }

        RemoveOneRandomTet();
    }

    [ContextMenu("Apply Gravity To Native")]
    private void ApplyGravityContextMenu()
    {
        if (!Application.isPlaying)
        {
            UnityEngine.Debug.Log("Enter Play Mode first. Native world does not exist yet.", this);
            return;
        }

        ApplyGravityToNative();

        UnityEngine.Debug.Log(
            $"Gravity applied to native: ({gravity.x:F6}, {gravity.y:F6}, {gravity.z:F6})",
            this);
    }

    [ContextMenu("Reset Gravity To Unity Default")]
    private void ResetGravityToUnityDefault()
    {
        gravity = new Vector3(0.0f, -9.81f, 0.0f);

        UnityEngine.Debug.Log(
            $"Gravity reset to Unity default: ({gravity.x:F6}, {gravity.y:F6}, {gravity.z:F6})",
            this);
    }

    private void OnDestroy()
    {
        if (world != IntPtr.Zero)
        {
            PhysiKNative.PHYSIK_DestroyWorld(world);
            world = IntPtr.Zero;
        }

        if (surfaceObject != null)
        {
            Destroy(surfaceObject);
        }
    }

}
