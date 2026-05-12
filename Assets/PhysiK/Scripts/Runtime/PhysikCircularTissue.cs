using System;
using System.Collections.Generic;
using UnityEngine;
using PhysiK.Unity;

public class PhysikCircularTissue : MonoBehaviour
{
    [Header("PhysiK")]
    [SerializeField] private int substeps = 8;
    [SerializeField] private bool useImplicitEuler = true;

    [Header("Gravity")]
    [SerializeField] private Vector3 gravity = new Vector3(0.0f, -9.81f, 0.0f);
    [SerializeField] private bool applyGravityEveryFixedUpdate = true;

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

    [Header("Runtime Cutting")]
    [SerializeField] private KeyCode cutKey = KeyCode.R;
    [SerializeField] private int randomSeed = 12345;

    [Header("Visual")]
    [SerializeField] private bool drawNodes = true;
    [SerializeField] private bool drawTetEdges = true;
    [SerializeField] private float nodeRadius = 0.025f;
    [SerializeField] private float edgeWidth = 0.006f;
    [SerializeField] private Material lineMaterial;

    private IntPtr world = IntPtr.Zero;
    private PhysiKComponentHandle tetMesh;

    private int[] nodes;
    private int[] tetNodeIndices;

    private Vector3[] initialLocalPositions;
    private int[] boundaryLocalNodeIndices;
    private Vector3[] boundaryAnchorPositions;

    private Transform[] nodeVisuals;
    private readonly List<TetEdgeVisual> tetEdgeVisuals = new List<TetEdgeVisual>();

    private System.Random random;

    private struct TetEdgeVisual
    {
        public int tetIndex;
        public int localNodeA;
        public int localNodeB;
        public LineRenderer line;
    }

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

        PhysiKNative.PHYSIK_SetSubstepCount(world, Mathf.Max(1, substeps));
        PhysiKNative.PHYSIK_SetSolverMode(world, useImplicitEuler ? 1 : 0);

        ApplyGravityToNative();

        CreateCircularTissueTetMesh();
        CreateVisuals();
        UpdateVisuals();

        Debug.Log(
            $"Circular tissue created. FEM={femModel}, nodes={nodes.Length}, tets={tetNodeIndices.Length / 4}, boundaryNodes={boundaryLocalNodeIndices.Length}. Press {cutKey} to remove one random tet.",
            this);
    }

    private void Update()
    {
        if (Input.GetKeyDown(cutKey))
        {
            RemoveOneRandomTet();
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

        AddBoundaryPointConnections();

        PhysiKNative.PHYSIK_Step(world, Time.fixedDeltaTime);

        UpdateVisuals();
    }

    private void CreateCircularTissueTetMesh()
    {
        if (material == null)
        {
            Debug.LogError("PhysiK material is not assigned.", this);
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
        List<Vector3> localPositions = new List<Vector3>();
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

            Vector3 localPosition = new Vector3(x, y, z);
            Vector3 worldPosition = origin + localPosition;

            int globalNode = PhysiKNative.PHYSIK_AddNode(
                world,
                worldPosition.x,
                worldPosition.y,
                worldPosition.z);

            int localIndex = globalNodes.Count;

            gridToLocalNode.Add(key, localIndex);
            globalNodes.Add(globalNode);
            localPositions.Add(worldPosition);

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

            // Six tetrahedra filling one thin cube around diagonal 0 -> 7.
            AddTet(tets, g0, g3, g1, g7);
            AddTet(tets, g0, g2, g3, g7);
            AddTet(tets, g0, g6, g2, g7);
            AddTet(tets, g0, g4, g6, g7);
            AddTet(tets, g0, g5, g4, g7);
            AddTet(tets, g0, g1, g5, g7);
        }

        nodes = globalNodes.ToArray();
        initialLocalPositions = localPositions.ToArray();
        tetNodeIndices = tets.ToArray();

        BuildBoundaryNodes(selectedCells, selectedCellSet, gridToLocalNode);

        PhysikMaterialDesc nativeMaterial = material.ToNative();

        Debug.Log(
            $"Creating circular tissue TetMesh. Grid={gridResolution}, cells={selectedCells.Count}, nodes={nodes.Length}, tets={tetNodeIndices.Length / 4}, FEM={femModel}",
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
            Debug.LogError("Circular tissue TetMesh component creation failed.", this);
            enabled = false;
            return;
        }

        int totalTetCount = tetNodeIndices.Length / 4;
        int activeTetCount = PhysiKNative.PHYSIK_GetActiveTetCount(world, tetMesh);

        Debug.Log(
            $"Tissue created: totalTets={totalTetCount}, activeTets={activeTetCount}, nodes={nodes.Length}",
            this);
    }

    private void BuildBoundaryNodes(
        List<Vector2Int> selectedCells,
        HashSet<Vector2Int> selectedCellSet,
        Dictionary<GridNodeKey, int> gridToLocalNode)
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
            boundaryAnchorPositions[i] = initialLocalPositions[boundaryLocalNodeIndices[i]];
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
            Debug.Log("No active tets left to remove.", this);
            return;
        }

        int selectedTet = -1;

        // First try random attempts.
        for (int attempt = 0; attempt < 64; ++attempt)
        {
            int candidate = random.Next(0, totalTetCount);

            if (PhysiKNative.PHYSIK_IsTetActive(world, tetMesh, candidate) != 0)
            {
                selectedTet = candidate;
                break;
            }
        }

        // Fallback: linear scan.
        if (selectedTet < 0)
        {
            for (int tet = 0; tet < totalTetCount; ++tet)
            {
                if (PhysiKNative.PHYSIK_IsTetActive(world, tetMesh, tet) != 0)
                {
                    selectedTet = tet;
                    break;
                }
            }
        }

        if (selectedTet < 0)
        {
            Debug.Log("Could not find an active tet to remove.", this);
            return;
        }

        PhysiKNative.PHYSIK_DeactivateTet(world, tetMesh, selectedTet);

        int activeAfter = PhysiKNative.PHYSIK_GetActiveTetCount(world, tetMesh);

        Debug.Log(
            $"Removed random tet {selectedTet}. Active tets: {activeAfter}/{totalTetCount}",
            this);

        UpdateVisuals();
    }

    private void CreateVisuals()
    {
        if (drawNodes)
        {
            nodeVisuals = new Transform[nodes.Length];

            for (int i = 0; i < nodeVisuals.Length; ++i)
            {
                GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                sphere.name = $"PhysiK_Tissue_Node_{i}";
                sphere.transform.localScale = Vector3.one * nodeRadius;

                Collider collider = sphere.GetComponent<Collider>();
                if (collider != null)
                {
                    Destroy(collider);
                }

                nodeVisuals[i] = sphere.transform;
            }
        }

        if (drawTetEdges)
        {
            CreateTetEdgeVisuals();
        }
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
        GameObject edge = new GameObject($"PhysiK_Tissue_Tet_{tetIndex}_Edge");
        LineRenderer line = edge.AddComponent<LineRenderer>();

        line.positionCount = 2;
        line.widthMultiplier = edgeWidth;
        line.useWorldSpace = true;

        if (lineMaterial != null)
        {
            line.material = lineMaterial;
        }

        tetEdgeVisuals.Add(new TetEdgeVisual
        {
            tetIndex = tetIndex,
            localNodeA = localA,
            localNodeB = localB,
            line = line
        });
    }

    private void UpdateVisuals()
    {
        if (world == IntPtr.Zero || nodes == null)
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

            if (nodeVisuals != null && i < nodeVisuals.Length && nodeVisuals[i] != null)
            {
                nodeVisuals[i].position = positions[i];
            }
        }

        if (!drawTetEdges)
        {
            return;
        }

        for (int i = 0; i < tetEdgeVisuals.Count; ++i)
        {
            TetEdgeVisual edge = tetEdgeVisuals[i];

            if (edge.line == null)
            {
                continue;
            }

            bool active = PhysiKNative.PHYSIK_IsTetActive(world, tetMesh, edge.tetIndex) != 0;

            edge.line.enabled = active;

            if (active)
            {
                edge.line.SetPosition(0, positions[edge.localNodeA]);
                edge.line.SetPosition(1, positions[edge.localNodeB]);
            }
        }
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
            Debug.Log("Enter Play Mode first.", this);
            return;
        }

        RemoveOneRandomTet();
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
