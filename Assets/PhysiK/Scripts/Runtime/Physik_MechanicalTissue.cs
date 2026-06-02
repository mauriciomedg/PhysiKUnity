using System;
using System.Collections.Generic;
using UnityEngine;
using PhysiK.Unity;

public class Physik_MechanicalTissue : MonoBehaviour, IPhysikWorldParticipant
{
    [Header("PhysiK World")]
    [SerializeField] private Physik_World physikWorld;

    [Header("Radial Circular Tissue Mesh")]
    [SerializeField] private int radialSegments = 6;
    [SerializeField] private int angularSegments = 30;
    [SerializeField] private float radius = 2.0f;
    [SerializeField] private float thickness = 0.12f;

    [Header("FEM")]
    [SerializeField] private PhysikMaterialAsset material;

    [Header("Native Surface Visual")]
    [SerializeField] private bool drawSurface = true;
    [SerializeField] private Material surfaceMaterial;
    [SerializeField] private bool recalculateSurfaceBounds = true;

    [Header("Tet Wireframe Debug Draw")]
    [SerializeField] private bool drawWireframe = true;
    [SerializeField] private Material wireframeMaterial;

    private bool initialized;

    private IntPtr world = IntPtr.Zero;

    private PhysiKComponentHandle tetMesh;
    private PhysiKComponentHandle surfaceExtractionHandle;
    private PhysiKComponentHandle surfaceVisualHandle;

    // Generated local node index -> World global node index.
    private int[] nodes;

    // Clean local tet indices returned by the generated tet mesh.
    private int[] tetLocalNodeIndices;

    // Current World positions indexed by generated local node index.
    private Vector3[] nodeWorldPositions;

    // Used only by the temporary tet wireframe debug draw.
    private bool topologyDirty = true;

    // Native surface visual.
    private GameObject surfaceObject;
    private Mesh surfaceMesh;
    private MeshFilter surfaceMeshFilter;
    private MeshRenderer surfaceMeshRenderer;

    private Vector3[] surfaceVertices;
    private Vector3[] surfaceNormals;
    private int[] surfaceTriangleIndices;

    // Tet wireframe debug draw.
    private GameObject wireframeObject;
    private Mesh wireframeMesh;
    private MeshFilter wireframeMeshFilter;
    private MeshRenderer wireframeMeshRenderer;
    private int[] wireframeLineIndices;

    public bool IsInitialized => initialized;

    public IntPtr WorldHandle => world;

    public PhysiKComponentHandle TetMeshHandle => tetMesh;

    public float TissuePlaneY => transform.position.y;

    public Physik_World WorldOwner => physikWorld;

    public int[] GlobalNodeIndices => nodes;

    public Vector3[] NodeWorldPositions => nodeWorldPositions;

    public float TissueRadius => radius;

    public Vector3 TissueCenter => transform.position;

    private void Start()
    {
        if (physikWorld == null)
        {
            Debug.LogError(
                "Physik_World is not assigned.",
                this);

            enabled =
                false;

            return;
        }

        world =
            physikWorld.WorldHandle;

        if (world == IntPtr.Zero)
        {
            Debug.LogError(
                "Physik_World has not created its native world.",
                this);

            enabled =
                false;

            return;
        }

        if (!CreateRadialCircularTissueTetMesh())
        {
            enabled =
                false;

            return;
        }

        if (drawSurface)
        {
            if (!CreateNativeSurfaceVisualComponents())
            {
                enabled =
                    false;

                return;
            }

            CreateSurfaceVisual();
            UpdateSurfaceVisualFromNative();
        }

        if (drawWireframe)
        {
            CreateWireframeVisual();
            RebuildWireframeTopology();
            UpdateWireframeVertices();
        }

        int totalTetCount =
            tetLocalNodeIndices.Length /
            4;

        int activeTetCount =
            PhysiKNative.PHYSIK_GetActiveTetCount(
                world,
                tetMesh);

        initialized =
            true;

        physikWorld.RegisterParticipant(
            this);

        Debug.Log(
            $"Radial tissue created. " +
            $"nodes={nodes.Length}, " +
            $"totalTets={totalTetCount}, " +
            $"activeTets={activeTetCount}, " +
            $"radialSegments={radialSegments}, " +
            $"angularSegments={angularSegments}.",
            this);
    }

    private void Update()
    {
        if (!initialized ||
            world == IntPtr.Zero)
        {
            return;
        }
    }

    public void OnPhysikBeforeSimulationStep(float dt)
    {
        if (!initialized ||
            world == IntPtr.Zero)
        {
            return;
        }

        // Boundary point connections are pushed by Physik_BoundaryConnections.
    }

    public void OnPhysikAfterSimulationFrame()
    {
        if (!initialized ||
            world == IntPtr.Zero)
        {
            return;
        }

        UpdateNodeWorldPositions();

        if (drawSurface)
        {
            UpdateSurfaceVisualFromNative();
        }

        if (drawWireframe)
        {
            if (topologyDirty)
            {
                RebuildWireframeTopology();
            }

            UpdateWireframeVertices();
        }

        topologyDirty =
            false;
    }

    public void OnPhysikWorldDestroyed()
    {
        initialized =
            false;

        world =
            IntPtr.Zero;
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

        Vector3 origin =
            transform.position;

        float halfThickness =
            thickness *
            0.5f;

        List<Vector3> rawPositions =
            new List<Vector3>();

        List<int> rawTetLocalNodeIndices =
            new List<int>();

        List<(int a, int b, int c)> triangles2D =
            new List<(int a, int b, int c)>();

        int[,] bottomRingNodes =
            new int[radialSegments + 1, angularSegments];

        int CreateLocalNode(Vector3 worldPosition)
        {
            int localIndex =
                rawPositions.Count;

            rawPositions.Add(
                worldPosition);

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
            float ringRadius =
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
                    ringRadius;

                float z =
                    Mathf.Sin(angle) *
                    ringRadius;

                // Bottom and top nodes are intentionally created in pairs.
                bottomRingNodes[ring, segment] =
                    CreateLocalNode(
                        origin +
                        new Vector3(
                            x,
                            -halfThickness,
                            z));

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

            triangles2D.Add(
                (a, b, c));
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

            int globalNodeCount =
                PhysiKNative.PHYSIK_GetTetMeshGlobalNodeCount(
                    world,
                    tetMesh);

            if (globalNodeCount <= 0)
            {
                Debug.LogError(
                    "Tet mesh physics component has no global nodes.",
                    this);

                return false;
            }

            if (globalNodeCount != generatedPositions.Length)
            {
                Debug.LogError(
                    $"Generated mesh node count does not match physics component node count. " +
                    $"generatedNodes={generatedPositions.Length}, " +
                    $"physicsNodes={globalNodeCount}.",
                    this);

                return false;
            }

            nodes =
                new int[globalNodeCount];

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

            nodeWorldPositions =
                generatedPositions;

            tetLocalNodeIndices =
                generatedTetLocalNodeIndices;

            int totalTetCount =
                tetLocalNodeIndices.Length /
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
        SortPrismColumnsByBottomNode(
            ref a,
            ref at,
            ref b,
            ref bt,
            ref c,
            ref ct);

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

    private static void SortPrismColumnsByBottomNode(
        ref int a,
        ref int at,
        ref int b,
        ref int bt,
        ref int c,
        ref int ct)
    {
        if (a > b)
        {
            (a, b) = (b, a);
            (at, bt) = (bt, at);
        }

        if (b > c)
        {
            (b, c) = (c, b);
            (bt, ct) = (ct, bt);
        }

        if (a > b)
        {
            (a, b) = (b, a);
            (at, bt) = (bt, at);
        }
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

        if (Mathf.Abs(signedVolume6) < 1.0e-10f)
        {
            return;
        }

        if (signedVolume6 < 0.0f)
        {
            (n1, n2) =
                (n2, n1);
        }

        tets.Add(n0);
        tets.Add(n1);
        tets.Add(n2);
        tets.Add(n3);
    }

    private bool CreateNativeSurfaceVisualComponents()
    {
        surfaceExtractionHandle =
            PhysiKNative.PHYSIK_CreateSurfaceExtractionComponent(
                world,
                tetMesh);

        if (PhysiKNative.PHYSIK_IsComponentHandleValid(
                world,
                surfaceExtractionHandle) == 0)
        {
            Debug.LogError(
                "Failed to create native SurfaceExtractionComponent.",
                this);

            return false;
        }

        surfaceVisualHandle =
            PhysiKNative.PHYSIK_CreateSurfaceVisualComponent(
                world,
                surfaceExtractionHandle);

        if (PhysiKNative.PHYSIK_IsComponentHandleValid(
                world,
                surfaceVisualHandle) == 0)
        {
            Debug.LogError(
                "Failed to create native SurfaceVisualComponent.",
                this);

            return false;
        }

        return true;
    }

    private void CreateSurfaceVisual()
    {
        surfaceObject =
            new GameObject(
                "PhysiK_MechanicalTissue_NativeSurfaceVisual");

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
                    "PhysiK_MechanicalTissue_NativeSurfaceVisual_Mesh",

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

    private void UpdateSurfaceVisualFromNative()
    {
        if (surfaceMesh == null ||
            surfaceVisualHandle.IsValid == false)
        {
            return;
        }

        int vertexCount =
            PhysiKNative.PHYSIK_GetSurfaceVisualVertexCount(
                world,
                surfaceVisualHandle);

        int triangleIndexCount =
            PhysiKNative.PHYSIK_GetSurfaceVisualTriangleIndexCount(
                world,
                surfaceVisualHandle);

        int normalCount =
            PhysiKNative.PHYSIK_GetSurfaceVisualNormalCount(
                world,
                surfaceVisualHandle);

        if (vertexCount <= 0 ||
            triangleIndexCount <= 0 ||
            triangleIndexCount % 3 != 0)
        {
            surfaceMesh.Clear();
            return;
        }

        bool topologyChanged =
            surfaceVertices == null ||
            surfaceVertices.Length != vertexCount ||
            surfaceTriangleIndices == null ||
            surfaceTriangleIndices.Length != triangleIndexCount;

        if (surfaceVertices == null ||
            surfaceVertices.Length != vertexCount)
        {
            surfaceVertices =
                new Vector3[vertexCount];
        }

        if (surfaceNormals == null ||
            surfaceNormals.Length != vertexCount)
        {
            surfaceNormals =
                new Vector3[vertexCount];
        }

        if (surfaceTriangleIndices == null ||
            surfaceTriangleIndices.Length != triangleIndexCount)
        {
            surfaceTriangleIndices =
                new int[triangleIndexCount];
        }

        for (int vertexIndex = 0;
             vertexIndex < vertexCount;
             ++vertexIndex)
        {
            int ok =
                PhysiKNative.PHYSIK_GetSurfaceVisualVertex(
                    world,
                    surfaceVisualHandle,
                    vertexIndex,
                    out float x,
                    out float y,
                    out float z);

            if (ok == 0)
            {
                return;
            }

            surfaceVertices[vertexIndex] =
                new Vector3(
                    x,
                    y,
                    z);
        }

        for (int index = 0;
             index < triangleIndexCount;
             ++index)
        {
            int ok =
                PhysiKNative.PHYSIK_GetSurfaceVisualTriangleIndex(
                    world,
                    surfaceVisualHandle,
                    index,
                    out int triangleIndex);

            if (ok == 0)
            {
                return;
            }

            surfaceTriangleIndices[index] =
                triangleIndex;
        }

        bool nativeNormalsAvailable =
            normalCount ==
            vertexCount;

        if (nativeNormalsAvailable)
        {
            for (int normalIndex = 0;
                 normalIndex < normalCount;
                 ++normalIndex)
            {
                int ok =
                    PhysiKNative.PHYSIK_GetSurfaceVisualNormal(
                        world,
                        surfaceVisualHandle,
                        normalIndex,
                        out float x,
                        out float y,
                        out float z);

                if (ok == 0)
                {
                    nativeNormalsAvailable =
                        false;

                    break;
                }

                surfaceNormals[normalIndex] =
                    new Vector3(
                        x,
                        y,
                        z);
            }
        }

        if (topologyChanged)
        {
            surfaceMesh.Clear();
        }

        surfaceMesh.vertices =
            surfaceVertices;

        surfaceMesh.triangles =
            surfaceTriangleIndices;

        if (nativeNormalsAvailable)
        {
            surfaceMesh.normals =
                surfaceNormals;
        }
        else
        {
            surfaceMesh.RecalculateNormals();
        }

        if (recalculateSurfaceBounds)
        {
            surfaceMesh.RecalculateBounds();
        }
    }

    public bool DeactivateTet(int tetIndex)
    {
        if (world == IntPtr.Zero ||
            tetLocalNodeIndices == null)
        {
            return false;
        }

        if (tetIndex < 0 ||
            tetIndex >= tetLocalNodeIndices.Length / 4)
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

    private void UpdateNodeWorldPositions()
    {
        if (nodes == null ||
            nodeWorldPositions == null)
        {
            return;
        }

        for (int localNode = 0;
             localNode < nodes.Length;
             ++localNode)
        {
            PhysiKNative.PHYSIK_GetNodePosition(
                world,
                nodes[localNode],
                out float x,
                out float y,
                out float z);

            nodeWorldPositions[localNode] =
                new Vector3(
                    x,
                    y,
                    z);
        }
    }

    private void CreateWireframeVisual()
    {
        wireframeObject =
            new GameObject(
                "PhysiK_Tissue_Wireframe_Debug");

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
                    "PhysiK_Tissue_Wireframe_Debug_Mesh",

                indexFormat =
                    UnityEngine.Rendering.IndexFormat.UInt32
            };

        wireframeMesh.MarkDynamic();

        wireframeMeshFilter.sharedMesh =
            wireframeMesh;

        wireframeMeshRenderer.shadowCastingMode =
            UnityEngine.Rendering.ShadowCastingMode.Off;

        wireframeMeshRenderer.receiveShadows =
            false;

        if (wireframeMaterial != null)
        {
            wireframeMeshRenderer.sharedMaterial =
                wireframeMaterial;
        }
    }

    private void RebuildWireframeTopology()
    {
        if (wireframeMesh == null ||
            tetLocalNodeIndices == null)
        {
            return;
        }

        HashSet<(int, int)> uniqueEdges =
            new HashSet<(int, int)>();

        int tetCount =
            tetLocalNodeIndices.Length /
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
                tetLocalNodeIndices[baseIndex + 0];

            int b =
                tetLocalNodeIndices[baseIndex + 1];

            int c =
                tetLocalNodeIndices[baseIndex + 2];

            int d =
                tetLocalNodeIndices[baseIndex + 3];

            AddWireEdge(uniqueEdges, a, b);
            AddWireEdge(uniqueEdges, a, c);
            AddWireEdge(uniqueEdges, a, d);
            AddWireEdge(uniqueEdges, b, c);
            AddWireEdge(uniqueEdges, b, d);
            AddWireEdge(uniqueEdges, c, d);
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

        if (wireframeLineIndices.Length > 0)
        {
            wireframeMesh.SetIndices(
                wireframeLineIndices,
                MeshTopology.Lines,
                0);
        }

        wireframeMesh.RecalculateBounds();
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

    private void OnDestroy()
    {
        if (physikWorld != null)
        {
            physikWorld.UnregisterParticipant(
                this);
        }

        initialized =
            false;

        world =
            IntPtr.Zero;

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

    }
}

