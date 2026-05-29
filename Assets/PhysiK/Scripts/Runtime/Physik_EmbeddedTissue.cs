using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using PhysiK.Unity;

public class Physik_EmbeddedTissue : MonoBehaviour
{
    [Header("Host")]
    [SerializeField] private Physik_MechanicalTissue tissueHost;

    [Header("Mapped Tet Mesh")]
    [SerializeField] private int radialSegments = 6;
    [SerializeField] private int angularSegments = 30;
    [SerializeField] private float radius = 2.0f;
    [SerializeField] private float thickness = 0.12f;
    [SerializeField] private float yOffset = 0.0f;

    [Header("Wireframe")]
    [SerializeField] private bool drawWireframe = true;
    [SerializeField] private Material wireframeMaterial;

    [Header("Debug")]
    [SerializeField] private bool logCreation = true;

    private PhysiKComponentHandle mappedTetMeshHandle;
    private PhysiKComponentHandle mapperHandle;

    private GameObject wireframeObject;
    private Mesh wireframeMesh;
    private MeshFilter wireframeMeshFilter;
    private MeshRenderer wireframeMeshRenderer;
    private int[] wireframeLineIndices;

    private Vec3[] nativeRestPositions;
    private Vector3[] mappedVertices;
    private int[] tetLocalNodeIndices;

    private bool initialized;
    private int previousActiveTetCount = -1;

    public bool IsInitialized => initialized;

    public System.IntPtr WorldHandle => tissueHost != null
        ? tissueHost.WorldHandle
        : System.IntPtr.Zero;

    public PhysiKComponentHandle MappedTetMeshHandle => mappedTetMeshHandle;

    public int VertexCount => nativeRestPositions != null
        ? nativeRestPositions.Length
        : 0;

    private IEnumerator Start()
    {
        if (tissueHost == null)
        {
            Debug.LogError("Missing tissueHost.", this);
            yield break;
        }

        yield return new WaitUntil(() => tissueHost.IsInitialized);

        CreateMappedCircularTetMesh();
    }

    private void LateUpdate()
    {
        if (!initialized)
        {
            return;
        }

        UpdateMappedVerticesFromNative();

        if (drawWireframe)
        {
            EnsureWireframeExists();
            RebuildWireframeTopologyIfNeeded();
            UpdateWireframeVertices();
        }
        else if (wireframeObject != null && wireframeObject.activeSelf)
        {
            wireframeObject.SetActive(false);
        }
    }

    private void CreateMappedCircularTetMesh()
    {
        radialSegments = Mathf.Max(1, radialSegments);
        angularSegments = Mathf.Max(8, angularSegments);
        radius = Mathf.Max(0.01f, radius);
        thickness = Mathf.Max(0.001f, thickness);

        float halfThickness = thickness * 0.5f;
        Vector3 origin = tissueHost.transform.position + new Vector3(0.0f, yOffset, 0.0f);

        List<Vector3> positions = new List<Vector3>();
        List<int> tets = new List<int>();
        List<(int a, int b, int c)> bottomTriangles = new List<(int a, int b, int c)>();

        int[,] bottomRingNodes = new int[radialSegments + 1, angularSegments];
        int[,] topRingNodes = new int[radialSegments + 1, angularSegments];

        int CreateLocalNode(Vector3 position)
        {
            int localIndex = positions.Count;
            positions.Add(position);
            return localIndex;
        }

        int bottomCenter = CreateLocalNode(origin + new Vector3(0.0f, -halfThickness, 0.0f));
        int topCenter = CreateLocalNode(origin + new Vector3(0.0f, halfThickness, 0.0f));

        for (int ring = 1; ring <= radialSegments; ++ring)
        {
            float r = radius * ring / radialSegments;

            for (int segment = 0; segment < angularSegments; ++segment)
            {
                float angle = 2.0f * Mathf.PI * segment / angularSegments;
                float x = Mathf.Cos(angle) * r;
                float z = Mathf.Sin(angle) * r;

                bottomRingNodes[ring, segment] =
                    CreateLocalNode(origin + new Vector3(x, -halfThickness, z));

                topRingNodes[ring, segment] =
                    CreateLocalNode(origin + new Vector3(x, halfThickness, z));
            }
        }

        for (int segment = 0; segment < angularSegments; ++segment)
        {
            int next = (segment + 1) % angularSegments;

            int a = bottomCenter;
            int b = bottomRingNodes[1, segment];
            int c = bottomRingNodes[1, next];

            bottomTriangles.Add((a, b, c));
        }

        for (int ring = 1; ring < radialSegments; ++ring)
        {
            for (int segment = 0; segment < angularSegments; ++segment)
            {
                int next = (segment + 1) % angularSegments;

                int inner0 = bottomRingNodes[ring, segment];
                int inner1 = bottomRingNodes[ring, next];
                int outer0 = bottomRingNodes[ring + 1, segment];
                int outer1 = bottomRingNodes[ring + 1, next];

                bottomTriangles.Add((inner0, outer0, outer1));
                bottomTriangles.Add((inner0, outer1, inner1));
            }
        }

        int GetTopLocalNodeFromBottomLocalNode(int bottomLocalNode)
        {
            if (bottomLocalNode == bottomCenter)
            {
                return topCenter;
            }

            return bottomLocalNode + 1;
        }

        foreach ((int a, int b, int c) in bottomTriangles)
        {
            int at = GetTopLocalNodeFromBottomLocalNode(a);
            int bt = GetTopLocalNodeFromBottomLocalNode(b);
            int ct = GetTopLocalNodeFromBottomLocalNode(c);

            AddPrismTetsLocal(tets, positions, a, b, c, at, bt, ct);
        }

        nativeRestPositions = new Vec3[positions.Count];
        mappedVertices = new Vector3[positions.Count];

        for (int i = 0; i < positions.Count; ++i)
        {
            Vector3 p = positions[i];

            nativeRestPositions[i] = new Vec3
            {
                x = p.x,
                y = p.y,
                z = p.z
            };

            mappedVertices[i] = p;
        }

        tetLocalNodeIndices = tets.ToArray();

        mappedTetMeshHandle = PhysiKNative.PHYSIK_CreateTetMeshComponent(
            tissueHost.WorldHandle,
            nativeRestPositions,
            nativeRestPositions.Length,
            tetLocalNodeIndices,
            tetLocalNodeIndices.Length / 4);

        if (PhysiKNative.PHYSIK_IsComponentHandleValid(
                tissueHost.WorldHandle,
                mappedTetMeshHandle) == 0)
        {
            Debug.LogError("Failed to create mapped local TetMeshComponent.", this);
            initialized = false;
            return;
        }

        mapperHandle = PhysiKNative.PHYSIK_CreateTetMeshMapperComponent(
            tissueHost.WorldHandle,
            tissueHost.TetMeshHandle,
            mappedTetMeshHandle);

        if (PhysiKNative.PHYSIK_IsComponentHandleValid(
                tissueHost.WorldHandle,
                mapperHandle) == 0)
        {
            Debug.LogError("Failed to create TetMeshMapperComponent.", this);
            initialized = false;
            return;
        }

        if (drawWireframe)
        {
            EnsureWireframeExists();
            RebuildWireframeTopology();
            UpdateWireframeVertices();
        }

        initialized = true;

        if (logCreation)
        {
            Debug.Log(
                $"Mapped tet wireframe created. vertices={nativeRestPositions.Length}, " +
                $"tets={tetLocalNodeIndices.Length / 4}, " +
                $"wireEdges={wireframeLineIndices.Length / 2}. " +
                $"Mapper will auto-build during PHYSIK_Step.",
                this);
        }
    }

    private void UpdateMappedVerticesFromNative()
    {
        if (mappedVertices == null || mappedTetMeshHandle.IsValid == false)
        {
            return;
        }

        for (int i = 0; i < mappedVertices.Length; ++i)
        {
            int ok = PhysiKNative.PHYSIK_GetTetMeshLocalCurrentPosition(
                tissueHost.WorldHandle,
                mappedTetMeshHandle,
                i,
                out float x,
                out float y,
                out float z);

            if (ok != 0)
            {
                mappedVertices[i] = new Vector3(x, y, z);
            }
        }
    }
    private void EnsureWireframeExists()
    {
        if (wireframeObject == null)
        {
            CreateWireframeVisual();
            previousActiveTetCount = -1;
        }

        if (!wireframeObject.activeSelf)
        {
            wireframeObject.SetActive(true);
            previousActiveTetCount = -1;
        }
    }
    private void CreateWireframeVisual()
    {
        wireframeObject = new GameObject("PhysiK_MappedTetMesh_Wireframe");
        wireframeObject.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);

        wireframeMeshFilter = wireframeObject.AddComponent<MeshFilter>();
        wireframeMeshRenderer = wireframeObject.AddComponent<MeshRenderer>();

        wireframeMesh = new Mesh
        {
            name = "PhysiK_MappedTetMesh_Wireframe_Mesh",
            indexFormat = UnityEngine.Rendering.IndexFormat.UInt32
        };

        wireframeMesh.MarkDynamic();
        wireframeMeshFilter.sharedMesh = wireframeMesh;

        wireframeMeshRenderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        wireframeMeshRenderer.receiveShadows = false;

        if (wireframeMaterial != null)
        {
            wireframeMeshRenderer.sharedMaterial = wireframeMaterial;
        }
    }

    private void RebuildWireframeTopologyIfNeeded()
    {
        if (mappedTetMeshHandle.IsValid == false)
        {
            return;
        }

        int activeTetCount = PhysiKNative.PHYSIK_GetActiveTetCount(
            tissueHost.WorldHandle,
            mappedTetMeshHandle);

        if (activeTetCount == previousActiveTetCount)
        {
            return;
        }

        RebuildWireframeTopology();
        previousActiveTetCount = activeTetCount;
    }

    private void RebuildWireframeTopology()
    {
        if (wireframeMesh == null || tetLocalNodeIndices == null)
        {
            return;
        }

        HashSet<(int, int)> uniqueEdges = new HashSet<(int, int)>();
        int tetCount = tetLocalNodeIndices.Length / 4;

        for (int tet = 0; tet < tetCount; ++tet)
        {
            if (PhysiKNative.PHYSIK_IsTetActive(
                    tissueHost.WorldHandle,
                    mappedTetMeshHandle,
                    tet) == 0)
            {
                continue;
            }

            int baseIndex = tet * 4;

            int a = tetLocalNodeIndices[baseIndex + 0];
            int b = tetLocalNodeIndices[baseIndex + 1];
            int c = tetLocalNodeIndices[baseIndex + 2];
            int d = tetLocalNodeIndices[baseIndex + 3];

            AddWireEdge(uniqueEdges, a, b);
            AddWireEdge(uniqueEdges, a, c);
            AddWireEdge(uniqueEdges, a, d);
            AddWireEdge(uniqueEdges, b, c);
            AddWireEdge(uniqueEdges, b, d);
            AddWireEdge(uniqueEdges, c, d);
        }

        List<int> lines = new List<int>(uniqueEdges.Count * 2);
        foreach ((int a, int b) in uniqueEdges)
        {
            lines.Add(a);
            lines.Add(b);
        }

        wireframeLineIndices = lines.ToArray();

        wireframeMesh.Clear();
        wireframeMesh.vertices = mappedVertices;

        if (wireframeLineIndices.Length > 0)
        {
            wireframeMesh.SetIndices(wireframeLineIndices, MeshTopology.Lines, 0);
        }

        wireframeMesh.RecalculateBounds();
    }

    private void UpdateWireframeVertices()
    {
        if (wireframeMesh == null || mappedVertices == null)
        {
            return;
        }

        wireframeMesh.vertices = mappedVertices;

        if (wireframeLineIndices != null)
        {
            wireframeMesh.SetIndices(wireframeLineIndices, MeshTopology.Lines, 0);
        }

        wireframeMesh.RecalculateBounds();
    }

    private static void AddWireEdge(HashSet<(int, int)> edges, int a, int b)
    {
        if (a > b)
        {
            (a, b) = (b, a);
        }

        edges.Add((a, b));
    }

    private static void AddPrismTetsLocal(
        List<int> tets,
        List<Vector3> positions,
        int a,
        int b,
        int c,
        int at,
        int bt,
        int ct)
    {
        AddTetPositiveLocal(tets, positions, a, b, c, at);
        AddTetPositiveLocal(tets, positions, b, bt, c, at);
        AddTetPositiveLocal(tets, positions, c, bt, ct, at);
    }

    private static void AddTetPositiveLocal(
        List<int> tets,
        List<Vector3> positions,
        int n0,
        int n1,
        int n2,
        int n3)
    {
        float signedVolume6 = Vector3.Dot(
            Vector3.Cross(positions[n1] - positions[n0], positions[n2] - positions[n0]),
            positions[n3] - positions[n0]);

        if (Mathf.Abs(signedVolume6) < 1.0e-10f)
        {
            return;
        }

        if (signedVolume6 < 0.0f)
        {
            (n1, n2) = (n2, n1);
        }

        tets.Add(n0);
        tets.Add(n1);
        tets.Add(n2);
        tets.Add(n3);
    }

    private void OnDestroy()
    {
        if (wireframeObject != null)
        {
            Destroy(wireframeObject);
        }
    }
}