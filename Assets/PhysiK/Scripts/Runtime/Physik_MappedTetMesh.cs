using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using PhysiK.Unity;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class Physik_MappedTetMesh : MonoBehaviour
{
    [Header("Host")]
    [SerializeField] private PhysikCircularTissue tissueHost;

    [Header("Mapped Tet Mesh")]
    [SerializeField] private int radialSegments = 6;
    [SerializeField] private int angularSegments = 30;
    [SerializeField] private float radius = 2.0f;
    [SerializeField] private float thickness = 0.12f;
    [SerializeField] private float yOffset = 0.0f;

    [Header("Debug")]
    [SerializeField] private bool logCreation = true;

    private PhysiKComponentHandle mappedTetMeshHandle;
    private PhysiKComponentHandle mapperHandle;

    private Mesh mesh;

    private Vec3[] nativeRestPositions;
    private Vector3[] unityVertices;
    private int[] tetLocalNodeIndices;
    private int[] surfaceTriangles;

    private bool initialized;

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
        List<int> topSurfaceTriangles = new List<int>();
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

            // Render only the top layer for now.
            topSurfaceTriangles.Add(at);
            topSurfaceTriangles.Add(ct);
            topSurfaceTriangles.Add(bt);
        }

        nativeRestPositions = new Vec3[positions.Count];
        unityVertices = new Vector3[positions.Count];

        for (int i = 0; i < positions.Count; ++i)
        {
            Vector3 p = positions[i];

            nativeRestPositions[i] = new Vec3
            {
                x = p.x,
                y = p.y,
                z = p.z
            };

            unityVertices[i] = p;
        }

        tetLocalNodeIndices = tets.ToArray();
        surfaceTriangles = topSurfaceTriangles.ToArray();

        CreateUnityMesh();

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

        // No manual PHYSIK_BuildTetMeshMapping call here.
        // The mapper should mark itself dirty on creation and build automatically
        // during the normal native world step.
        initialized = true;

        if (logCreation)
        {
            Debug.Log(
                $"Mapped tet mesh created. vertices={nativeRestPositions.Length}, " +
                $"tets={tetLocalNodeIndices.Length / 4}, " +
                $"surfaceTriangles={surfaceTriangles.Length / 3}. " +
                $"Mapper will auto-build during PHYSIK_Step.",
                this);
        }
    }

    private void CreateUnityMesh()
    {
        mesh = new Mesh
        {
            name = "PhysiK_Mapped_TetMesh_Surface",
            indexFormat = UnityEngine.Rendering.IndexFormat.UInt32
        };

        mesh.MarkDynamic();
        mesh.vertices = unityVertices;
        mesh.triangles = surfaceTriangles;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();

        GetComponent<MeshFilter>().sharedMesh = mesh;
    }

    private void UpdateMappedVerticesFromNative()
    {
        if (unityVertices == null || mappedTetMeshHandle.IsValid == false)
        {
            return;
        }

        for (int i = 0; i < unityVertices.Length; ++i)
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
                unityVertices[i] = new Vector3(x, y, z);
            }
        }

        mesh.vertices = unityVertices;

        if (surfaceTriangles != null)
        {
            mesh.triangles = surfaceTriangles;
        }

        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
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
}