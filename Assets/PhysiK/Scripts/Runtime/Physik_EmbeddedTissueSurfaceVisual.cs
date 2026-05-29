using System.Collections;
using UnityEngine;
using PhysiK.Unity;

[RequireComponent(typeof(MeshFilter))]
[RequireComponent(typeof(MeshRenderer))]
public class Physik_EmbeddedTissueSurfaceVisual : MonoBehaviour
{
    [Header("Embedded Tissue Surface")]
    [SerializeField] private Physik_EmbeddedTissueSurface embeddedTissueSurface;

    [Header("Rendering")]
    [SerializeField] private Material surfaceMaterial;
    [SerializeField] private bool recalculateBounds = true;

    [Header("Debug")]
    [SerializeField] private bool logCreation = true;

    private PhysiKComponentHandle surfaceVisualHandle;

    private MeshFilter meshFilter;
    private MeshRenderer meshRenderer;
    private Mesh mesh;

    private Vector3[] visualVertices;
    private Vector3[] visualNormals;
    private int[] visualTriangleIndices;

    private bool initialized;

    public bool IsInitialized => initialized;
    public PhysiKComponentHandle SurfaceVisualHandle => surfaceVisualHandle;

    private IEnumerator Start()
    {
        if (embeddedTissueSurface == null)
        {
            Debug.LogError("Missing embeddedTissueSurface.", this);
            yield break;
        }

        yield return new WaitUntil(() => embeddedTissueSurface.IsInitialized);

        CreateSurfaceVisualComponent();
    }

    private void LateUpdate()
    {
        if (!initialized)
        {
            return;
        }

        UpdateMeshFromNative();
    }

    private void CreateSurfaceVisualComponent()
    {
        System.IntPtr world = embeddedTissueSurface.WorldHandle;

        if (world == System.IntPtr.Zero)
        {
            Debug.LogError("Embedded tissue surface has invalid world handle.", this);
            initialized = false;
            return;
        }

        if (embeddedTissueSurface.SurfaceExtractionHandle.IsValid == false)
        {
            Debug.LogError("Embedded tissue surface has invalid surface extraction handle.", this);
            initialized = false;
            return;
        }

        meshFilter = GetComponent<MeshFilter>();
        meshRenderer = GetComponent<MeshRenderer>();

        mesh = new Mesh
        {
            name = "PhysiK_EmbeddedTissueSurfaceVisual_Mesh",
            indexFormat = UnityEngine.Rendering.IndexFormat.UInt32
        };

        mesh.MarkDynamic();
        meshFilter.sharedMesh = mesh;

        if (surfaceMaterial != null)
        {
            meshRenderer.sharedMaterial = surfaceMaterial;
        }

        surfaceVisualHandle = PhysiKNative.PHYSIK_CreateSurfaceVisualComponent(
            world,
            embeddedTissueSurface.SurfaceExtractionHandle);

        if (PhysiKNative.PHYSIK_IsComponentHandleValid(
                world,
                surfaceVisualHandle) == 0)
        {
            Debug.LogError("Failed to create SurfaceVisualComponent.", this);
            initialized = false;
            return;
        }

        initialized = true;

        UpdateMeshFromNative();

        if (logCreation)
        {
            Debug.Log(
                $"Embedded tissue surface visual created. " +
                $"vertices={visualVertices?.Length ?? 0}, " +
                $"triangles={(visualTriangleIndices?.Length ?? 0) / 3}, " +
                $"normals={visualNormals?.Length ?? 0}.",
                this);
        }
    }

    private void UpdateMeshFromNative()
    {
        System.IntPtr world = embeddedTissueSurface.WorldHandle;

        int vertexCount = PhysiKNative.PHYSIK_GetSurfaceVisualVertexCount(
            world,
            surfaceVisualHandle);

        int triangleIndexCount = PhysiKNative.PHYSIK_GetSurfaceVisualTriangleIndexCount(
            world,
            surfaceVisualHandle);

        int normalCount = PhysiKNative.PHYSIK_GetSurfaceVisualNormalCount(
            world,
            surfaceVisualHandle);

        if (vertexCount <= 0 || triangleIndexCount <= 0 || normalCount <= 0)
        {
            mesh.Clear();
            return;
        }

        if (triangleIndexCount % 3 != 0)
        {
            Debug.LogWarning(
                $"Surface visual triangle index count is not divisible by 3: {triangleIndexCount}",
                this);
            return;
        }

        if (normalCount != vertexCount)
        {
            Debug.LogWarning(
                $"Surface visual normal count does not match vertex count. " +
                $"vertices={vertexCount}, normals={normalCount}",
                this);
            return;
        }

        EnsureBuffers(vertexCount, triangleIndexCount, normalCount);

        ReadVisualVertices(world, vertexCount);
        ReadVisualTriangleIndices(world, triangleIndexCount);
        ReadVisualNormals(world, normalCount);

        mesh.Clear();
        mesh.vertices = visualVertices;
        mesh.triangles = visualTriangleIndices;
        mesh.normals = visualNormals;

        if (recalculateBounds)
        {
            mesh.RecalculateBounds();
        }
    }

    private void EnsureBuffers(
        int vertexCount,
        int triangleIndexCount,
        int normalCount)
    {
        if (visualVertices == null || visualVertices.Length != vertexCount)
        {
            visualVertices = new Vector3[vertexCount];
        }

        if (visualTriangleIndices == null ||
            visualTriangleIndices.Length != triangleIndexCount)
        {
            visualTriangleIndices = new int[triangleIndexCount];
        }

        if (visualNormals == null || visualNormals.Length != normalCount)
        {
            visualNormals = new Vector3[normalCount];
        }
    }

    private void ReadVisualVertices(System.IntPtr world, int vertexCount)
    {
        for (int i = 0; i < vertexCount; ++i)
        {
            int ok = PhysiKNative.PHYSIK_GetSurfaceVisualVertex(
                world,
                surfaceVisualHandle,
                i,
                out float x,
                out float y,
                out float z);

            if (ok != 0)
            {
                visualVertices[i] = new Vector3(x, y, z);
            }
        }
    }

    private void ReadVisualTriangleIndices(System.IntPtr world, int triangleIndexCount)
    {
        for (int i = 0; i < triangleIndexCount; ++i)
        {
            int ok = PhysiKNative.PHYSIK_GetSurfaceVisualTriangleIndex(
                world,
                surfaceVisualHandle,
                i,
                out int index);

            if (ok != 0)
            {
                visualTriangleIndices[i] = index;
            }
        }
    }

    private void ReadVisualNormals(System.IntPtr world, int normalCount)
    {
        for (int i = 0; i < normalCount; ++i)
        {
            int ok = PhysiKNative.PHYSIK_GetSurfaceVisualNormal(
                world,
                surfaceVisualHandle,
                i,
                out float x,
                out float y,
                out float z);

            if (ok != 0)
            {
                visualNormals[i] = new Vector3(x, y, z);
            }
        }
    }
}