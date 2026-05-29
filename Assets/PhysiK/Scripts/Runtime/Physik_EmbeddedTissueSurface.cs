using System.Collections;
using UnityEngine;
using PhysiK.Unity;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class Physik_EmbeddedTissueSurface : MonoBehaviour
{
    [Header("Embedded Volumetric Tissue")]
    [SerializeField] private Physik_EmbeddedTissue embeddedVolumetricTissue;

    [Header("Rendering")]
    [SerializeField] private Material surfaceMaterial;
    [SerializeField] private bool recalculateNormals = true;
    [SerializeField] private bool recalculateBounds = true;

    [Header("Debug")]
    [SerializeField] private bool logCreation = true;

    private PhysiKComponentHandle surfaceExtractionHandle;

    private Mesh mesh;
    private Vector3[] vertices;
    private int[] nativeSurfaceTriangles;
    private int[] surfaceTriangles;

    private bool initialized;

    private IEnumerator Start()
    {
        if (embeddedVolumetricTissue == null)
        {
            Debug.LogError("Missing embeddedVolumetricTissue.", this);
            yield break;
        }

        yield return new WaitUntil(() => embeddedVolumetricTissue.IsInitialized);

        InitializeSurfaceExtraction();
    }

    private void LateUpdate()
    {
        if (!initialized)
        {
            return;
        }

        UpdateVerticesFromEmbeddedTetMesh();
        UpdateSurfaceTrianglesFromNative();

        mesh.Clear();
        mesh.vertices = vertices;

        if (surfaceTriangles != null)
        {
            mesh.triangles = surfaceTriangles;
        }

        if (recalculateNormals)
        {
            mesh.RecalculateNormals();
        }

        if (recalculateBounds)
        {
            mesh.RecalculateBounds();
        }
    }

    private void InitializeSurfaceExtraction()
    {
        int vertexCount = embeddedVolumetricTissue.VertexCount;
        if (vertexCount <= 0)
        {
            Debug.LogError("Embedded volumetric tissue has no vertices.", this);
            initialized = false;
            return;
        }

        vertices = new Vector3[vertexCount];

        surfaceExtractionHandle = PhysiKNative.PHYSIK_CreateSurfaceExtractionComponent(
            embeddedVolumetricTissue.WorldHandle,
            embeddedVolumetricTissue.MappedTetMeshHandle);

        if (PhysiKNative.PHYSIK_IsComponentHandleValid(
                embeddedVolumetricTissue.WorldHandle,
                surfaceExtractionHandle) == 0)
        {
            Debug.LogError("Failed to create SurfaceExtractionComponent.", this);
            initialized = false;
            return;
        }

        mesh = new Mesh
        {
            name = "PhysiK_SurfaceExtracted_VisualMesh",
            indexFormat = UnityEngine.Rendering.IndexFormat.UInt32
        };

        mesh.MarkDynamic();

        MeshFilter meshFilter = GetComponent<MeshFilter>();
        MeshRenderer meshRenderer = GetComponent<MeshRenderer>();

        meshFilter.sharedMesh = mesh;

        if (surfaceMaterial != null)
        {
            meshRenderer.sharedMaterial = surfaceMaterial;
        }

        initialized = true;

        if (logCreation)
        {
            Debug.Log(
                $"Surface extraction visual mesh initialized. vertexCount={vertexCount}",
                this);
        }
    }

    private void UpdateVerticesFromEmbeddedTetMesh()
    {
        if (vertices == null ||
            embeddedVolumetricTissue.MappedTetMeshHandle.IsValid == false)
        {
            return;
        }

        for (int i = 0; i < vertices.Length; ++i)
        {
            int ok = PhysiKNative.PHYSIK_GetTetMeshLocalCurrentPosition(
                embeddedVolumetricTissue.WorldHandle,
                embeddedVolumetricTissue.MappedTetMeshHandle,
                i,
                out float x,
                out float y,
                out float z);

            if (ok != 0)
            {
                vertices[i] = new Vector3(x, y, z);
            }
        }
    }

    private void UpdateSurfaceTrianglesFromNative()
    {
        if (surfaceExtractionHandle.IsValid == false)
        {
            return;
        }

        int nativeTriangleIndexCount = PhysiKNative.PHYSIK_GetSurfaceTriangleIndexCount(
            embeddedVolumetricTissue.WorldHandle,
            surfaceExtractionHandle);

        if (nativeTriangleIndexCount <= 0)
        {
            surfaceTriangles = System.Array.Empty<int>();
            return;
        }

        if (nativeSurfaceTriangles == null ||
            nativeSurfaceTriangles.Length != nativeTriangleIndexCount)
        {
            nativeSurfaceTriangles = new int[nativeTriangleIndexCount];
        }

        int copied = PhysiKNative.PHYSIK_CopySurfaceTriangleIndices(
            embeddedVolumetricTissue.WorldHandle,
            surfaceExtractionHandle,
            nativeSurfaceTriangles,
            nativeSurfaceTriangles.Length);

        if (copied <= 0)
        {
            return;
        }

        if (surfaceTriangles == null || surfaceTriangles.Length != copied)
        {
            surfaceTriangles = new int[copied];
        }

        System.Array.Copy(nativeSurfaceTriangles, surfaceTriangles, copied);
    }
}