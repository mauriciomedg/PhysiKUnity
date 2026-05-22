using UnityEngine;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class Physik_VisualMesh : MonoBehaviour
{
    [SerializeField] private PhysikCircularTissue tissueHost;

    private Mesh mesh;
    private PhysiKComponentHandle visualMeshHandle;

    private Vec3[] nativeVertices;
    private int[] nativeTriangles;
    private Vector3[] unityVertices;

    private bool initialized;

    private void Start()
    {
        if (tissueHost == null)
            return;

        mesh = GetComponent<MeshFilter>().mesh;

        Vector3[] sourceVertices = mesh.vertices;
        nativeTriangles = mesh.triangles;
        nativeVertices = new Vec3[sourceVertices.Length];
        unityVertices = new Vector3[sourceVertices.Length];

        for (int i = 0; i < sourceVertices.Length; ++i)
        {
            nativeVertices[i] = new Vec3
            {
                x = sourceVertices[i].x,
                y = sourceVertices[i].y,
                z = sourceVertices[i].z
            };
        }

        visualMeshHandle = PhysiKNative.PHYSIK_CreateVisualMeshComponent(
            tissueHost.WorldHandle,
            tissueHost.TetMeshHandle);

        PhysiKNative.PHYSIK_SetVisualMeshData(
            tissueHost.WorldHandle,
            visualMeshHandle,
            nativeVertices,
            nativeVertices.Length,
            nativeTriangles,
            nativeTriangles.Length);

        int ok = PhysiKNative.PHYSIK_BuildVisualMeshEmbedding(
            tissueHost.WorldHandle,
            visualMeshHandle);

        initialized = ok != 0;
    }

    private void LateUpdate()
    {
        if (!initialized || tissueHost == null)
            return;

        int copied = PhysiKNative.PHYSIK_CopyVisualMeshVertices(
            tissueHost.WorldHandle,
            visualMeshHandle,
            nativeVertices,
            nativeVertices.Length);

        for (int i = 0; i < copied; ++i)
        {
            unityVertices[i] = new Vector3(
                nativeVertices[i].x,
                nativeVertices[i].y,
                nativeVertices[i].z);
        }

        mesh.vertices = unityVertices;
        mesh.triangles = nativeTriangles;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
    }
}