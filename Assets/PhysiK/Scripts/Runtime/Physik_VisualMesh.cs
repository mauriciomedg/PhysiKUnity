using UnityEngine;
using PhysiK.Unity;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class Physik_VisualMesh : MonoBehaviour
{
    [SerializeField] private PhysikCircularTissue tissueHost;
    [SerializeField] private int radialSegments = 40;
    [SerializeField] private int angularSegments = 160;
    [SerializeField] private float radius = 2.0f;
    [SerializeField] private float yOffset = 0.06f;

    private Mesh mesh;
    private PhysiKComponentHandle visualMeshHandle;

    private Vec3[] nativeVertices;
    private Vector3[] unityVertices;
    private int[] triangles;

    private bool initialized;

    private void InitializeVisualMesh()
    {
        if (tissueHost == null)
        {
            Debug.LogError("Missing tissueHost.", this);
            return;
        }

        CreateCircularVisualMesh();

        visualMeshHandle = PhysiKNative.PHYSIK_CreateVisualMeshComponent(
            tissueHost.WorldHandle,
            tissueHost.TetMeshHandle);

        PhysiKNative.PHYSIK_SetVisualMeshData(
            tissueHost.WorldHandle,
            visualMeshHandle,
            nativeVertices,
            nativeVertices.Length,
            triangles,
            triangles.Length);

        int ok = PhysiKNative.PHYSIK_BuildVisualMeshEmbedding(
            tissueHost.WorldHandle,
            visualMeshHandle);

        initialized = ok != 0;

        Debug.Log($"Visual mesh embedding result: {ok}, vertices={nativeVertices.Length}, triangles={triangles.Length / 3}", this);

    }
    private System.Collections.IEnumerator Start()
    {
        if (tissueHost == null)
        {
            Debug.LogError("Missing tissueHost.", this);
            yield break;
        }

        yield return new WaitUntil(() => tissueHost.IsInitialized);

        InitializeVisualMesh();
    }

    private void LateUpdate()
    {
        if (!initialized)
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
        mesh.triangles = triangles;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
    }

    private void CreateCircularVisualMesh()
    {
        radialSegments = Mathf.Max(1, radialSegments);
        angularSegments = Mathf.Max(8, angularSegments);

        int vertexCount = 1 + radialSegments * angularSegments;
        nativeVertices = new Vec3[vertexCount];
        unityVertices = new Vector3[vertexCount];

        float y = tissueHost.TissuePlaneY + yOffset;

        nativeVertices[0] = new Vec3 { x = 0.0f, y = y, z = 0.0f };
        unityVertices[0] = new Vector3(0.0f, y, 0.0f);

        int index = 1;
        for (int ring = 1; ring <= radialSegments; ++ring)
        {
            float r = radius * ring / radialSegments;

            for (int segment = 0; segment < angularSegments; ++segment)
            {
                float angle = 2.0f * Mathf.PI * segment / angularSegments;
                float x = Mathf.Cos(angle) * r;
                float z = Mathf.Sin(angle) * r;

                nativeVertices[index] = new Vec3 { x = x, y = y, z = z };
                unityVertices[index] = new Vector3(x, y, z);
                ++index;
            }
        }

        System.Collections.Generic.List<int> tris = new();

        for (int segment = 0; segment < angularSegments; ++segment)
        {
            int next = (segment + 1) % angularSegments;
            tris.Add(0);
            tris.Add(1 + segment);
            tris.Add(1 + next);
        }

        for (int ring = 1; ring < radialSegments; ++ring)
        {
            int innerStart = 1 + (ring - 1) * angularSegments;
            int outerStart = 1 + ring * angularSegments;

            for (int segment = 0; segment < angularSegments; ++segment)
            {
                int next = (segment + 1) % angularSegments;

                int inner0 = innerStart + segment;
                int inner1 = innerStart + next;
                int outer0 = outerStart + segment;
                int outer1 = outerStart + next;

                tris.Add(inner0);
                tris.Add(outer0);
                tris.Add(outer1);

                tris.Add(inner0);
                tris.Add(outer1);
                tris.Add(inner1);
            }
        }

        triangles = tris.ToArray();

        mesh = new Mesh
        {
            name = "Physik Embedded Visual Mesh",
            indexFormat = UnityEngine.Rendering.IndexFormat.UInt32
        };

        mesh.MarkDynamic();
        mesh.vertices = unityVertices;
        mesh.triangles = triangles;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();

        GetComponent<MeshFilter>().sharedMesh = mesh;
    }
}