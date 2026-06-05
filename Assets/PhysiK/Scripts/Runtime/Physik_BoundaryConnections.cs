using System;
using System.Collections.Generic;
using UnityEngine;
using PhysiK.Unity;

[RequireComponent(typeof(Physik_MechanicalTissue))]
public class Physik_BoundaryConnections :
    Physik_ScriptComponent
{
    [Header("Mechanical Tissue")]
    [SerializeField]
    private Physik_MechanicalTissue mechanicalTissue;

    [Header("Boundary Point Connections")]
    [SerializeField]
    private float boundaryStiffness =
        20000.0f;

    [SerializeField]
    private float boundaryDamping =
        0.0f;

    [SerializeField]
    private float radialBoundaryOffset =
        0.15f;

    [Header("Boundary Marker Debug Draw")]
    [SerializeField]
    private bool drawBoundaryMarkers =
        true;

    [SerializeField]
    private Material boundaryMarkerMaterial;

    [SerializeField]
    private float boundaryMarkerRadius =
        0.035f;

    [Header("Point Connection Line Debug Draw")]
    [SerializeField]
    private bool drawPointConnectionLines =
        true;

    [SerializeField]
    private Material pointConnectionLineMaterial;

    private bool initialized;

    private IntPtr world =
        IntPtr.Zero;

    private int[] boundaryLocalNodeIndices;

    private Vector3[] boundaryAnchorPositions;

    // Boundary marker debug draw.
    private GameObject boundaryMarkerObject;

    private Mesh boundaryMarkerMesh;

    private MeshFilter boundaryMarkerMeshFilter;

    private MeshRenderer boundaryMarkerMeshRenderer;

    private int[] boundaryMarkerTriangles;

    // Point-connection line debug draw.
    private GameObject pointConnectionLineObject;

    private Mesh pointConnectionLineMesh;

    private MeshFilter pointConnectionLineMeshFilter;

    private MeshRenderer pointConnectionLineMeshRenderer;

    private int[] pointConnectionLineIndices;

    private Vector3[] pointConnectionLineVertices;

    public bool IsInitialized =>
        initialized;

    public int BoundaryNodeCount =>
        boundaryLocalNodeIndices !=
        null
            ? boundaryLocalNodeIndices.Length
            : 0;

    private void Start()
    {
        TryInitialize();
    }

    private void Update()
    {
        if (!initialized)
        {
            TryInitialize();

            return;
        }

        UpdateDebugVisuals();
    }

    private void TryInitialize()
    {
        if (initialized)
        {
            return;
        }

        if (mechanicalTissue ==
            null)
        {
            mechanicalTissue =
                GetComponent<
                    Physik_MechanicalTissue>();
        }

        if (mechanicalTissue ==
                null ||
            !mechanicalTissue.IsInitialized)
        {
            return;
        }

        Initialize(
            mechanicalTissue);
    }

    private bool Initialize(
        Physik_MechanicalTissue tissue)
    {
        if (initialized)
        {
            return true;
        }

        mechanicalTissue =
            tissue;

        if (mechanicalTissue ==
            null)
        {
            Debug.LogError(
                "Physik_MechanicalTissue is not assigned.",
                this);

            return false;
        }

        if (!mechanicalTissue.IsInitialized)
        {
            return false;
        }

        world =
            mechanicalTissue.WorldHandle;

        if (world ==
            IntPtr.Zero)
        {
            Debug.LogError(
                "The mechanical tissue has no valid native world.",
                this);

            return false;
        }

        Vector3[] positions =
            mechanicalTissue.NodeWorldPositions;

        if (positions ==
                null ||
            positions.Length ==
                0)
        {
            Debug.LogError(
                "Mechanical tissue has no generated node positions.",
                this);

            return false;
        }

        BuildRadialBoundaryNodes(
            positions,
            mechanicalTissue.TissueCenter,
            mechanicalTissue.TissueRadius);

        if (!TryInitializeNativeScriptComponent())
        {
            Debug.LogError(
                "Failed to initialize the native ScriptComponent for Physik_BoundaryConnections.",
                this);

            return false;
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

        initialized =
            true;

        Debug.Log(
            $"Boundary connections initialized. " +
            $"boundaryNodes={BoundaryNodeCount}, " +
            $"stiffness={boundaryStiffness}, " +
            $"damping={boundaryDamping}, " +
            $"radialOffset={radialBoundaryOffset}.",
            this);

        return true;
    }

    protected override IntPtr
        GetScriptWorldHandle()
    {
        return world;
    }

    protected override bool
        CanInitializeScriptComponent()
    {
        return world !=
                IntPtr.Zero &&
            mechanicalTissue !=
                null &&
            boundaryLocalNodeIndices !=
                null &&
            boundaryAnchorPositions !=
                null;
    }

    protected override void
        OnPhysikPreUpdate()
    {
        if (!initialized ||
            world ==
                IntPtr.Zero)
        {
            return;
        }

        AddBoundaryPointConnections();
    }

    private void BuildRadialBoundaryNodes(
        Vector3[] positions,
        Vector3 center,
        float radius)
    {
        List<int> boundary =
            new List<int>();

        float tolerance =
            Mathf.Max(
                1.0e-4f,
                radius *
                1.0e-4f);

        for (int localNode = 0;
             localNode <
                 positions.Length;
             ++localNode)
        {
            Vector3 position =
                positions[
                    localNode];

            float radialDistance =
                new Vector2(
                    position.x -
                    center.x,
                    position.z -
                    center.z)
                .magnitude;

            if (Mathf.Abs(
                    radialDistance -
                    radius) <=
                tolerance)
            {
                boundary.Add(
                    localNode);
            }
        }

        boundaryLocalNodeIndices =
            boundary.ToArray();

        Array.Sort(
            boundaryLocalNodeIndices);

        boundaryAnchorPositions =
            new Vector3[
                boundaryLocalNodeIndices
                    .Length];

        for (int i = 0;
             i <
                 boundaryLocalNodeIndices
                     .Length;
             ++i)
        {
            int localNode =
                boundaryLocalNodeIndices[
                    i];

            Vector3 originalPosition =
                positions[
                    localNode];

            Vector3 radial =
                new Vector3(
                    originalPosition.x -
                    center.x,
                    0.0f,
                    originalPosition.z -
                    center.z);

            if (radial.sqrMagnitude >
                1.0e-8f)
            {
                radial.Normalize();
            }
            else
            {
                radial =
                    Vector3.zero;
            }

            boundaryAnchorPositions[
                i] =
                originalPosition +
                radial *
                radialBoundaryOffset;
        }
    }

    private void AddBoundaryPointConnections()
    {
        if (mechanicalTissue ==
                null ||
            boundaryLocalNodeIndices ==
                null ||
            boundaryAnchorPositions ==
                null)
        {
            return;
        }

        int[] nodes =
            mechanicalTissue
                .GlobalNodeIndices;

        if (nodes ==
            null)
        {
            return;
        }

        for (int i = 0;
             i <
                 boundaryLocalNodeIndices
                     .Length;
             ++i)
        {
            int localNodeIndex =
                boundaryLocalNodeIndices[
                    i];

            if (localNodeIndex <
                    0 ||
                localNodeIndex >=
                    nodes.Length)
            {
                continue;
            }

            int globalNodeIndex =
                nodes[
                    localNodeIndex];

            Vector3 target =
                boundaryAnchorPositions[
                    i];

            PhysiKNative
                .PHYSIK_AddPointConnection(
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

    private void UpdateDebugVisuals()
    {
        if (!initialized ||
            world ==
                IntPtr.Zero)
        {
            return;
        }

        if (drawBoundaryMarkers)
        {
            UpdateBoundaryMarkerVertices();
        }

        if (drawPointConnectionLines)
        {
            UpdatePointConnectionLineVertices();
        }
    }

    private void CreateBoundaryMarkerVisual()
    {
        boundaryMarkerObject =
            new GameObject(
                "PhysiK_PointConnected_Boundary_Nodes_Debug");

        boundaryMarkerObject
            .transform
            .SetPositionAndRotation(
                Vector3.zero,
                Quaternion.identity);

        boundaryMarkerMeshFilter =
            boundaryMarkerObject
                .AddComponent<
                    MeshFilter>();

        boundaryMarkerMeshRenderer =
            boundaryMarkerObject
                .AddComponent<
                    MeshRenderer>();

        boundaryMarkerMesh =
            new Mesh
            {
                name =
                    "PhysiK_PointConnected_Boundary_Nodes_Debug_Mesh",

                indexFormat =
                    UnityEngine.Rendering
                        .IndexFormat
                        .UInt32
            };

        boundaryMarkerMesh
            .MarkDynamic();

        boundaryMarkerMeshFilter
            .sharedMesh =
            boundaryMarkerMesh;

        if (boundaryMarkerMaterial !=
            null)
        {
            boundaryMarkerMeshRenderer
                .sharedMaterial =
                boundaryMarkerMaterial;
        }
    }

    private void RebuildBoundaryMarkerTopology()
    {
        if (boundaryMarkerMesh ==
                null ||
            boundaryLocalNodeIndices ==
                null)
        {
            return;
        }

        int markerCount =
            boundaryLocalNodeIndices
                .Length;

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
             i <
                 markerCount;
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

            triangles.Add(
                py);

            triangles.Add(
                px);

            triangles.Add(
                pz);

            triangles.Add(
                py);

            triangles.Add(
                pz);

            triangles.Add(
                nx);

            triangles.Add(
                py);

            triangles.Add(
                nx);

            triangles.Add(
                nz);

            triangles.Add(
                py);

            triangles.Add(
                nz);

            triangles.Add(
                px);

            triangles.Add(
                ny);

            triangles.Add(
                pz);

            triangles.Add(
                px);

            triangles.Add(
                ny);

            triangles.Add(
                nx);

            triangles.Add(
                pz);

            triangles.Add(
                ny);

            triangles.Add(
                nz);

            triangles.Add(
                nx);

            triangles.Add(
                ny);

            triangles.Add(
                px);

            triangles.Add(
                nz);
        }

        boundaryMarkerTriangles =
            triangles.ToArray();

        boundaryMarkerMesh.Clear();

        boundaryMarkerMesh.vertices =
            vertices;

        boundaryMarkerMesh.triangles =
            boundaryMarkerTriangles;

        boundaryMarkerMesh
            .RecalculateNormals();

        boundaryMarkerMesh
            .RecalculateBounds();
    }

    private void UpdateBoundaryMarkerVertices()
    {
        if (boundaryMarkerMesh ==
                null ||
            boundaryLocalNodeIndices ==
                null ||
            mechanicalTissue ==
                null ||
            mechanicalTissue
                .NodeWorldPositions ==
                null)
        {
            return;
        }

        Vector3[] nodeWorldPositions =
            mechanicalTissue
                .NodeWorldPositions;

        int markerCount =
            boundaryLocalNodeIndices
                .Length;

        Vector3[] vertices =
            boundaryMarkerMesh
                .vertices;

        if (vertices ==
                null ||
            vertices.Length !=
                markerCount *
                6)
        {
            RebuildBoundaryMarkerTopology();

            vertices =
                boundaryMarkerMesh
                    .vertices;
        }

        float markerRadius =
            Mathf.Max(
                0.001f,
                boundaryMarkerRadius);

        for (int i = 0;
             i <
                 markerCount;
             ++i)
        {
            int localNode =
                boundaryLocalNodeIndices[
                    i];

            if (localNode <
                    0 ||
                localNode >=
                    nodeWorldPositions
                        .Length)
            {
                continue;
            }

            Vector3 position =
                nodeWorldPositions[
                    localNode];

            int baseVertex =
                i *
                6;

            vertices[
                baseVertex +
                0] =
                position +
                new Vector3(
                    markerRadius,
                    0.0f,
                    0.0f);

            vertices[
                baseVertex +
                1] =
                position +
                new Vector3(
                    -markerRadius,
                    0.0f,
                    0.0f);

            vertices[
                baseVertex +
                2] =
                position +
                new Vector3(
                    0.0f,
                    markerRadius,
                    0.0f);

            vertices[
                baseVertex +
                3] =
                position +
                new Vector3(
                    0.0f,
                    -markerRadius,
                    0.0f);

            vertices[
                baseVertex +
                4] =
                position +
                new Vector3(
                    0.0f,
                    0.0f,
                    markerRadius);

            vertices[
                baseVertex +
                5] =
                position +
                new Vector3(
                    0.0f,
                    0.0f,
                    -markerRadius);
        }

        boundaryMarkerMesh.vertices =
            vertices;

        if (boundaryMarkerTriangles !=
            null)
        {
            boundaryMarkerMesh.triangles =
                boundaryMarkerTriangles;
        }

        boundaryMarkerMesh
            .RecalculateNormals();

        boundaryMarkerMesh
            .RecalculateBounds();
    }

    private void CreatePointConnectionLineVisual()
    {
        pointConnectionLineObject =
            new GameObject(
                "PhysiK_PointConnection_Lines_Debug");

        pointConnectionLineObject
            .transform
            .SetPositionAndRotation(
                Vector3.zero,
                Quaternion.identity);

        pointConnectionLineMeshFilter =
            pointConnectionLineObject
                .AddComponent<
                    MeshFilter>();

        pointConnectionLineMeshRenderer =
            pointConnectionLineObject
                .AddComponent<
                    MeshRenderer>();

        pointConnectionLineMesh =
            new Mesh
            {
                name =
                    "PhysiK_PointConnection_Lines_Debug_Mesh",

                indexFormat =
                    UnityEngine.Rendering
                        .IndexFormat
                        .UInt32
            };

        pointConnectionLineMesh
            .MarkDynamic();

        pointConnectionLineMeshFilter
            .sharedMesh =
            pointConnectionLineMesh;

        if (pointConnectionLineMaterial !=
            null)
        {
            pointConnectionLineMeshRenderer
                .sharedMaterial =
                pointConnectionLineMaterial;
        }
    }

    private void RebuildPointConnectionLineTopology()
    {
        if (pointConnectionLineMesh ==
                null ||
            boundaryLocalNodeIndices ==
                null ||
            boundaryAnchorPositions ==
                null)
        {
            return;
        }

        int connectionCount =
            boundaryLocalNodeIndices
                .Length;

        pointConnectionLineVertices =
            new Vector3[
                connectionCount *
                2];

        pointConnectionLineIndices =
            new int[
                connectionCount *
                2];

        for (int i = 0;
             i <
                 connectionCount;
             ++i)
        {
            int baseIndex =
                i *
                2;

            pointConnectionLineIndices[
                baseIndex +
                0] =
                baseIndex +
                0;

            pointConnectionLineIndices[
                baseIndex +
                1] =
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

        pointConnectionLineMesh
            .RecalculateBounds();
    }

    private void UpdatePointConnectionLineVertices()
    {
        if (pointConnectionLineMesh ==
                null ||
            boundaryLocalNodeIndices ==
                null ||
            boundaryAnchorPositions ==
                null ||
            mechanicalTissue ==
                null ||
            mechanicalTissue
                .NodeWorldPositions ==
                null)
        {
            return;
        }

        Vector3[] nodeWorldPositions =
            mechanicalTissue
                .NodeWorldPositions;

        int connectionCount =
            boundaryLocalNodeIndices
                .Length;

        if (pointConnectionLineVertices ==
                null ||
            pointConnectionLineVertices
                .Length !=
                connectionCount *
                2)
        {
            RebuildPointConnectionLineTopology();
        }

        for (int i = 0;
             i <
                 connectionCount;
             ++i)
        {
            int localNodeIndex =
                boundaryLocalNodeIndices[
                    i];

            if (localNodeIndex <
                    0 ||
                localNodeIndex >=
                    nodeWorldPositions
                        .Length)
            {
                continue;
            }

            Vector3 nodePosition =
                nodeWorldPositions[
                    localNodeIndex];

            Vector3 anchorPosition =
                boundaryAnchorPositions[
                    i];

            int baseIndex =
                i *
                2;

            pointConnectionLineVertices[
                baseIndex +
                0] =
                nodePosition;

            pointConnectionLineVertices[
                baseIndex +
                1] =
                anchorPosition;
        }

        pointConnectionLineMesh.vertices =
            pointConnectionLineVertices;

        if (pointConnectionLineIndices !=
            null)
        {
            pointConnectionLineMesh.SetIndices(
                pointConnectionLineIndices,
                MeshTopology.Lines,
                0);
        }

        pointConnectionLineMesh
            .RecalculateBounds();
    }

    private void OnValidate()
    {
        boundaryStiffness =
            Mathf.Max(
                0.0f,
                boundaryStiffness);

        boundaryDamping =
            Mathf.Max(
                0.0f,
                boundaryDamping);

        boundaryMarkerRadius =
            Mathf.Max(
                0.001f,
                boundaryMarkerRadius);
    }

    protected override void OnDestroy()
    {
        DestroyNativeScriptComponent();

        initialized =
            false;

        world =
            IntPtr.Zero;

        if (boundaryMarkerObject !=
            null)
        {
            Destroy(
                boundaryMarkerObject);
        }

        if (pointConnectionLineObject !=
            null)
        {
            Destroy(
                pointConnectionLineObject);
        }

        base.OnDestroy();
    }
}
