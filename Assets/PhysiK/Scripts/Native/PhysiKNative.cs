using System;
using System.Runtime.InteropServices;

[StructLayout(LayoutKind.Sequential)]
public struct PhysiKComponentHandle
{
    public uint index;
    public uint generation;

    public bool IsValid => index != 0xFFFFFFFFu && generation != 0u;
}

[StructLayout(LayoutKind.Sequential)]
public struct PhysikMaterialDesc
{
    public float density;
    public float youngModulus;
    public float poissonRatio;
    public float damping;
}

[StructLayout(LayoutKind.Sequential)]
public struct PhysiKGeneratedTetMeshHandle
{
    public IntPtr value;

    public bool IsValid => value != IntPtr.Zero;
}

public enum PhysiKFemModel : uint
{
    Linear = 0,
    Corotational = 1,
    NeoHookean = 2
}

public enum PhysikOverlapGeometryType
{
    Unknown = 0,
    Tetrahedron = 1,
    Triangle = 2,
    Sphere = 3,
    Node = 4
}

[StructLayout(LayoutKind.Sequential)]
public struct Vec3
{
    public float x;
    public float y;
    public float z;
}

public static class PhysiKNative
{
    private const string DllName = "PhysiK";

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern IntPtr PHYSIK_CreateWorld();

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void PHYSIK_DestroyWorld(IntPtr world);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void PHYSIK_Step(IntPtr world, float dt);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void PHYSIK_SetSubstepCount(IntPtr world, int substepCount);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void PHYSIK_SetSolverMode(IntPtr world, int mode);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void PHYSIK_SetGravity(
        IntPtr world,
        float x,
        float y,
        float z);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int PHYSIK_AddNode(
        IntPtr world,
        float x,
        float y,
        float z);

    // -----------------------------------------------------------------
    // Generated tet mesh
    // -----------------------------------------------------------------

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern PhysiKGeneratedTetMeshHandle PHYSIK_GenerateTetMesh(
        [In] Vec3[] positions,
        int nodeCount,
        [In] int[] tetLocalNodeIndices,
        int tetCount);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int PHYSIK_IsGeneratedTetMeshHandleValid(
        PhysiKGeneratedTetMeshHandle generatedTetMeshHandle);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void PHYSIK_DestroyGeneratedTetMesh(
        PhysiKGeneratedTetMeshHandle generatedTetMeshHandle);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int PHYSIK_GetGeneratedTetMeshVertexCount(
        PhysiKGeneratedTetMeshHandle generatedTetMeshHandle);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int PHYSIK_GetGeneratedTetMeshTetCount(
        PhysiKGeneratedTetMeshHandle generatedTetMeshHandle);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int PHYSIK_GetGeneratedTetMeshTetIndexCount(
        PhysiKGeneratedTetMeshHandle generatedTetMeshHandle);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int PHYSIK_GetGeneratedTetMeshVertex(
        PhysiKGeneratedTetMeshHandle generatedTetMeshHandle,
        int vertexIndex,
        out float outX,
        out float outY,
        out float outZ);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int PHYSIK_GetGeneratedTetMeshTetNodeIndex(
        PhysiKGeneratedTetMeshHandle generatedTetMeshHandle,
        int tetIndexArrayIndex,
        out int outNodeIndex);

    // -----------------------------------------------------------------
    // Tet mesh components
    // Components consume generated meshes only.
    // -----------------------------------------------------------------

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern PhysiKComponentHandle PHYSIK_CreateTetMeshComponent(
        IntPtr world,
        PhysiKGeneratedTetMeshHandle generatedTetMeshHandle);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern PhysiKComponentHandle PHYSIK_CreateTetMeshPhysicsComponent(
        IntPtr world,
        PhysiKGeneratedTetMeshHandle generatedTetMeshHandle,
        ref PhysikMaterialDesc material);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern PhysiKComponentHandle PHYSIK_CreateTetMeshMapperComponent(
        IntPtr world,
        PhysiKComponentHandle sourceTetMesh,
        PhysiKComponentHandle destinationTetMesh);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int PHYSIK_GetTetMeshLocalCurrentPosition(
        IntPtr world,
        PhysiKComponentHandle tetMesh,
        int localNodeIndex,
        out float x,
        out float y,
        out float z);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void PHYSIK_SetTetMeshMaterial(
        IntPtr world,
        PhysiKComponentHandle component,
        ref PhysikMaterialDesc material);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int PHYSIK_IsTetActive(
    IntPtr world,
    PhysiKComponentHandle component,
    int tetIndex);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void PHYSIK_SetTetActive(
        IntPtr world,
        PhysiKComponentHandle component,
        int tetIndex,
        int active);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void PHYSIK_DeactivateTet(
        IntPtr world,
        PhysiKComponentHandle component,
        int tetIndex);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int PHYSIK_GetActiveTetCount(
        IntPtr world,
        PhysiKComponentHandle component);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int PHYSIK_GetTetMeshGlobalNodeBeginIndex(
        IntPtr world,
        PhysiKComponentHandle tetMeshPhysicsHandle);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int PHYSIK_GetTetMeshGlobalNodeCount(
        IntPtr world,
        PhysiKComponentHandle tetMeshPhysicsHandle);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int PHYSIK_GetTetMeshGlobalNodeIndex(
        IntPtr world,
        PhysiKComponentHandle tetMeshPhysicsHandle,
        int localNodeIndex);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int PHYSIK_IsComponentHandleValid(
        IntPtr world,
        PhysiKComponentHandle component);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void PHYSIK_GetNodePosition(
        IntPtr world,
        int nodeIndex,
        out float x,
        out float y,
        out float z);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void PHYSIK_GetNodeVelocity(
        IntPtr world,
        int nodeIndex,
        out float x,
        out float y,
        out float z);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void PHYSIK_SetNodePosition(
        IntPtr world,
        int nodeIndex,
        float x,
        float y,
        float z);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void PHYSIK_AddPointConnection(
    IntPtr world,
    int node0,
    int node1,
    int node2,
    int node3,
    float barycentricX,
    float barycentricY,
    float barycentricZ,
    float barycentricW,
    float targetX,
    float targetY,
    float targetZ,
    float stiffness,
    float damping);

    [StructLayout(LayoutKind.Sequential)]
    public struct PhysikCollisionSphereOverlap
    {
        public int geometryType;

        public PhysiKComponentHandle component;
        public int primitiveIndex;

        public int node0;
        public int node1;
        public int node2;
        public int node3;

        public int overlappedNodeMask;
        public int overlappedNodeCount;

        public float sphereCenterX;
        public float sphereCenterY;
        public float sphereCenterZ;
        public float sphereRadius;
        public float minDistance;
    }

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern PhysiKComponentHandle PHYSIK_CreateCollisionSphereComponent(
        IntPtr world,
        float x,
        float y,
        float z,
        float radius);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void PHYSIK_SetCollisionComponentKinematicTarget(
        IntPtr world,
        PhysiKComponentHandle component,
        float x,
        float y,
        float z);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int PHYSIK_GetCollisionSphereOverlapCount(
        IntPtr world,
        PhysiKComponentHandle sphereComponent);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int PHYSIK_GetCollisionSphereOverlaps(
        IntPtr world,
        PhysiKComponentHandle sphereComponent,
        [Out] PhysikCollisionSphereOverlap[] outOverlaps,
        int maxOverlaps);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void PHYSIK_DestroyComponent(
        IntPtr world,
        PhysiKComponentHandle component);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void PHYSIK_SetCollisionSphereConnectionSettings(
        IntPtr world,
        PhysiKComponentHandle sphereComponent,
        float stiffness,
        float damping);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void PHYSIK_GetCollisionSphereConnectionSettings(
        IntPtr world,
        PhysiKComponentHandle sphereComponent,
        out float stiffness,
        out float damping);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern PhysiKComponentHandle PHYSIK_CreateVisualMeshComponent(
    IntPtr world,
    PhysiKComponentHandle hostTetMesh);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void PHYSIK_SetVisualMeshData(
        IntPtr world,
        PhysiKComponentHandle visualMesh,
        Vec3[] vertices,
        int vertexCount,
        int[] triangleIndices,
        int triangleIndexCount);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int PHYSIK_BuildVisualMeshEmbedding(
        IntPtr world,
        PhysiKComponentHandle visualMesh);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int PHYSIK_GetVisualMeshVertexCount(
        IntPtr world,
        PhysiKComponentHandle visualMesh);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int PHYSIK_GetVisualMeshTriangleIndexCount(
            IntPtr world,
            PhysiKComponentHandle visualMesh);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int PHYSIK_CopyVisualMeshVertices(
            IntPtr world,
            PhysiKComponentHandle visualMesh,
            [Out] Vec3[] outVertices,
            int maxVertexCount);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int PHYSIK_CopyVisualMeshTriangleIndices(
            IntPtr world,
            PhysiKComponentHandle visualMesh,
            [Out] int[] outIndices,
            int maxIndexCount);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern PhysiKComponentHandle PHYSIK_CreateSurfaceExtractionComponent(
            IntPtr world,
            PhysiKComponentHandle hostTetMesh);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int PHYSIK_GetSurfaceTriangleIndexCount(
            IntPtr world,
            PhysiKComponentHandle surfaceExtraction);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int PHYSIK_CopySurfaceTriangleIndices(
            IntPtr world,
            PhysiKComponentHandle surfaceExtraction,
            [Out] int[] outIndices,
            int maxIndexCount);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern PhysiKComponentHandle PHYSIK_CreateSurfaceVisualComponent(
            System.IntPtr world,
            PhysiKComponentHandle surfaceExtractionHandle);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int PHYSIK_GetSurfaceVisualVertexCount(
            System.IntPtr world,
            PhysiKComponentHandle surfaceVisualHandle);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int PHYSIK_GetSurfaceVisualTriangleIndexCount(
            System.IntPtr world,
            PhysiKComponentHandle surfaceVisualHandle);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int PHYSIK_GetSurfaceVisualNormalCount(
            System.IntPtr world,
            PhysiKComponentHandle surfaceVisualHandle);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int PHYSIK_GetSurfaceVisualVertex(
            System.IntPtr world,
            PhysiKComponentHandle surfaceVisualHandle,
            int visualVertexIndex,
            out float outX,
            out float outY,
            out float outZ);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int PHYSIK_GetSurfaceVisualTriangleIndex(
            System.IntPtr world,
            PhysiKComponentHandle surfaceVisualHandle,
            int triangleIndexArrayIndex,
            out int outIndex);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int PHYSIK_GetSurfaceVisualNormal(
            System.IntPtr world,
            PhysiKComponentHandle surfaceVisualHandle,
            int visualNormalIndex,
            out float outX,
            out float outY,
            out float outZ);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void PHYSIK_SetConjugateGradientTolerance(
        IntPtr world,
        float tolerance);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern float PHYSIK_GetConjugateGradientTolerance(
        IntPtr world);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void PHYSIK_SetConjugateGradientMaxIterations(
        IntPtr world,
        int maxIterations);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int PHYSIK_GetConjugateGradientMaxIterations(
        IntPtr world);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int PHYSIK_GetLastConjugateGradientIterations(
        IntPtr world);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern float PHYSIK_GetLastConjugateGradientResidualNorm(
        IntPtr world);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int PHYSIK_DidLastConjugateGradientSolveConverge(
        IntPtr world);

    [UnmanagedFunctionPointer(
            CallingConvention.Cdecl)]
    public delegate void ExternalLogicCallback(
            IntPtr world,
            IntPtr userData);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern PhysiKComponentHandle
        PHYSIK_CreateScriptComponent(IntPtr world);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void
        PHYSIK_SetScriptComponentCallback(IntPtr world,
            PhysiKComponentHandle scriptComponent,
            ExternalLogicCallback callback,
            IntPtr userData);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void
        PHYSIK_ClearScriptComponentCallback(
            IntPtr world,
            PhysiKComponentHandle scriptComponent);

}