using System;
using System.Runtime.InteropServices;

[StructLayout(LayoutKind.Sequential)]
public struct PhysiKComponentHandle
{
    public uint index;
    public uint generation;

    public bool IsValid => index != 0xFFFFFFFFu && generation != 0u;
}

public static class PhysiKNative
{
    private const string DllName = "PhysiK";

    [DllImport(DllName)]
    public static extern IntPtr PHYSIK_CreateWorld();

    [DllImport(DllName)]
    public static extern void PHYSIK_DestroyWorld(IntPtr world);

    [DllImport(DllName)]
    public static extern void PHYSIK_Step(IntPtr world, float dt);

    [DllImport(DllName)]
    public static extern void PHYSIK_SetSubstepCount(IntPtr world, int substepCount);

    [DllImport(DllName)]
    public static extern void PHYSIK_SetSolverMode(IntPtr world, int mode);

    [DllImport(DllName)]
    public static extern void PHYSIK_SetGravity(
        IntPtr world,
        float x,
        float y,
        float z);

    [DllImport(DllName)]
    public static extern int PHYSIK_AddNode(
        IntPtr world,
        float x,
        float y,
        float z);

    [DllImport(DllName)]
    public static extern PhysiKComponentHandle PHYSIK_CreateTetMeshComponent(
        IntPtr world,
        int[] nodeIndices,
        int nodeCount,
        int[] tetNodeIndices,
        int tetCount);

    [DllImport(DllName)]
    public static extern int PHYSIK_IsComponentHandleValid(
        IntPtr world,
        PhysiKComponentHandle component);

    [DllImport(DllName)]
    public static extern void PHYSIK_GetNodePosition(
        IntPtr world,
        int nodeIndex,
        out float x,
        out float y,
        out float z);

    [DllImport(DllName)]
    public static extern void PHYSIK_GetNodeVelocity(
        IntPtr world,
        int nodeIndex,
        out float x,
        out float y,
        out float z);

    [DllImport(DllName)]
    public static extern void PHYSIK_SetNodePosition(
        IntPtr world,
        int nodeIndex,
        float x,
        float y,
        float z);

    [DllImport(DllName)]
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
}