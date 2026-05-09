using System.Runtime.InteropServices;
using System;
using UnityEngine;

public class PhysiKNative
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
        float z,
        float inverseMass);

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
}
