using System;
using System.Collections.Generic;
using UnityEngine;
using PhysiK.Unity;

public interface IPhysikWorldParticipant
{
    void OnPhysikBeforeSimulationStep(float dt);

    void OnPhysikAfterSimulationFrame();

    void OnPhysikWorldDestroyed();
}

public class Physik_World : MonoBehaviour
{
    [Header("PhysiK")]
    [SerializeField] private int substeps = 8;
    [SerializeField] private bool useImplicitEuler = true;

    [Header("Simulation Loop")]
    [SerializeField] private float simulationDt = 1.0f / 30.0f;
    [SerializeField] private int maxSimulationStepsPerFrame = 1;

    [Header("Gravity")]
    [SerializeField]
    private Vector3 gravity =
        new Vector3(0.0f, -9.81f, 0.0f);

    [SerializeField] private bool applyGravityEveryStep = true;

    [Header("CG")]
    [SerializeField]
    private PhysiK_ConjugateGradientService conjugateGradientService;

    private readonly List<IPhysikWorldParticipant> participants =
        new List<IPhysikWorldParticipant>();

    private IntPtr world = IntPtr.Zero;
    private float simulationAccumulator;

    public bool IsInitialized =>
        world != IntPtr.Zero;

    public IntPtr WorldHandle =>
        world;

    private void Awake()
    {
        world =
            PhysiKNative.PHYSIK_CreateWorld();

        if (world == IntPtr.Zero)
        {
            Debug.LogError(
                "Failed to create PhysiK world.",
                this);

            enabled = false;
            return;
        }

        if (conjugateGradientService != null)
        {
            conjugateGradientService.BindWorld(
                world);
        }
        else
        {
            Debug.LogWarning(
                "No PhysiK_ConjugateGradientService is assigned to the PhysiK world.",
                this);
        }

        ApplyConfigurationToNative();
    }

    private void Update()
    {
        if (world == IntPtr.Zero)
        {
            return;
        }

        float safeSimulationDt =
            Mathf.Max(
                1.0e-6f,
                simulationDt);

        int safeMaxSimulationStepsPerFrame =
            Mathf.Max(
                1,
                maxSimulationStepsPerFrame);

        simulationAccumulator +=
            Time.deltaTime;

        int steps =
            0;

        while (simulationAccumulator >= safeSimulationDt &&
               steps < safeMaxSimulationStepsPerFrame)
        {
            StepSimulation(
                safeSimulationDt);

            simulationAccumulator -=
                safeSimulationDt;

            ++steps;
        }

        if (steps == safeMaxSimulationStepsPerFrame)
        {
            simulationAccumulator =
                0.0f;
        }

        for (int i = 0;
             i < participants.Count;
             ++i)
        {
            participants[i]
                .OnPhysikAfterSimulationFrame();
        }
    }

    private void StepSimulation(float dt)
    {
        if (applyGravityEveryStep)
        {
            ApplyGravityToNative();
        }

        for (int i = 0;
             i < participants.Count;
             ++i)
        {
            participants[i]
                .OnPhysikBeforeSimulationStep(
                    dt);
        }

        PhysiKNative.PHYSIK_Step(
            world,
            dt);

        if (conjugateGradientService != null)
        {
            conjugateGradientService.RefreshDiagnostics();
            conjugateGradientService.LogDiagnostics(
                dt);
        }
    }

    public void RegisterParticipant(
        IPhysikWorldParticipant participant)
    {
        if (participant == null ||
            participants.Contains(participant))
        {
            return;
        }

        participants.Add(
            participant);
    }

    public void UnregisterParticipant(
        IPhysikWorldParticipant participant)
    {
        if (participant == null)
        {
            return;
        }

        participants.Remove(
            participant);
    }

    private void ApplyConfigurationToNative()
    {
        if (world == IntPtr.Zero)
        {
            return;
        }

        PhysiKNative.PHYSIK_SetSubstepCount(
            world,
            Mathf.Max(
                1,
                substeps));

        PhysiKNative.PHYSIK_SetSolverMode(
            world,
            useImplicitEuler ? 1 : 0);

        ApplyGravityToNative();
    }

    private void ApplyGravityToNative()
    {
        if (world == IntPtr.Zero)
        {
            return;
        }

        PhysiKNative.PHYSIK_SetGravity(
            world,
            gravity.x,
            gravity.y,
            gravity.z);
    }

    private void OnValidate()
    {
        substeps =
            Mathf.Max(
                1,
                substeps);

        simulationDt =
            Mathf.Max(
                1.0e-6f,
                simulationDt);

        maxSimulationStepsPerFrame =
            Mathf.Max(
                1,
                maxSimulationStepsPerFrame);

        if (Application.isPlaying)
        {
            ApplyConfigurationToNative();
        }
    }

    [ContextMenu("Apply Configuration To Native")]
    private void ApplyConfigurationContextMenu()
    {
        if (!Application.isPlaying)
        {
            Debug.Log(
                "Enter Play Mode first. Native world does not exist yet.",
                this);

            return;
        }

        ApplyConfigurationToNative();

        Debug.Log(
            $"PhysiK world configuration applied. " +
            $"substeps={substeps}, " +
            $"useImplicitEuler={useImplicitEuler}, " +
            $"simulationDt={simulationDt:F6}, " +
            $"maxSimulationStepsPerFrame={maxSimulationStepsPerFrame}, " +
            $"gravity=({gravity.x:F6}, {gravity.y:F6}, {gravity.z:F6}).",
            this);
    }

    [ContextMenu("Reset Gravity To Unity Default")]
    private void ResetGravityToUnityDefault()
    {
        gravity =
            new Vector3(
                0.0f,
                -9.81f,
                0.0f);

        if (Application.isPlaying)
        {
            ApplyGravityToNative();
        }

        Debug.Log(
            $"Gravity reset to default: " +
            $"({gravity.x:F6}, {gravity.y:F6}, {gravity.z:F6})",
            this);
    }

    private void OnDestroy()
    {
        for (int i = participants.Count - 1;
             i >= 0;
             --i)
        {
            participants[i]
                .OnPhysikWorldDestroyed();
        }

        participants.Clear();

        if (conjugateGradientService != null)
        {
            conjugateGradientService.UnbindWorld();
        }

        if (world != IntPtr.Zero)
        {
            PhysiKNative.PHYSIK_DestroyWorld(
                world);

            world =
                IntPtr.Zero;
        }
    }
}
