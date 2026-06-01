using System;
using UnityEngine;

public class PhysiK_ConjugateGradientService : MonoBehaviour
{
    private const float MinTolerance = 1.0e-8f;
    private const float MaxTolerance = 1.0e-1f;

    private const int MinMaxIterations = 1;
    private const int MaxMaxIterations = 1024;

    [Header("Conjugate Gradient Settings")]

    [SerializeField]
    [Tooltip("Relative residual threshold. Lower values improve accuracy but may require more iterations.")]
    private float tolerance = 1.0e-4f;

    [SerializeField]
    [Range(MinMaxIterations, MaxMaxIterations)]
    [Tooltip("Maximum number of CG iterations allowed for one linear solve.")]
    private int maxIterations = 128;

    [Header("Latest Native Solve Diagnostics")]

    [SerializeField]
    [Tooltip("Iterations used by the most recent native solve.")]
    private int lastIterations;

    [SerializeField]
    [Tooltip("Residual norm reported by the most recent native solve.")]
    private float lastResidualNorm;

    [SerializeField]
    [Tooltip("True when the most recent native solve reached the requested tolerance.")]
    private bool lastSolveConverged;

    [SerializeField]
    [Tooltip("True after a valid native world handle has been assigned.")]
    private bool isBound;

    [Header("Debug Logging")]

    [SerializeField]
    [Tooltip("Print CG diagnostics periodically in the console.")]
    private bool enableConsoleLogging = true;

    [SerializeField]
    [Min(0.1f)]
    [Tooltip("Time in seconds between console logs.")]
    private float logIntervalSeconds = 1.0f;

    private IntPtr worldHandle = IntPtr.Zero;

    private float lastAppliedTolerance = float.NaN;
    private int lastAppliedMaxIterations = -1;

    private float logTimer;

    public float Tolerance => tolerance;
    public int MaxIterations => maxIterations;

    public int LastIterations => lastIterations;
    public float LastResidualNorm => lastResidualNorm;
    public bool LastSolveConverged => lastSolveConverged;

    public bool IsBound =>
        isBound &&
        worldHandle != IntPtr.Zero;

    /// <summary>
    /// Call immediately after creating the native world.
    /// </summary>
    public void BindWorld(IntPtr nativeWorldHandle)
    {
        if (nativeWorldHandle == IntPtr.Zero)
        {
            Debug.LogError(
                $"{nameof(PhysiK_ConjugateGradientService)} received an invalid world handle.",
                this);

            UnbindWorld();
            return;
        }

        worldHandle = nativeWorldHandle;
        isBound = true;

        ApplySettings(force: true);
        RefreshSettingsFromNative();
        RefreshDiagnostics();
    }

    /// <summary>
    /// Call before destroying the native world.
    /// </summary>
    public void UnbindWorld()
    {
        worldHandle = IntPtr.Zero;
        isBound = false;

        lastAppliedTolerance = float.NaN;
        lastAppliedMaxIterations = -1;

        lastIterations = 0;
        lastResidualNorm = 0.0f;
        lastSolveConverged = false;

        logTimer = 0.0f;
    }

    public void SetTolerance(float newTolerance)
    {
        tolerance = ClampTolerance(newTolerance);
        ApplySettings(force: false);
    }

    public void SetMaxIterations(int newMaxIterations)
    {
        maxIterations = Mathf.Clamp(
            newMaxIterations,
            MinMaxIterations,
            MaxMaxIterations);

        ApplySettings(force: false);
    }

    /// <summary>
    /// Push inspector values to the native world.
    /// Values are sent only when changed unless force is true.
    /// </summary>
    public void ApplySettings(bool force = false)
    {
        ClampSerializedSettings();

        if (!IsBound)
        {
            return;
        }

        if (force || tolerance != lastAppliedTolerance)
        {
            PhysiKNative.PHYSIK_SetConjugateGradientTolerance(
                worldHandle,
                tolerance);

            tolerance =
                PhysiKNative.PHYSIK_GetConjugateGradientTolerance(
                    worldHandle);

            lastAppliedTolerance = tolerance;
        }

        if (force || maxIterations != lastAppliedMaxIterations)
        {
            PhysiKNative.PHYSIK_SetConjugateGradientMaxIterations(
                worldHandle,
                maxIterations);

            maxIterations =
                PhysiKNative.PHYSIK_GetConjugateGradientMaxIterations(
                    worldHandle);

            lastAppliedMaxIterations = maxIterations;
        }
    }

    /// <summary>
    /// Reads the native settings currently stored by the world.
    /// </summary>
    public void RefreshSettingsFromNative()
    {
        if (!IsBound)
        {
            return;
        }

        tolerance =
            PhysiKNative.PHYSIK_GetConjugateGradientTolerance(
                worldHandle);

        maxIterations =
            PhysiKNative.PHYSIK_GetConjugateGradientMaxIterations(
                worldHandle);

        lastAppliedTolerance = tolerance;
        lastAppliedMaxIterations = maxIterations;
    }

    /// <summary>
    /// Reads diagnostics from the latest completed native solve.
    /// Call immediately after stepping the native simulation.
    /// </summary>
    public void RefreshDiagnostics()
    {
        if (!IsBound)
        {
            return;
        }

        lastIterations =
            PhysiKNative.PHYSIK_GetLastConjugateGradientIterations(
                worldHandle);

        lastResidualNorm =
            PhysiKNative.PHYSIK_GetLastConjugateGradientResidualNorm(
                worldHandle);

        lastSolveConverged =
            PhysiKNative.PHYSIK_DidLastConjugateGradientSolveConverge(
                worldHandle) != 0;
    }

    /// <summary>
    /// Call after RefreshDiagnostics using the simulation delta time.
    /// Logs at a controlled frequency.
    /// </summary>
    public void LogDiagnostics(float simulationDt)
    {
        if (!enableConsoleLogging || !IsBound)
        {
            return;
        }

        logTimer += simulationDt;

        if (logTimer < logIntervalSeconds)
        {
            return;
        }

        logTimer = 0.0f;

        Debug.Log(
            $"CG | tolerance: {tolerance:E3} | " +
            $"max iterations: {maxIterations} | " +
            $"last iterations: {lastIterations} | " +
            $"residual: {lastResidualNorm:E3} | " +
            $"converged: {lastSolveConverged}",
            this);
    }

    private void Update()
    {
        ApplySettings(force: false);
    }

    private void OnValidate()
    {
        ClampSerializedSettings();

        logIntervalSeconds =
            Mathf.Max(0.1f, logIntervalSeconds);
    }

    private void ClampSerializedSettings()
    {
        tolerance = ClampTolerance(tolerance);

        maxIterations = Mathf.Clamp(
            maxIterations,
            MinMaxIterations,
            MaxMaxIterations);
    }

    private static float ClampTolerance(float value)
    {
        if (float.IsNaN(value) || float.IsInfinity(value))
        {
            return 1.0e-4f;
        }

        return Mathf.Clamp(
            value,
            MinTolerance,
            MaxTolerance);
    }
}