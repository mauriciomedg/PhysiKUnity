using System;
using UnityEngine;
using UnityEngine.InputSystem;
using PhysiK.Unity;
using static PhysiKNative;

[DefaultExecutionOrder(-200)]
public sealed class Physik_CutTool :
    Physik_ScriptComponent
{
    [Header("Mechanical Tissue")]
    [SerializeField]
    private Physik_MechanicalTissue tissue;

    [Header("Physical Interaction")]
    [SerializeField]
    private float connectionStiffness =
        10000.0f;

    [SerializeField]
    private float connectionDamping =
        0.0f;

    private PhysiKComponentHandle sphereComponent;

    private bool initialized;

    private bool possessed;

    public bool IsInitialized =>
        initialized;

    public bool IsPossessed =>
        possessed;

    private float RadiusFromTransform
    {
        get
        {
            Vector3 scale =
                transform.lossyScale;

            return Mathf.Max(
                       scale.x,
                       scale.y,
                       scale.z) *
                   0.5f;
        }
    }

    private void Awake()
    {
        gameObject.name =
            "Physik_CutTool";
    }

    private void OnValidate()
    {
        connectionStiffness =
            Mathf.Max(
                0.0f,
                connectionStiffness);

        connectionDamping =
            Mathf.Max(
                0.0f,
                connectionDamping);
    }

    private void Start()
    {
        TryInitialize();
    }

    private void Update()
    {
        if (!initialized)
        {
            TryInitialize();
        }
    }

    public void SetPossessed(
        bool isPossessed)
    {
        possessed =
            isPossessed;
    }

    private bool TryInitialize()
    {
        if (initialized)
        {
            return true;
        }

        if (tissue ==
            null)
        {
            tissue =
                FindFirstObjectByType<
                    Physik_MechanicalTissue>();
        }

        if (tissue ==
                null ||
            !tissue.IsInitialized ||
            tissue.WorldHandle ==
                IntPtr.Zero)
        {
            return false;
        }

        Vector3 position =
            transform.position;

        sphereComponent =
            PhysiKNative
                .PHYSIK_CreateCollisionSphereComponent(
                    tissue.WorldHandle,
                    position.x,
                    position.y,
                    position.z,
                    RadiusFromTransform);

        initialized =
            PhysiKNative
                .PHYSIK_IsComponentHandleValid(
                    tissue.WorldHandle,
                    sphereComponent) !=
            0;

        if (!initialized)
        {
            Debug.LogError(
                "Failed to create native CollisionSphereComponent for Physik_CutTool.",
                this);

            enabled =
                false;

            return false;
        }

        ApplyConnectionSettings();

        PushNativeSpherePosition();

        if (!TryInitializeNativeScriptComponent())
        {
            Debug.LogError(
                "Failed to initialize native ScriptComponent for Physik_CutTool.",
                this);

            PhysiKNative
                .PHYSIK_DestroyComponent(
                    tissue.WorldHandle,
                    sphereComponent);

            initialized =
                false;

            enabled =
                false;

            return false;
        }

        return true;
    }

    protected override IntPtr
        GetScriptWorldHandle()
    {
        return tissue !=
                null
            ? tissue.WorldHandle
            : IntPtr.Zero;
    }

    protected override bool
        CanInitializeScriptComponent()
    {
        return initialized &&
            tissue !=
                null &&
            tissue.IsInitialized &&
            tissue.WorldHandle !=
                IntPtr.Zero;
    }

    protected override void
        OnPhysikPreUpdate()
    {
        if (!initialized ||
            tissue ==
                null ||
            tissue.WorldHandle ==
                IntPtr.Zero)
        {
            return;
        }

        ApplyConnectionSettings();

        PushNativeSpherePosition();

        bool shouldCut =
            possessed &&
            Keyboard.current !=
                null &&
            Keyboard.current.cKey
                .isPressed;

        if (shouldCut)
        {
            CutOverlappingTets();
        }
    }

    private void ApplyConnectionSettings()
    {
        PhysiKNative
            .PHYSIK_SetCollisionSphereConnectionSettings(
                tissue.WorldHandle,
                sphereComponent,
                connectionStiffness,
                connectionDamping);
    }

    private void PushNativeSpherePosition()
    {
        Vector3 position =
            transform.position;

        PhysiKNative
            .PHYSIK_SetCollisionComponentKinematicTarget(
                tissue.WorldHandle,
                sphereComponent,
                position.x,
                position.y,
                position.z);
    }

    private void CutOverlappingTets()
    {
        int overlapCount =
            PhysiKNative
                .PHYSIK_GetCollisionSphereOverlapCount(
                    tissue.WorldHandle,
                    sphereComponent);

        if (overlapCount <=
            0)
        {
            return;
        }

        PhysikCollisionSphereOverlap[] overlaps =
            new PhysikCollisionSphereOverlap[
                overlapCount];

        int written =
            PhysiKNative
                .PHYSIK_GetCollisionSphereOverlaps(
                    tissue.WorldHandle,
                    sphereComponent,
                    overlaps,
                    overlaps.Length);

        for (int i = 0;
             i <
                 written;
             ++i)
        {
            if ((PhysikOverlapGeometryType)
                    overlaps[i]
                        .geometryType !=
                PhysikOverlapGeometryType
                    .Tetrahedron)
            {
                continue;
            }

            if (!SameHandle(
                    overlaps[i]
                        .component,
                    tissue
                        .TetMeshHandle))
            {
                continue;
            }

            tissue.DeactivateTet(
                overlaps[i]
                    .primitiveIndex);
        }
    }

    private static bool SameHandle(
        PhysiKComponentHandle a,
        PhysiKComponentHandle b)
    {
        return a.index ==
                b.index &&
            a.generation ==
                b.generation;
    }

    protected override void OnDestroy()
    {
        DestroyNativeScriptComponent();

        if (initialized &&
            tissue !=
                null &&
            tissue.WorldHandle !=
                IntPtr.Zero)
        {
            PhysiKNative
                .PHYSIK_DestroyComponent(
                    tissue.WorldHandle,
                    sphereComponent);
        }

        initialized =
            false;

        base.OnDestroy();
    }
}